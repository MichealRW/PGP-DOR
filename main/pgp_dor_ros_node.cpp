#include "CRemoveDynamic/CRemoveDynamicV2.h"

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr double SYNC_TOLERANCE_MS = 50.0;

struct TopicConfig {
    std::string input_pointcloud_topic = "/sensor/cloud_registered";
    std::string input_odometry_topic = "/self_state/LidarOdometry";
    std::string output_labeled_pointcloud_topic = "/pgp_dor_open/labeled_cloud";
};

struct GlobalConfig {
    std::filesystem::path lidar_data_path;
};

std::filesystem::path ResolveExecutableDirectory(const char *argv_0)
{
    std::error_code error_code;
    std::filesystem::path executable_path = std::filesystem::canonical(argv_0, error_code);
    if (error_code) {
        executable_path = std::filesystem::absolute(argv_0, error_code);
    }
    if (error_code) {
        executable_path = std::filesystem::current_path() / argv_0;
    }
    return executable_path.parent_path();
}

TopicConfig ReadTopicConfig(const std::filesystem::path &config_path)
{
    TopicConfig topic_config;
    std::ifstream fin(config_path);
    if (!fin.is_open()) {
        throw std::runtime_error("Fail to open topic config: " + config_path.string());
    }

    std::string token;
    while (fin >> token) {
        if (token.empty()) {
            continue;
        }
        if (token[0] == '#' || token[0] == '/') {
            std::getline(fin, token);
            continue;
        }
        if (token == "input_pointcloud_topic") {
            fin >> topic_config.input_pointcloud_topic;
            continue;
        }
        if (token == "input_odometry_topic") {
            fin >> topic_config.input_odometry_topic;
            continue;
        }
        if (token == "output_labeled_pointcloud_topic") {
            fin >> topic_config.output_labeled_pointcloud_topic;
            continue;
        }
    }

    return topic_config;
}

GlobalConfig ReadGlobalConfig(const std::filesystem::path &config_path)
{
    GlobalConfig global_config;
    std::ifstream fin(config_path);
    if (!fin.is_open()) {
        throw std::runtime_error("Fail to open global config: " + config_path.string());
    }

    std::string token;
    while (fin >> token) {
        if (token.empty()) {
            continue;
        }
        if (token[0] == '#' || token[0] == '/') {
            std::getline(fin, token);
            continue;
        }
        if (token == "m_LoadPath") {
            fin >> global_config.lidar_data_path;
        }
    }

    if (global_config.lidar_data_path.empty()) {
        throw std::runtime_error("m_LoadPath is missing in " + config_path.string());
    }

    return global_config;
}

class DynamicMapRosNode {
public:
    DynamicMapRosNode(
        ros::NodeHandle &node_handle,
        const std::filesystem::path &lidar_data_path,
        const std::filesystem::path &remove_dynamic_param_path,
        const TopicConfig &topic_config)
        : node_handle_(node_handle),
          lidar_data_path_(lidar_data_path),
          topic_config_(topic_config)
    {
        remove_dynamic_.SetLiDARDataPath(lidar_data_path_.string());
        remove_dynamic_.SetStartFrame(0);
        remove_dynamic_.SetParamFile(remove_dynamic_param_path.string());
        remove_dynamic_.Initialize();

        labeled_cloud_publisher_ =
            node_handle_.advertise<sensor_msgs::PointCloud2>(
                topic_config_.output_labeled_pointcloud_topic, 2);
        ROS_INFO(
            "DynamicMap ROS topics: cloud=%s odom=%s output=%s",
            topic_config_.input_pointcloud_topic.c_str(),
            topic_config_.input_odometry_topic.c_str(),
            topic_config_.output_labeled_pointcloud_topic.c_str());

        odometry_subscriber_ =
            node_handle_.subscribe<nav_msgs::Odometry>(
                topic_config_.input_odometry_topic, 2000,
                &DynamicMapRosNode::OdometryCallback, this);
        pointcloud_subscriber_ =
            node_handle_.subscribe<sensor_msgs::PointCloud2>(
                topic_config_.input_pointcloud_topic, 100,
                &DynamicMapRosNode::PointCloudCallback, this);
    }

    ~DynamicMapRosNode()
    {
        remove_dynamic_.LoadAll();
        remove_dynamic_.EndOut();
    }

    void RunLoop()
    {
        ros::Rate rate(100);
        while (ros::ok()) {
            ros::spinOnce();
            ProcessPendingClouds();
            rate.sleep();
        }
    }

private:
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_message)
    {
        std::lock_guard<std::mutex> lock(pointcloud_mutex_);
        pointcloud_queue_.push_back(*pointcloud_message);
        while (pointcloud_queue_.size() > 20) {
            pointcloud_queue_.pop_front();
        }
    }

    void OdometryCallback(const nav_msgs::Odometry::ConstPtr &odometry_message)
    {
        std::lock_guard<std::mutex> lock(odometry_mutex_);
        odometry_queue_.push_back(*odometry_message);
        while (odometry_queue_.size() > 2000) {
            odometry_queue_.pop_front();
        }
    }

    bool FindMatchedOdometry(const ros::Time &cloud_stamp, nav_msgs::Odometry &matched_odometry)
    {
        const double cloud_time_ms = cloud_stamp.toSec() * 1000.0;
        std::lock_guard<std::mutex> lock(odometry_mutex_);

        while (!odometry_queue_.empty()) {
            const double odometry_time_ms = odometry_queue_.front().header.stamp.toSec() * 1000.0;
            if (odometry_time_ms < cloud_time_ms - SYNC_TOLERANCE_MS) {
                odometry_queue_.pop_front();
                continue;
            }
            if (odometry_time_ms > cloud_time_ms + SYNC_TOLERANCE_MS) {
                return false;
            }
            matched_odometry = odometry_queue_.front();
            return true;
        }

        return false;
    }

    void ProcessPendingClouds()
    {
        sensor_msgs::PointCloud2 cloud_message;
        {
            std::lock_guard<std::mutex> lock(pointcloud_mutex_);
            if (pointcloud_queue_.empty()) {
                return;
            }
            cloud_message = pointcloud_queue_.front();
        }

        nav_msgs::Odometry matched_odometry;
        if (!FindMatchedOdometry(cloud_message.header.stamp, matched_odometry)) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(pointcloud_mutex_);
            if (pointcloud_queue_.empty()) {
                return;
            }
            cloud_message = pointcloud_queue_.front();
            pointcloud_queue_.pop_front();
        }

        ProcessFrame(cloud_message, matched_odometry);
    }

    void ProcessFrame(
        const sensor_msgs::PointCloud2 &cloud_message,
        const nav_msgs::Odometry &odometry_message)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(cloud_message, *local_cloud);

        const Eigen::Vector3d t_w_l(
            odometry_message.pose.pose.position.x,
            odometry_message.pose.pose.position.y,
            odometry_message.pose.pose.position.z);

        const Eigen::Quaterniond quaternion_w_l(
            odometry_message.pose.pose.orientation.w,
            odometry_message.pose.pose.orientation.x,
            odometry_message.pose.pose.orientation.y,
            odometry_message.pose.pose.orientation.z);
        const Eigen::Matrix3d r_w_l = quaternion_w_l.normalized().toRotationMatrix();

        pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        global_cloud->reserve(local_cloud->size());
        for (const pcl::PointXYZI &point_local : local_cloud->points) {
            pcl::PointXYZI point_global = point_local;
            const Eigen::Vector3d local_point(point_local.x, point_local.y, point_local.z);
            const Eigen::Vector3d world_point = r_w_l * local_point + t_w_l;
            point_global.x = static_cast<float>(world_point.x());
            point_global.y = static_cast<float>(world_point.y());
            point_global.z = static_cast<float>(world_point.z());
            global_cloud->push_back(point_global);
        }

        remove_dynamic_.Run(global_cloud, t_w_l, r_w_l, frame_index_);
        PublishLabeledCloud(cloud_message.header);
        ++frame_index_;
    }

    void PublishLabeledCloud(const std_msgs::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZI> labeled_cloud;
        labeled_cloud.reserve(
            remove_dynamic_.output_nody_pts->size() +
            remove_dynamic_.dynamicmap_rm->dy_points->size());

        for (const pcl::PointXYZ &point_static : remove_dynamic_.output_nody_pts->points) {
            pcl::PointXYZI point_labeled;
            point_labeled.x = point_static.x;
            point_labeled.y = point_static.y;
            point_labeled.z = point_static.z;
            point_labeled.intensity = 0.0f;
            labeled_cloud.push_back(point_labeled);
        }

        for (const pcl::PointXYZ &point_dynamic : remove_dynamic_.dynamicmap_rm->dy_points->points) {
            pcl::PointXYZI point_labeled;
            point_labeled.x = point_dynamic.x;
            point_labeled.y = point_dynamic.y;
            point_labeled.z = point_dynamic.z;
            point_labeled.intensity = 1.0f;
            labeled_cloud.push_back(point_labeled);
        }

        sensor_msgs::PointCloud2 labeled_cloud_message;
        pcl::toROSMsg(labeled_cloud, labeled_cloud_message);
        labeled_cloud_message.header = header;
        labeled_cloud_publisher_.publish(labeled_cloud_message);
    }

    ros::NodeHandle node_handle_;
    std::filesystem::path lidar_data_path_;
    TopicConfig topic_config_;

    ros::Subscriber pointcloud_subscriber_;
    ros::Subscriber odometry_subscriber_;
    ros::Publisher labeled_cloud_publisher_;

    RemoveDynamicV2 remove_dynamic_;

    std::mutex pointcloud_mutex_;
    std::mutex odometry_mutex_;
    std::deque<sensor_msgs::PointCloud2> pointcloud_queue_;
    std::deque<nav_msgs::Odometry> odometry_queue_;

    INT64 frame_index_ = 0;
};

}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pgp_dor_ros_node");

    try {
        const std::filesystem::path executable_dir = ResolveExecutableDirectory(argv[0]);
        std::filesystem::current_path(executable_dir);

        const std::filesystem::path parameter_dir = executable_dir / "parameters";
        const std::filesystem::path global_config_path =
            parameter_dir / "pgp_dor_global.ini";
        const std::filesystem::path remove_dynamic_param_path =
            parameter_dir / "pgp_dor_remove_dynamic.ini";
        const std::filesystem::path topic_config_path =
            parameter_dir / "pgp_dor_ros_topics.ini";

        ros::NodeHandle node_handle;
        const TopicConfig topic_config = ReadTopicConfig(topic_config_path);
        const GlobalConfig global_config = ReadGlobalConfig(global_config_path);
        DynamicMapRosNode dynamic_map_ros_node(
            node_handle,
            global_config.lidar_data_path,
            remove_dynamic_param_path,
            topic_config);
        dynamic_map_ros_node.RunLoop();
    } catch (const std::exception &exception) {
        ROS_ERROR("%s", exception.what());
        return 1;
    }

    return 0;
}
