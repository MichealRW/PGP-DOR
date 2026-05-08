#ifndef DYNAMICMAP_H
#define DYNAMICMAP_H

//#include "ikd_Tree.h"
#include <thread>
#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/octree/octree_search.h>
#include "misc/Utils.h"

#include "CExpandableMap.h"

#include "opencv2/opencv.hpp"
#include "misc/CoordTransformation.h"
//#include "misc/BGK_SmootherComplex.h"
//#include "CTerrainAnalysis/CAccumedTerrainMapper.h"
#include "CTerrainAnalysis/CInstantTerrainMapper.h"
#include "CTerrainAnalysis/CAccumedTerrainMapper.h"
#include "misc/Visualize.h"


#define OBSERLUTE_HEIGHT 2.5
#define MIN_CLOUDS_IN 12
#define BIG_BEISHU 10
#define DEEP_ABSORT 0.4
#define MAX_DYNAMIC 1.6
#define OBSTACLE_BEISHU 3
#define MAX_SCORE (1 * MIN_CLOUDS_IN + MIN_CLOUDS_IN * (MIN_CLOUDS_IN - 1) / 2)

class PNtestModel{
    public:
    PNtestModel(){
        Tp_this = 0;
        Tn_this = 0;
        Fp_this = 0;
        Fn_this = 0;

        Tp_total = 0;
        Tn_total = 0;
        Fp_total = 0;
        Fn_total = 0;
    };
    ~PNtestModel(){

    };

    void AddObservation(int pn_mode){
        switch (pn_mode)
        {
        case 0:
            Tp_this++;
            break;
        case 1:
            Tn_this++;
            break;
        case 2:
            Fp_this++;
            break;
        case 3:
            Fn_this++;
            break;
        
        default:
            break;
        }
    };
    void ReSetThis(){
        Tp_total += Tp_this;
        Tn_total += Tn_this;
        Fp_total += Fp_this;
        Fn_total += Fn_this;
        Tp_this = 0;
        Tn_this = 0;
        Fp_this = 0;
        Fn_this = 0;
    };

    float ReturnPrecision_this(){
        return (float)Tp_this / (Tp_this + Fp_this);
    };
    float ReturnRecall_this(){
        return (float)Tp_this / (Tp_this + Fn_this);
    };
    float ReturnACC_this(){
        return (float)(Tp_this + Tn_this) / (Tp_this + Tn_this + Fp_this + Fn_this);
    };
    float ReturnFalsePositiveRate_this(){
        return (float)Fp_this / (Fp_this + Tn_this);
    }

    float ReturnPrecision_total(){
        return (float)Tp_total / (Tp_total + Fp_total);
    };
    float ReturnRecall_total(){
        return (float)Tp_total / (Tp_total + Fn_total);
    };
    float ReturnACC_total(){
        return (float)(Tp_total + Tn_total) / (Tp_total + Tn_total + Fp_total + Fn_total);
    };
    float ReturnFalsePositiveRate_total(){
        return (float)Fp_total / (Fp_total + Tn_total);
    }

    int Tp_this, Tn_this;
    int Fp_this, Fn_this;

    int Tp_total, Tn_total;
    int Fp_total, Fn_total;
};

class CGlobalGridCell {
public:
    CGlobalGridCell(/*double reference_height_*/);
    ~CGlobalGridCell();

    void AddObservation(float ob_global_point_height, int current_overlap, int num_clouds_in);
    void SetTerrainHeight(double terrain_height_);
    void TerrainObservation();
    void NoObCheck();
    void NoObCheckV2();
    void ReBool();

    float ob_min_height;
    double terrain_height;
    //double reference_height;
    //bool no_obcheck;
    bool p_value;
    bool observation_this;
    bool terrain_in;
    int n;
};

class CGlobalGridCellV2 {
public:
    CGlobalGridCellV2(/*double reference_height_*/);
    ~CGlobalGridCellV2();

    void AddObservation(float ob_global_point_height, int current_overlap, int num_clouds_in);
    void OverCheck();
    void OverCheckV2();
    void SetTerrainHeight(double terrain_height_);
    void TerrainObservation();

    void ReBool();

    float ob_min_height;
    double terrain_height;
    
    bool p_value;
    bool curr_state;
    bool last_state;
    bool observation_this;
    bool terrain_in;

    int num_pt;
    int conti_n;
    int max_overlap;
    int n;
};

class DynamicMap
{
public:

    DynamicMap();
    ~DynamicMap();
    //int Mode_in = 0;
    std::mutex mtx_point;
    //KD_TREE ikdtree_map;
    //std::deque<KD_TREE> ikdtree_map_buffer;
    int points_in;
    void SetAccumedTerrainMapper(CAccumedTerrainMapper *accumed_terrain_mapper_);
    void SetInstantTerrainMapper(CInstantTerrainMapper *instant_terrain_mapper_);
    CAccumedTerrainMapper *accumed_terrain_mapper;
    CInstantTerrainMapper *instant_terrain_mapper;
    //void SetAccumedTerrainMapper(CAccumedTerrainMapper *accumed_terrain_mapper_);
    //CAccumedTerrainMapper *accumed_terrain_mapper;
    void SetReferenceHeight(double reference_height_);
    double reference_height;
    void SetMapResolution(double map_resolution_);
    void SetROIRange(float roi_range_);
    float roi_range;
    void SetMapSize(int map_rows_, int map_cols_);
    double map_resolution;
    int map_rows, map_cols;
    float inital_center_x, inital_center_y;
    double inital_residual_x, inital_residual_y, current_residual_x, current_residual_y;
    void SetStartFrame(int start_frame_);
    int start_frame;

    void SetBGKSmoother(BGK_Smoother *bgk_smoother_);
    BGK_Smoother *bgk_smoother; 

    void SetCheckmode(int mode_);
    int check_mode;

    void Initialize();

    int Big_x_bei, Big_y_bei;
    void OverBoundaryCheck();
    void ChangeBigMat(int flag);
    //////////////////////////// V2 ///////////////////////
    
    CExpandableMap<CGlobalGridCell> *big_global_ob_map;
    CExpandableMap<CGlobalGridCellV2> *big_global_ob_map2;
    void PointsInOCV2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void PointsInOCV3(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void DynamicVoteOCV2(pcl::PointCloud<pcl::PointXYZ>::Ptr ob_cld, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void DynamicVoteOCV3(pcl::PointCloud<pcl::PointXYZ>::Ptr ob_cld, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void SetTwlRwlV2(Eigen::Vector3d T_w_l, Eigen::Matrix3d R_w_l);

    void InitializeV2();
    void LoadMatV2();
    void LoadBigTerrainMat();
    void DLoadBigTerrainMat();
    void DLoadMatV2();

    void OverBoundaryCheckV2();
    void ChangeBigMatV2(int flag);

    void VisualizeDynamicMat(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_terrain);
    void VisualizeDynamicMatkitti(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_terrain, std::string output_path);
    //// test ////

    PNtestModel *pn_test_model;
    void TestModel_All(cv::Mat &test_model, int offset_r, int offset_c);

    ///////////////////////////////////////////////////////
    void SetTwlRwl(Eigen::Vector3d T_w_l, Eigen::Matrix3d R_w_l);
    Eigen::Vector3d t_w_l;
    Eigen::Matrix3d r_w_l;
    //void PointsInIKD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void PointsInOC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    float StdMatchOut(std::vector<float> distance);
    //void DynamicVote(pcl::PointCloud<pcl::PointXYZ>::Ptr ob_cld, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void DynamicVoteOC(pcl::PointCloud<pcl::PointXYZ>::Ptr ob_cld, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void Addpoint2clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr dy_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndy_points;

    cv::Mat big_ndy_map_mat;
    cv::Mat big_ndy_term_map_mat;
    cv::Mat big_terrain_map_mat;

    cv::Mat ndy_predict_map_mat;
    cv::Mat small_ndy_map_mat;
    cv::Mat predicted_terrain_height_mat;

    // Test
    cv::Mat dynamic_mat;
    cv::Mat dynamic_big_mat;

    void ResetIntermediateVariables();
    std::deque<pcl::octree::OctreePointCloud<pcl::PointXYZ>> ndy_octree_buffer;
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> sequence_clouds_buffer;

    void LoadMat();
    void DLoadMat();
    bool no_storage_in;

    int nums_vis = 0;

    void VisualizeDynamicMap(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, bool b_pause);
    pcl::visualization::PCLVisualizer *vis;

    void SetTimes(float dy_times_);
    float dy_times;

    ///////////////////////// test ////////////////////////
    INT64 timestamp_this;
    double splot[6];
    void Output_splot();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dy_points_tmp;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ndy_points_tmp;
    void OutPutMap();
    void OutPutMap(std::string output_path);

    //////////////////////// vec //////////////////////////
// 计算目标速度和方向（调整为实际单位）
cv::Point2f calculateVelocity(cv::Point2f previous, cv::Point2f current, float time_interval) {
    // 每个栅格代表0.2米
    float grid_size = 0.2f;
    return (current - previous) * grid_size / time_interval;  // 转换为实际速度单位（米/秒）
};

// 计算目标方向
float calculateDirection(cv::Point2f velocity) {
    return atan2(velocity.y, velocity.x) * 180.0 / CV_PI;  // 以度为单位
};

// 在标记时转换为实际单位
void visualizeResult(cv::Mat& frame, const std::vector<cv::Rect>& target_boxes, const std::vector<cv::Point2f>& velocities, const std::vector<float>& directions) {
    for (size_t i = 0; i < target_boxes.size(); i++) {
        // 绘制边框
        cv::rectangle(frame, target_boxes[i], cv::Scalar(0, 0, 255), 2);
        
        // 转换为实际速度单位，并显示速度和方向
        float speed = cv::norm(velocities[i]); // 像素单位
        float actual_speed = speed * 0.2f;  // 转换为米/秒
        cv::putText(frame, "Speed: " + std::to_string(actual_speed) + " m/s", target_boxes[i].tl(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
        cv::putText(frame, "Direction: " + std::to_string(directions[i]) + " degrees", target_boxes[i].br(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    }

    // 显示标记后的图像
    cv::imshow("Tracked Dynamic Targets", frame);
    cv::waitKey(1); // 等待1ms并刷新图像窗口
};

// 处理单帧点云图像，检测动态目标并画框
void processFrame(const cv::Mat& frame, std::vector<cv::Rect>& target_boxes, std::vector<cv::Point2f>& target_centers) {
    cv::Mat gray, thresh, edges;

    // 检查是否能成功转换为灰度图
    if (frame.empty()) {
        std::cerr << "Error: Input frame is empty!" << std::endl;
        return;
    }
    
    // 转为灰度图
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 确保灰度图不为空
    if (gray.empty()) {
        std::cerr << "Error: Gray image conversion failed!" << std::endl;
        return;
    }

    // 阈值分割，提取动态目标
    cv::threshold(gray, thresh, 50, 255, cv::THRESH_BINARY);

    // 轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 提取目标边框
    target_boxes.clear();
    target_centers.clear();
    for (size_t i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > 100) {  // 排除小区域，阈值可根据需要调整
            cv::Rect bounding_box = cv::boundingRect(contours[i]);
            target_boxes.push_back(bounding_box);
            target_centers.push_back(cv::Point2f(bounding_box.x + bounding_box.width / 2, bounding_box.y + bounding_box.height / 2));
        }
    }
};

// 处理图像进行腐蚀、膨胀操作
cv::Mat preprocessImage(cv::Mat frame) {
    // 转为灰度图
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 腐蚀操作：缩小目标区域
    cv::Mat eroded;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 5x5结构元素
    cv::erode(gray, eroded, element);

    // 膨胀操作：恢复目标区域
    cv::Mat dilated;
    cv::dilate(eroded, dilated, element);

    // 重新赋值回原图
    return dilated;
}

};

#endif