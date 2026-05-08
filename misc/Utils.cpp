


#include "Utils.h"


const UINT16 ones_LUT[16] = {1, 2, 4, 8, 16, 32, 64, 128,
                                   256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
const UINT16 not_ones_LUT[16] = {65535-1, 65535-2, 65535-4, 65535-8, 65535-16, 65535-32, 65535-64, 65535-128,
                                   65535-256, 65535-512, 65535-1024, 65535-2048, 65535-4096, 65535-8192, 65535-16384, 65535-32768};


const UINT32 ones_32_LUT[32] = {1, 2, 4, 8, 16, 32, 64, 128,
                                256, 512, 1024, 2048, 4096, 8192, 16384, 32768,
                               65536, 131072, 262144, 524288, 1048576, 2097152, 4194304, 8388608,
                               16777216, 33554432, 67108864, 134217728, 268435456, 536870912, 1073741824, 2147483648};
const UINT32 not_ones_32_LUT[32] = {4294967295-1, 4294967295-2, 4294967295-4, 4294967295-8, 4294967295-16, 4294967295-32, 4294967295-64, 4294967295-128,
                                    4294967295-256, 4294967295-512, 4294967295-1024, 4294967295-2048, 4294967295-4096, 4294967295-8192, 4294967295-16384, 4294967295-32768,
                                    4294967295-65536, 4294967295-131072, 4294967295-262144, 4294967295-524288, 4294967295-1048576, 4294967295-2097152, 4294967295-4194304, 4294967295-8388608,
                                    4294967295-16777216, 4294967295-33554432, 4294967295-67108864, 4294967295-134217728, 4294967295-268435456, 4294967295-536870912, 4294967295-1073741824, 4294967295-2147483648};

void WriteMat(std::string fileName, cv::Mat &src)
{
    MatHeader matHeader{src.rows, src.cols, src.type()};

    std::ofstream out(fileName, std::ios::binary);

    out.write((char *) &matHeader, sizeof(MatHeader));
    out.write((char *) src.data, src.rows * src.cols * src.elemSize());

    out.flush();
    out.close();
}

cv::Mat ReadMat(std::string fileName)
{

    std::ifstream in(fileName, std::ios::binary);

    MatHeader matHeader{0, 0, 0};
    in.read((char *) &matHeader, sizeof(MatHeader));

    cv::Mat mat(matHeader.rows, matHeader.cols, matHeader.type);
    in.read((char *) mat.data, mat.rows * mat.cols * mat.elemSize());

    in.close();

    return mat;
}

void VoxelizeWhilePreservingLabels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size) {
    // code from ERASOR
    /**< IMPORTANT
         * Because PCL voxlizaiton just does average the intensity of point cloud,
         * so this function is to conduct voxelization followed by nearest points search to re-assign the label of each point */
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);

    // 1. Voxelization
    static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(src);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*ptr_voxelized);

    // 2. Find nearest point to update intensity (index and id)
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(src);

    ptr_reassigned->points.reserve(ptr_voxelized->points.size());

    int K = 1;

    std::vector<int>   pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    // Set dst <- output
    for (const auto &pt: ptr_voxelized->points) {
        if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            auto updated = pt;
            // Update meaned intensity to original intensity
            updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
            ptr_reassigned->points.emplace_back(updated);
        }
    }
    dst = *ptr_reassigned;
}




void VoxelizeWhilePreservingLabelsLargeScale(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);

    // 1. Voxelization

//    static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
//    voxel_filter.setInputCloud(src);
//    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
//    voxel_filter.filter(*ptr_voxelized);

    std::unordered_map<utils::VOXEL_LOC, utils::M_POINT> feature_map;
    size_t pt_size = src->size();

    for (size_t i = 0; i < pt_size; i++) {
        pcl::PointXYZI &pt_trans = src->points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pt_trans.data[j] / leaf_size;
            if (loc_xyz[j] < 0)
                loc_xyz[j] -= 1.0;
        }

        utils::VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feature_map.find(position);
        if (iter != feature_map.end()) {
            iter->second.xyz[0] += pt_trans.x;
            iter->second.xyz[1] += pt_trans.y;
            iter->second.xyz[2] += pt_trans.z;
            iter->second.count++;
        }
        else {
            utils::M_POINT anp;
            anp.xyz[0] = pt_trans.x;
            anp.xyz[1] = pt_trans.y;
            anp.xyz[2] = pt_trans.z;
            anp.count = 1;
            feature_map[position] = anp;
        }
    }

    pt_size = feature_map.size();
    ptr_voxelized->clear();
    ptr_voxelized->resize(pt_size);

    size_t i = 0;
    for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter) {
        ptr_voxelized->points[i].x = iter->second.xyz[0] / iter->second.count;
        ptr_voxelized->points[i].y = iter->second.xyz[1] / iter->second.count;
        ptr_voxelized->points[i].z = iter->second.xyz[2] / iter->second.count;
        i++;
    }
















    // 2. Find nearest point to update intensity (index and id)
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(src);

    ptr_reassigned->points.reserve(ptr_voxelized->points.size());

    int K = 1;

    std::vector<int>   pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    // Set dst <- output
    for (const auto &pt: ptr_voxelized->points) {
        if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            auto updated = pt;
            // Update meaned intensity to original intensity
            updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
            ptr_reassigned->points.emplace_back(updated);
        }
    }
    dst = *ptr_reassigned;
}



void RemoveBackRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, float angle_range)
{
    for(size_t i=0; i<point_cloud->size(); i++) {

        if(!isnan(point_cloud->points[i].x)) {

            float x = point_cloud->points[i].x;
            float y = point_cloud->points[i].y;
            float yaw = atan2(x, y);

            if(yaw<-PI+angle_range || yaw>PI-angle_range) {
                point_cloud->points[i].x = 0;
                point_cloud->points[i].y = 0;
                point_cloud->points[i].z = 0;
            }
        }
    }
}

void RemoveBackRegion(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, float angle_range)
{

    for(size_t i=0; i<point_cloud->size(); i++) {

        if(!isnan(point_cloud->points[i].x)) {

            float x = point_cloud->points[i].x;
            float y = point_cloud->points[i].y;
            float yaw = atan2(x, y);

            if(yaw<-PI+angle_range || yaw>PI-angle_range) {
                point_cloud->points[i].x = 0;
                point_cloud->points[i].y = 0;
                point_cloud->points[i].z = 0;
            }
        }
    }
}

