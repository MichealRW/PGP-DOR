//#pragma once
#ifndef __UTILS_H__
#define __UTILS_H__


#include <unordered_map>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/PCLPointCloud2.h>

#include <opencv2/opencv.hpp>

#include "CommenDefinition.h"

extern const UINT16 ones_LUT[16];
extern const UINT16 not_ones_LUT[16];

extern const UINT32 ones_32_LUT[32];
extern const UINT32 not_ones_32_LUT[32];

typedef struct {
   int rows;//行数
   int cols;//列数
   int type;//类型
} MatHeader;

void WriteMat(std::string fileName, cv::Mat &src);
cv::Mat ReadMat(std::string fileName);



namespace utils {

#define SMALL_EPS 1e-10
#define HASH_P 116101
#define MAX_N 10000000019


class VOXEL_LOC
{
public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0): x(vx), y(vy), z(vz){}

    bool operator== (const VOXEL_LOC &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }
};



struct M_POINT
{
    float xyz[3];
    int count = 0;
};
}

namespace std
{
template<>
struct hash<utils::VOXEL_LOC>
{
    size_t operator() (const utils::VOXEL_LOC &s) const
    {
        using std::size_t;
        using std::hash;
        long index_x, index_y, index_z;
        double cub_len = 1.0/8;
        index_x = int(round(floor((s.x)/cub_len + SMALL_EPS)));
        index_y = int(round(floor((s.y)/cub_len + SMALL_EPS)));
        index_z = int(round(floor((s.z)/cub_len + SMALL_EPS)));
        return (((((index_z * HASH_P) % MAX_N + index_y) * HASH_P) % MAX_N) + index_x) % MAX_N;
    }
};
}


void VoxelizeWhilePreservingLabels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size);
void VoxelizeWhilePreservingLabelsLargeScale(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size);


void RemoveBackRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, float angle_range = PI/12);
void RemoveBackRegion(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, float angle_range = PI/12);


#endif
