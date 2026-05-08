
#ifndef CINSTANTTERRAINMAPPER_H_
#define CINSTANTTERRAINMAPPER_H_

#include <opencv2/opencv.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/register_point_struct.h>
#include <pcl/registration/transforms.h>
#include "pcl/visualization/pcl_visualizer.h"

#include <pcl/io/pcd_io.h>


#include "misc/CommenDefinition.h"
#include "misc/FH_kdtree.h"
#include "misc/CoordTransformation.h"
#include "misc/BGK_Smoother.h"
#include "misc/Visualize.h"

#include "misc/Utils.h"

#include "CAccumedTerrainMapper.h"


class CAccumedTerrainMapper;

/////////////////////////////////////////////////////////////////////////////////

class CInstantTerrainCell {
public:
    CInstantTerrainCell();
    ~CInstantTerrainCell();

    int n;
    double sum_height;
    double square_sum_height;

    double mean_height;
    double std_height;

    double max_height;
    double min_height;

    UINT8 label;

    void ComputeStatistics();
    void AddObservation(double observed_height, double observed_height_std = 0);

    ///////////////////////////////////////////////////////////////////////

    void PushData(float observed_height);
    void PopData(float *data, int &num);
    void PopSortedData(float *data, int &num);
    void PopSortedDataWithinROIRange(float *data, int &num, float min_height, float max_height);

    float ref_height_array[5];
    UINT16 binary_data_array[5];

};

class CInstantTerrainMap {
public:
    CInstantTerrainMap();
    ~CInstantTerrainMap();

    void SetMapResolution(double resolution_);
    double resolution;

    void SetMapSize(int map_rows_, int map_cols_);
    int map_rows, map_cols, map_size;

    void Initialize();

    void ComputeStatistics();

    void Reset();

    CInstantTerrainCell default_value;

    CInstantTerrainCell *cells;
};






/////////////////////////////////////////////////////////////////////////////////

class CInstantTerrainMapper
{
public:
    CInstantTerrainMapper();
    ~CInstantTerrainMapper();

    /////////////////////////////////////////////////////////////////////////////////////////////

    void SetLiDARDataPath(std::string LiDARDataPath_);
    std::string LiDARDataPath;

    float lidar_height_to_ground;

    /////////////////////////////////////////////////////////////////////////////////////////////

    void SetParamFile(std::string param_filename_);
    std::string param_filename;

    float roi_range;
    bool b_debug_mode;

    double map_resolution;
    int map_cols, map_rows;

    float slope_angle_thresh;

    /////////////////////////////////////////////////////////////////////////////////////////////
    void SetBGKSmoother(BGK_Smoother *bgk_smoother_);
    BGK_Smoother *bgk_smoother;

    void SetAccumedTerrainMapper(CAccumedTerrainMapper *accumed_terrain_mapper_);
    CAccumedTerrainMapper *accumed_terrain_mapper;
    /////////////////////////////////////////////////////////////////////////////////////////////
    void Initialize();
    void ReadPara();


    void Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Vector3d t_w_l, Eigen::Matrix3d R_w_l, INT64 timestamp);
    void Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, double *xyzypr, INT64 timestamp);

    void ResetIntermediateVariables();
    void ResetIntermediateVariables2();

    void CalculateNormal(cv::Mat &terrain_map, cv::Mat &normal_mat, double map_resolution_);
    void CalculateSlope(cv::Mat &normal_mat, cv::Mat &slope_mat);

    void CalculateROIFromCurrentFrame(cv::Mat &slope_mat, cv::Mat &terrain_height_mat, cv::Mat &chosen_mat,
                                        double map_resolution_, float kLidarHeight, float slope_angle_thresh);
    bool CalculateROIFromAccumedMap(cv::Mat &slope_mat, cv::Mat &terrain_height_mat, cv::Mat &chosen_mat,
                                        double map_resolution_, float kLidarHeight, float slope_angle_thresh);

    void CalculateErrorStatistics(cv::Mat &observed_terrain_height_mat, cv::Mat &predicted_terrain_height_mat);

    CInstantTerrainMap *instant_terrain_map;


    cv::Mat observed_terrain_height_mat;
    cv::Mat observed_std_height_mat;
    cv::Mat predicted_terrain_height_mat;
    cv::Mat normal_mat;
    cv::Mat slope_mat;
    cv::Mat roi_mat;

    double residual_x;      // t_lidar_mapgrid
    double residual_y;

    float k_initial_seed_search_radius;

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::visualization::PCLVisualizer *vis;
    void VisualizeInstantMap(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l);
    void VisualizeInstantMap2(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l);
    void VisualizeAccumedMap(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud);

};






#endif





