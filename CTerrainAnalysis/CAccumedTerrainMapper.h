
#ifndef CACCUMEDTERRAINMAPPER_H_
#define CACCUMEDTERRAINMAPPER_H_

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
//#include "CRemoveDynamic/DynamicMap.h"


#include "CGridMap/CRollingGridMap.h"
//#include "CGridMap/CRollingBlockMap.h"


#include "CInstantTerrainMapper.h"


class CInstantTerrainMapper;

/////////////////////////////////////////////////////////////////////////////////

class CAccumedTerrainCell {
public:
    CAccumedTerrainCell();
    ~CAccumedTerrainCell();

    void AddObservation(float global_point_height);

//    float min_height;
//    float max_height;
    int n;

//    double sum_height;
//    double square_sum_height;

//    double mean_height;
//    double std_height;
    double avg_height;
};

class CAccumedTerrainCellV2 {
public:
    CAccumedTerrainCellV2();
    ~CAccumedTerrainCellV2();

    void AddObservation(float global_point_height);

//    float min_height;
//    float max_height;
    int n;

    double sum_height;
    double square_sum_height;
    int p_value;
    int sum_v;
//    double mean_height;
    double std_height;
    double avg_height;
};




/////////////////////////////////////////////////////////////////////////////////

class CAccumedTerrainMapper
{
public:
    CAccumedTerrainMapper();
    ~CAccumedTerrainMapper();

    /////////////////////////////////////////////////////////////////////////////////////////////
    void SetLiDARDataPath(std::string LiDARDataPath_);
    std::string LiDARDataPath;

    void SetParamFile(std::string param_filename_);
    std::string param_filename;

    bool b_debug_mode;
    double map_resolution;
    int map_cols, map_rows;

    //DynamicMap *check_observation_mapper;
    //void SetDynamicMap(DynamicMap *check_observation_mapper_);

    /////////////////////////////////////////////////////////////////////////////////////////////

    void SetBGKSmoother(BGK_Smoother *bgk_smoother_);
    BGK_Smoother *bgk_smoother;

    void SetInstantTerrainMapper(CInstantTerrainMapper *instant_terrain_mapper_);
    CInstantTerrainMapper *instant_terrain_mapper;

    void SetReferenceHeight(float reference_height_);
    float reference_height;

    /////////////////////////////////////////////////////////////////////////////////////////////

    void Initialize();
    void ReadPara();

    void PreProcess(double *xyzypr);
    void PreProcess(Eigen::Vector3d t_w_l, Eigen::Matrix3d R_w_l);
    // void Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, double *xyzypr, INT64 timestamp);
    void RunV2(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, double *xyzypr, INT64 timestamp);
    void Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, INT64 timestamp);

    void ResetIntermediateVariables();


//    CRollingBlockMap<CAccumedTerrainCell> *big_map;

    //CRollingGridMap<CAccumedTerrainCell> *accumed_terrain_map;
    CRollingGridMap<CAccumedTerrainCellV2> *accumed_terrain_map;
    cv::Mat accumed_observed_terrain_height_mat;
    cv::Mat accumed_predicted_value_mat;
    cv::Mat accumed_predicted_terrain_height_mat;


    double residual_x;      // t_lidar_mapgrid
    double residual_y;

    Eigen::Matrix3d R_w_l;
    Eigen::Vector3d t_w_l;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_point_cloud_n;

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::visualization::PCLVisualizer *vis;
    void AccumedTerrainMapbuild(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l, float t_map_lidar);
    void VisualizeAccumedTerrainMap(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l, float t_map_lidar);
    void VisualizeAccumedTerrainMap2(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l, float t_map_lidar);

};






#endif





