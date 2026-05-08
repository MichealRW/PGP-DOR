#ifndef CREMOVEDYNAMICV2_H
#define CREMOVEDYNAMICV2_H

#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/visualization/pcl_visualizer.h"
#include "misc/BGK_Smoother.h"
//#include "ikd_Tree.h"    //仅作为导出使用
#include "DynamicMap.h"

//#include "CStreamlineMap/CStreamlineMapV2.h" //仅作为导出使用

#include "CTerrainAnalysis/CInstantTerrainMapper.h"
#include "CTerrainAnalysis/CAccumedTerrainMapper.h"




class RemoveDynamicV2
{
private:

public:
    RemoveDynamicV2();
    ~RemoveDynamicV2();

    void ReadPara();
    void Initialize();
    CInstantTerrainMapper *instant_terrain_mapper;
    CAccumedTerrainMapper *accumed_terrain_mapper;
    //CRangeImageStack *range_image_stack;

    // KD_TREE ikdtree_dy_map;
    // KD_TREE ikdtree_ndy_map;

    float reference_height;
    Eigen::Vector3d T_w_l;
    Eigen::Matrix3d R_w_l;

    //void SetMode(int Mode_in_);
    int Mode_in;
    int num_cloudin;
    void SetLiDARDataPath(std::string LiDARDataPath_);
    std::string LiDARDataPath;
    void SetParamFile(std::string param_filename_);
    std::string param_filename;
    void SetStartFrame(int start_frame_);
    int start_frame;

    BGK_Smoother *bgk_smoother;
    BGK_Smoother *high_res_bgk_smoother;
    double map_resolution;
    float roi_range;
    float RONI_min_x, RONI_max_x, RONI_min_y, RONI_max_y, RONI_min_z, RONI_max_z;
    int map_cols, map_rows;
    double k_prediction_kernel_radius;
    double k_prediction_kernel_radius_s;
    int k_valid_points_num_thresh;
    int k_valid_points_num_thresh_s;
    double k_bf_height_std;
    bool b_enable_visualization;

    int chech_mode;

    DynamicMap *dynamicmap_rm;
    void PreprocessPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_point_cloud;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_dy;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud;

    void Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Vector3d t_w_l, Eigen::Matrix3d r_w_l, INT64 timestamp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_dy_pts;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_nody_pts;

    void LoadAll();

    ////////////// test ////////////////////
    void SetKD(Eigen::Vector3d t_w_l);
    void OutputKD();
    void EndOut();
    ///////////// test //////////////////////

    void Visualizeob(bool b_pause);
    void Visualizete(bool b_pause);
    void VisualizeDynamicMap(bool b_pause);
    pcl::visualization::PCLVisualizer *vis;
};

#endif
