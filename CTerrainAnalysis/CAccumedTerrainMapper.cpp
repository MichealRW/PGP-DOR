


#include "CAccumedTerrainMapper.h"



////////////////////////////////////////////////////////////////////////////////////////////////////

CAccumedTerrainCell::CAccumedTerrainCell()
{
//    min_height = FLT_MAX;
//    max_height = -FLT_MAX;
    n = 0;
//    std_height = 0;

//    sum_height = 0;
//    square_sum_height = 0;
}

CAccumedTerrainCell::~CAccumedTerrainCell()
{
}


void CAccumedTerrainCell::AddObservation(float point_global_height)
{
    n++;
//    if(point_global_height>max_height)
//        max_height = point_global_height;
//    if(point_global_height<min_height)
//        min_height = point_global_height;

//    sum_height += point_global_height;
//    square_sum_height += point_global_height * point_global_height;

    //    double mean_height = sum_height / n;
    //    double variance_height = (square_sum_height + mean_height*mean_height*n - 2*mean_height*sum_height)/double(n-1);
    //    std_height = sqrt(variance_height);

    float alpha = 0.8;  // moving average

    if(n == 1)
        avg_height = point_global_height;
    else
        avg_height = (1-alpha) * avg_height + alpha * point_global_height;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

CAccumedTerrainCellV2::CAccumedTerrainCellV2()
{
//    min_height = FLT_MAX;
//    max_height = -FLT_MAX;
    n = 0;
    std_height = 0;
    sum_v = 0;
    p_value = 0;

    sum_height = 0;
    square_sum_height = 0;
}

CAccumedTerrainCellV2::~CAccumedTerrainCellV2()
{
}


void CAccumedTerrainCellV2::AddObservation(float point_global_height)
{
    n++;
//    if(point_global_height>max_height)
//        max_height = point_global_height;
//    if(point_global_height<min_height)
//        min_height = point_global_height;

    sum_height += point_global_height;
    square_sum_height += point_global_height * point_global_height;

        double mean_height = sum_height / n;
        double variance_height = (square_sum_height + mean_height*mean_height*n - 2*mean_height*sum_height)/double(n-1);
        std_height = sqrt(variance_height);

    float alpha = 0.8;  // moving average

    if(n == 1)
        avg_height = point_global_height;
    else
        avg_height = (1-alpha) * avg_height + alpha * point_global_height;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

CAccumedTerrainMapper::CAccumedTerrainMapper()
{
    vis = NULL;
}

CAccumedTerrainMapper::~CAccumedTerrainMapper()
{
}

void CAccumedTerrainMapper::SetLiDARDataPath(std::string LiDARDataPath_)
{
    LiDARDataPath = LiDARDataPath_;
}

void CAccumedTerrainMapper::SetParamFile(std::string param_filename_)
{
    param_filename = param_filename_;
}

void CAccumedTerrainMapper::SetBGKSmoother(BGK_Smoother *bgk_smoother_)
{
    bgk_smoother = bgk_smoother_;
}

void CAccumedTerrainMapper::SetInstantTerrainMapper(CInstantTerrainMapper *instant_terrain_mapper_)
{
    instant_terrain_mapper = instant_terrain_mapper_;
}

// void CAccumedTerrainMapper::SetDynamicMap(DynamicMap *check_observation_mapper_)
// {
//     check_observation_mapper = check_observation_mapper_;
// }

void CAccumedTerrainMapper::SetReferenceHeight(float reference_height_)
{
    reference_height = reference_height_;
}


void CAccumedTerrainMapper::ReadPara()
{
    std::ifstream fin(param_filename.c_str());

    if (fin.is_open() != 1) {
        std::cout << "Fail to open params file: " << param_filename << std::endl;
        abort();
    }
    std::string t_s;
    while (fin >> t_s) {
        if (t_s[0] == '#' || t_s[0] == '/')
            std::getline(fin, t_s);
        else if (t_s == "b_debug_mode")
            fin >> b_debug_mode;
        else if (t_s == "map_resolution")
            fin >> map_resolution;
        else if (t_s == "map_cols")
            fin >> map_cols;
    }
    fin.close();

    map_rows = map_cols;
}
///////////////////////////////////////////////////////////////////////////////////////////////

void CAccumedTerrainMapper::Initialize()
{
    ReadPara();

    //accumed_terrain_map = new CRollingGridMap<CAccumedTerrainCell>(map_resolution, map_rows, map_cols);
    accumed_terrain_map = new CRollingGridMap<CAccumedTerrainCellV2>(map_resolution, map_rows, map_cols);
    accumed_predicted_value_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) - 1;
    accumed_observed_terrain_height_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    accumed_predicted_terrain_height_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    rebuild_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_point_cloud_n.reset(new pcl::PointCloud<pcl::PointXYZ>);
}


void CAccumedTerrainMapper::ResetIntermediateVariables()
{
    accumed_observed_terrain_height_mat.setTo(INVALID_HEIGHT);
    accumed_predicted_terrain_height_mat.setTo(INVALID_HEIGHT);
}


void CAccumedTerrainMapper::PreProcess(double *xyzypr)
{
    R_w_l = YPR2RotationMatrix(double3D(xyzypr[3], xyzypr[4], xyzypr[5]));
    t_w_l = Eigen::Vector3d(xyzypr[0], xyzypr[1], xyzypr[2]);

    residual_x = std::floor(t_w_l(0) / map_resolution) * map_resolution - t_w_l(0);      // t_lidar_mapgrid
    residual_y = std::floor(t_w_l(1) / map_resolution) * map_resolution - t_w_l(1);

    accumed_terrain_map->SetResidualFromLidarPose(t_w_l(0), t_w_l(1));
    accumed_terrain_map->ReCenter(t_w_l(0), t_w_l(1));
}

void CAccumedTerrainMapper::PreProcess(Eigen::Vector3d t_w_l_, Eigen::Matrix3d R_w_l_)
{
    R_w_l = R_w_l_;
    t_w_l = t_w_l_;

    residual_x = std::floor(t_w_l(0) / map_resolution) * map_resolution - t_w_l(0);      // t_lidar_mapgrid
    residual_y = std::floor(t_w_l(1) / map_resolution) * map_resolution - t_w_l(1);

    accumed_terrain_map->SetResidualFromLidarPose(t_w_l(0), t_w_l(1));
    accumed_terrain_map->ReCenter(t_w_l(0), t_w_l(1));
}



// void CAccumedTerrainMapper::Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, double *xyzypr, INT64 timestamp)
// {

//     //    double t=tic();

//     const cv::Mat observed_terrain_height_mat = instant_terrain_mapper->observed_terrain_height_mat;

//     for(int r=0; r<map_rows; r++) {
//         for(int c=0; c<map_cols; c++) {
//             float observed_height = observed_terrain_height_mat.at<double>(r,c);
//             if(observed_height != INVALID_HEIGHT) {
//                 CAccumedTerrainCell *this_accumed_cell = accumed_terrain_map->GetRCLocal(r, c);
//                 this_accumed_cell->AddObservation(observed_height + t_w_l(2) - reference_height);
//             }
//         }
//     }

//     ResetIntermediateVariables();

//     for(int r=0; r<map_rows; r++) {
//         for(int c=0; c<map_cols; c++) {
//             CAccumedTerrainCell *this_accumed_cell = accumed_terrain_map->GetRCLocal(r, c);
//             if(this_accumed_cell->n > 0)
//                 accumed_observed_terrain_height_mat.at<double>(r,c) = this_accumed_cell->avg_height;
// //                accumed_observed_terrain_height_mat.at<double>(r,c) = this_accumed_cell->sum_height / double(this_accumed_cell->n);
//         }
//     }
//     bgk_smoother->Run(accumed_observed_terrain_height_mat, accumed_predicted_terrain_height_mat, INVALID_HEIGHT, false);

// //    toc(t);

// //    if(b_debug_mode)
//     {
// //        VisualizeMatWithInvalidValue(accumed_observed_terrain_height_mat, INVALID_HEIGHT, "observed_terrain_height", false);
//         VisualizeAccumedTerrainMap(point_cloud, R_w_l, t_w_l(2) - reference_height);
// //        VisualizeAccumedTerrainMap2(point_cloud, R_w_l, t_w_l(2) - reference_height);
//     }
// }

void CAccumedTerrainMapper::Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, INT64 timestamp)
{
    //    double t=tic();

    const cv::Mat observed_terrain_height_mat = instant_terrain_mapper->observed_terrain_height_mat;

    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            float observed_height = observed_terrain_height_mat.at<double>(r,c);
            if(observed_height != INVALID_HEIGHT) {
                CAccumedTerrainCellV2 *this_accumed_cell = accumed_terrain_map->GetRCLocal(r, c);
                this_accumed_cell->AddObservation(observed_height + t_w_l(2) - reference_height);
            }
        }
    }

    ResetIntermediateVariables();

    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            CAccumedTerrainCellV2 *this_accumed_cell = accumed_terrain_map->GetRCLocal(r, c);
            if(this_accumed_cell->n > 0)
                accumed_observed_terrain_height_mat.at<double>(r,c) = this_accumed_cell->avg_height;
//                accumed_observed_terrain_height_mat.at<double>(r,c) = this_accumed_cell->sum_height / double(this_accumed_cell->n);
        }
    }
    bgk_smoother->Run(accumed_observed_terrain_height_mat, accumed_predicted_terrain_height_mat, INVALID_HEIGHT, false);

//    toc(t);

//    if(b_debug_mode)
    {
        // VisualizeMatWithInvalidValue(accumed_observed_terrain_height_mat, INVALID_HEIGHT, "observed_terrain_height", false);
        AccumedTerrainMapbuild(point_cloud, R_w_l, t_w_l(2) - reference_height);
        // VisualizeAccumedTerrainMap(point_cloud, R_w_l, t_w_l(2) - reference_height);
    }
}

void CAccumedTerrainMapper::RunV2(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, double *xyzypr, INT64 timestamp)
{

    //    double t=tic();

    const cv::Mat observed_terrain_height_mat = instant_terrain_mapper->observed_terrain_height_mat;

    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            float observed_height = observed_terrain_height_mat.at<double>(r,c);
            if(observed_height != INVALID_HEIGHT) {
                CAccumedTerrainCellV2 *this_accumed_cell = accumed_terrain_map->GetRCLocal(r, c);
                this_accumed_cell->AddObservation(observed_height + t_w_l(2) - reference_height);
            }
        }
    }

    ResetIntermediateVariables();

    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            CAccumedTerrainCellV2 *this_accumed_cell = accumed_terrain_map->GetRCLocal(r, c);
            if(this_accumed_cell->n > 0)
                accumed_observed_terrain_height_mat.at<double>(r,c) = this_accumed_cell->avg_height;
//                accumed_observed_terrain_height_mat.at<double>(r,c) = this_accumed_cell->sum_height / double(this_accumed_cell->n);
        }
    }
    bgk_smoother->Run(accumed_observed_terrain_height_mat, accumed_predicted_terrain_height_mat, INVALID_HEIGHT, false);

//    toc(t);

//    if(b_debug_mode)
    {
        VisualizeMatWithInvalidValue(accumed_observed_terrain_height_mat, INVALID_HEIGHT, "observed_terrain_height", false);
        AccumedTerrainMapbuild(point_cloud, R_w_l, t_w_l(2) - reference_height);
        //VisualizeAccumedTerrainMap(point_cloud, R_w_l, t_w_l(2) - reference_height);
    }
}

void CAccumedTerrainMapper::AccumedTerrainMapbuild(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l, float t_map_lidar)
{
    rebuild_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_point_cloud_n.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZRGB o;
    pcl::PointXYZ po_r;
    Eigen::Vector3d local_point, rotated_local_point;
    float local_x, local_y;
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for(size_t i=0; i<point_cloud->size(); i++) {
        local_point(0) = point_cloud->points[i].x;
        local_point(1) = point_cloud->points[i].y;
        local_point(2) = point_cloud->points[i].z;

        rotated_local_point = R_w_l * local_point;

        o.x = rotated_local_point(0);
        o.y = rotated_local_point(1);
        o.z = rotated_local_point(2);

        int32_t c = map_cols / 2 + std::floor((rotated_local_point(0) - residual_x) / map_resolution);
        int32_t r = map_rows / 2 - 1 - std::floor((rotated_local_point(1) - residual_y) / map_resolution);
        if(r<0 || r>=map_rows || c<0 || c>=map_cols) {
            o.r = 0; o.g = 0; o.b = 255;
        }
        else {
            double predicted_height = accumed_predicted_terrain_height_mat.at<double>(r,c);
            //diff = t_w_l(2) - reference_height + rotated_local_point(2) - predicted_height;
            float diff = t_map_lidar + rotated_local_point(2) - predicted_height;
            po_r.x = rotated_local_point(0);
            po_r.y = rotated_local_point(1);
            po_r.z = rotated_local_point(2);
            if(accumed_predicted_terrain_height_mat.at<double>(r,c) == INVALID_HEIGHT){
                // rebuild_point_cloud_n->points.push_back(po_r);
                rebuild_point_cloud->points.push_back(po_r);
                continue;
            }
            
            if(diff<0.15) {//0.15
                o.r = 0; o.g = 255; o.b = 0;
                rebuild_point_cloud_n->points.push_back(po_r);
            }
            else {
                o.r = 255; o.g = 0 ; o.b = 0;
                rebuild_point_cloud->points.push_back(po_r);
            }
        }
    }
}

void CAccumedTerrainMapper::VisualizeAccumedTerrainMap(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l, float t_map_lidar)
{
    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("AccumedMap");

    pcl::PointXYZRGB o;
    Eigen::Vector3d local_point, rotated_local_point;
    float local_x, local_y;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr observed_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(size_t i=0; i<point_cloud->size(); i++) {
        local_point(0) = point_cloud->points[i].x;
        local_point(1) = point_cloud->points[i].y;
        local_point(2) = point_cloud->points[i].z;

        rotated_local_point = R_w_l * local_point;

        o.x = rotated_local_point(0);
        o.y = rotated_local_point(1);
        o.z = rotated_local_point(2);

        int32_t c = map_cols / 2 + std::floor((rotated_local_point(0) - residual_x) / map_resolution);
        int32_t r = map_rows / 2 - 1 - std::floor((rotated_local_point(1) - residual_y) / map_resolution);
        if(r<0 || r>=map_rows || c<0 || c>=map_cols) {
            o.r = 0; o.g = 0; o.b = 255;
            observed_point_cloud->points.push_back(o);
        }
        else {
            if(accumed_predicted_terrain_height_mat.at<double>(r,c) == INVALID_HEIGHT)
                continue;
            double predicted_height = accumed_predicted_terrain_height_mat.at<double>(r,c);
            float diff = fabs(predicted_height - t_map_lidar - rotated_local_point(2));
            if(diff<0.15) {
                o.r = 0; o.g = 255; o.b = 0;
            }
            else {
                o.r = 255; o.g = 0 ; o.b = 0;
            }
            observed_point_cloud->points.push_back(o);
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr predicted_terrain_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            accumed_terrain_map->GetXY(r, c, local_x, local_y);
            if(accumed_predicted_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {
                o.x = local_x;
                o.y = local_y;
                o.z = accumed_predicted_terrain_height_mat.at<double>(r,c) - t_map_lidar;
                o.r = 128;
                o.g = 128;
                o.b = 128;
                predicted_terrain_cloud->points.push_back(o);
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    vis->removeAllPointClouds();
    vis->removeAllShapes();

    vis->addPointCloud(predicted_terrain_cloud, "predicted_terrain_cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "predicted_terrain_cloud");
    vis->addPointCloud(observed_point_cloud, "observed_point_cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "observed_point_cloud");

//    vis->spin();
    vis->spinOnce(10);
}

void CAccumedTerrainMapper::VisualizeAccumedTerrainMap2(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l, float t_map_lidar)
{
    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("AccumedMap");


    int v1(0);
    int v2(1);

    vis->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    vis->createViewPort(0.5, 0, 1.0, 1.0, v2);


    pcl::PointXYZRGB o;
    Eigen::Vector3d local_point, rotated_local_point;
    float local_x, local_y;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr observed_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(size_t i=0; i<point_cloud->size(); i++) {
        local_point(0) = point_cloud->points[i].x;
        local_point(1) = point_cloud->points[i].y;
        local_point(2) = point_cloud->points[i].z;

        rotated_local_point = R_w_l * local_point;

        o.x = rotated_local_point(0);
        o.y = rotated_local_point(1);
        o.z = rotated_local_point(2);

        int32_t c = map_cols / 2 + std::floor((rotated_local_point(0) - residual_x) / map_resolution);
        int32_t r = map_rows / 2 - 1 - std::floor((rotated_local_point(1) - residual_y) / map_resolution);
        if(r<0 || r>=map_rows || c<0 || c>=map_cols) {
            o.r = 0; o.g = 0; o.b = 255;
            observed_point_cloud->points.push_back(o);
        }
        else {
            double predicted_height = accumed_predicted_terrain_height_mat.at<double>(r,c);
            float diff = fabs(predicted_height - t_map_lidar - rotated_local_point(2));
            if(diff<0.15) {
                o.r = 0; o.g = 255; o.b = 0;
            }
            else {
                o.r = 255; o.g = 0 ; o.b = 0;
            }
            observed_point_cloud->points.push_back(o);
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr predicted_terrain_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            accumed_terrain_map->GetXY(r, c, local_x, local_y);
            if(accumed_predicted_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {
                o.x = local_x;
                o.y = local_y;
                o.z = accumed_predicted_terrain_height_mat.at<double>(r,c) - t_map_lidar;
                o.r = 128;
                o.g = 128;
                o.b = 128;
                predicted_terrain_cloud->points.push_back(o);
            }
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat result_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    bgk_smoother->Run(instant_terrain_mapper->observed_terrain_height_mat, result_mat, INVALID_HEIGHT, false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr observed_point_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(size_t i=0; i<point_cloud->size(); i++) {
        local_point(0) = point_cloud->points[i].x;
        local_point(1) = point_cloud->points[i].y;
        local_point(2) = point_cloud->points[i].z;

        rotated_local_point = R_w_l * local_point;

        o.x = rotated_local_point(0);
        o.y = rotated_local_point(1);
        o.z = rotated_local_point(2);

        int32_t c = map_cols / 2 + std::floor((rotated_local_point(0) - residual_x) / map_resolution);
        int32_t r = map_rows / 2 - 1 - std::floor((rotated_local_point(1) - residual_y) / map_resolution);
        if(r<0 || r>=map_rows || c<0 || c>=map_cols) {
            o.r = 0; o.g = 0; o.b = 255;
            observed_point_cloud2->points.push_back(o);
        }
        else {
            double predicted_height = result_mat.at<double>(r,c);
            float diff = fabs(predicted_height - rotated_local_point(2));
            if(diff<0.15) {
                o.r = 0; o.g = 255; o.b = 0;
            }
            else {
                o.r = 255; o.g = 0 ; o.b = 0;
            }
            observed_point_cloud2->points.push_back(o);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr predicted_terrain_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            if(result_mat.at<double>(r,c) != INVALID_HEIGHT) {
                o.x = (c - map_cols/2) * map_resolution + map_resolution/2 + residual_x;
                o.y = (map_rows/2 - 1 - r) * map_resolution + map_resolution/2 + residual_y;
                o.z = result_mat.at<double>(r,c);
                o.r = 128;
                o.g = 128;
                o.b = 128;
                predicted_terrain_cloud2->points.push_back(o);
            }
        }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    vis->removeAllPointClouds();
    vis->removeAllShapes();



    vis->addPointCloud(predicted_terrain_cloud, "predicted_terrain_cloud", v1);
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "predicted_terrain_cloud");
    vis->addPointCloud(observed_point_cloud, "observed_point_cloud", v1);
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "observed_point_cloud");

    vis->addPointCloud(predicted_terrain_cloud2, "predicted_terrain_cloud2", v2);
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "predicted_terrain_cloud2");
    vis->addPointCloud(observed_point_cloud2, "observed_point_cloud2", v2);
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "observed_point_cloud2");

//    pcl::PointXYZ p1, p2;
//    float radius = 10;
//    p1.z = o.z;  p2.z = o.z;
//    p1.x = radius; p1.y = radius;
//    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line1");
//    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line2");
//    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line3");
//    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line4");

//    radius = 20;
//    p1.x = radius; p1.y = radius;
//    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line5");
//    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line6");
//    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line7");
//    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line8");

//    radius = 30;
//    p1.x = radius; p1.y = radius;
//    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line9");
//    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line10");
//    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line11");
//    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line12");

    vis->spin();
}
