#include <filesystem>
#include "CInstantTerrainMapper.h"




////////////////////////////////////////////////////////////////////////////////////////////////////


CInstantTerrainCell::CInstantTerrainCell()
{
    n = 0;
    sum_height = 0;
    square_sum_height = 0;

    mean_height = 0;
    std_height = 0;

    max_height = -DBL_MAX;
    min_height = DBL_MAX;

    label = 0;

    ///////////////////////////////////////////////////////////

    for(size_t i=0; i<5; i++) {
        ref_height_array[i] = INVALID_HEIGHT;
        binary_data_array[i] = 0;
    }

}



CInstantTerrainCell::~CInstantTerrainCell()
{
}

void CInstantTerrainCell::AddObservation(double observed_height, double observed_height_std)
{
    n++;
    sum_height += observed_height;
    square_sum_height += observed_height*observed_height;

    if(observed_height > max_height)
        max_height = observed_height;
    if(observed_height < min_height)
        min_height = observed_height;

    PushData(float(observed_height));
}

void CInstantTerrainCell::PushData(float observed_height)
{
    ///////////////////////////////////////////////////////////

    float sector_resolution = 0.1 * 16.0;

    float ref_height = std::floor(observed_height / sector_resolution) * sector_resolution;

    for(size_t i=0; i<5; i++) {
        if(ref_height_array[i] == INVALID_HEIGHT) {
            ref_height_array[i] = ref_height;
            int idx = std::floor((observed_height - ref_height) / 0.1);
            binary_data_array[i] = binary_data_array[i] | ones_LUT[idx];
            break;
        }
        else {
            int idx = std::floor((observed_height - ref_height_array[i]) / 0.1);
            if(idx>=0 && idx<16) {
                binary_data_array[i] = binary_data_array[i] | ones_LUT[idx];
                break;
            }
        }
    }
}

void CInstantTerrainCell::PopData(float *data, int &num)
{
    num = 0;
    for(size_t i=0; i<5; i++) {
        if(ref_height_array[i] != INVALID_HEIGHT) {
            for(int j=0; j<16; j++) {
                if(binary_data_array[i] & ones_LUT[j])
                    data[num++] = ref_height_array[i] + j * 0.1 + 0.05;
            }
        }
    }
}


void CInstantTerrainCell::PopSortedData(float *data, int &num)
{
    UINT8 idx[5];
    UINT8 counter = 0;
    for(size_t i=0; i<5; i++) {
        if(ref_height_array[i] != INVALID_HEIGHT) {
            idx[counter++] = i;
        }
    }
    UINT8 tmp;
    for(size_t i=0; i<counter; i++) {
        for(size_t j=i+1; j<counter; j++) {
            if(ref_height_array[idx[i]] > ref_height_array[idx[j]]) {
                tmp = idx[i];
                idx[i] = idx[j];
                idx[j] = tmp;
            }
        }
    }

    num = 0;
    for(size_t i=0; i<counter; i++) {
        for(int j=0; j<16; j++) {
            if(binary_data_array[idx[i]] & ones_LUT[j])
                data[num++] = ref_height_array[idx[i]] + j * 0.1 + 0.05;
        }
    }
}


void CInstantTerrainCell::PopSortedDataWithinROIRange(float *data, int &num, float min_height, float max_height)
{
    UINT8 idx[5];
    UINT8 counter = 0;
    for(size_t i=0; i<5; i++) {
        if(ref_height_array[i] != INVALID_HEIGHT) {
            idx[counter++] = i;
        }
    }
    UINT8 tmp;
    for(size_t i=0; i<counter; i++) {
        for(size_t j=i+1; j<counter; j++) {
            if(ref_height_array[idx[i]] > ref_height_array[idx[j]]) {
                tmp = idx[i];
                idx[i] = idx[j];
                idx[j] = tmp;
            }
        }
    }

    num = 0;
    for(size_t i=0; i<counter; i++) {
        for(int j=0; j<16; j++) {
            if(binary_data_array[idx[i]] & ones_LUT[j]) {
                float value = ref_height_array[idx[i]] + j * 0.1 + 0.05;
                if(value > min_height && value < max_height)
                    data[num++] = value;
            }
        }
    }
}

void CInstantTerrainCell::ComputeStatistics()
{
    if(n>0)
        mean_height = sum_height / n;
    if(n>1) {
        double variance_height = (square_sum_height + mean_height*mean_height*n - 2*mean_height*sum_height)/double(n-1);
        std_height = sqrt(variance_height);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

CInstantTerrainMap::CInstantTerrainMap()
{
}

CInstantTerrainMap::~CInstantTerrainMap()
{
    delete [] cells;
}

void CInstantTerrainMap::SetMapResolution(double resolution_)
{
    resolution = resolution_;
}

void CInstantTerrainMap::SetMapSize(int map_rows_, int map_cols_)
{
    map_rows = map_rows_;
    map_cols = map_cols_;
    map_size = map_rows * map_cols;
}

void CInstantTerrainMap::Initialize()
{
    cells = new CInstantTerrainCell [map_size];
}

void CInstantTerrainMap::ComputeStatistics()
{
    for(int i=0; i<map_size; i++)
        cells[i].ComputeStatistics();
}

void CInstantTerrainMap::Reset()
{
    for (int i = 0; i < map_size; i++)
        memcpy(&cells[i], &default_value, sizeof(CInstantTerrainCell));
}



////////////////////////////////////////////////////////////////////////////////////////////////////

CInstantTerrainMapper::CInstantTerrainMapper()
{
    accumed_terrain_mapper = NULL;
    vis = NULL;
}

CInstantTerrainMapper::~CInstantTerrainMapper()
{
}


void CInstantTerrainMapper::SetLiDARDataPath(std::string LiDARDataPath_)
{
    LiDARDataPath = LiDARDataPath_;
}

void CInstantTerrainMapper::SetParamFile(std::string param_filename_)
{
    param_filename = param_filename_;
}

void CInstantTerrainMapper::SetBGKSmoother(BGK_Smoother *bgk_smoother_)
{
    bgk_smoother = bgk_smoother_;
}

void CInstantTerrainMapper::SetAccumedTerrainMapper(CAccumedTerrainMapper *accumed_terrain_mapper_)
{
    accumed_terrain_mapper = accumed_terrain_mapper_;
}


void CInstantTerrainMapper::ReadPara()
{
    const std::filesystem::path param_path(param_filename);
    const std::filesystem::path lidar_spec_path_in_param_dir =
        param_path.parent_path() / "pgp_dor_lidar_specification.ini";
    const std::filesystem::path lidar_spec_path_in_dataset =
        std::filesystem::path(LiDARDataPath) / "params" / "lidar_specification.ini";

    std::ifstream fin(lidar_spec_path_in_param_dir);
    if(fin.is_open()!=1) {
        fin.open(lidar_spec_path_in_dataset);
    }
    if(fin.is_open()!=1) {
        std::cout << "Fail to open params file: "
                  << lidar_spec_path_in_param_dir.string()
                  << " or " << lidar_spec_path_in_dataset.string() << std::endl;
        abort();
    }
    std::string t_s;
    while (fin >> t_s) {
        if (t_s[0] == '#' || t_s[0] == '/')
            std::getline(fin, t_s);
        else if (t_s == "lidar_height_to_ground")
            fin >> lidar_height_to_ground;
    }
    fin.close();


    ///////////////////////////////////////////////////////////////////////////////

    fin.open(param_filename.c_str());

    if (fin.is_open() != 1) {
        std::cout << "Fail to open params file: " << param_filename << std::endl;
        abort();
    }

    while (fin >> t_s) {
        if (t_s[0] == '#' || t_s[0] == '/')
            std::getline(fin, t_s);
        else if (t_s == "roi_range")
            fin >> roi_range;
        else if (t_s == "b_debug_mode")
            fin >> b_debug_mode;
        else if (t_s == "map_resolution")
            fin >> map_resolution;
        else if (t_s == "map_cols")
            fin >> map_cols;
        else if (t_s == "slope_angle_thresh")
            fin >> slope_angle_thresh;
        else if (t_s == "k_initial_seed_search_radius")
            fin >> k_initial_seed_search_radius;
    }
    fin.close();

    map_rows = map_cols;
}
///////////////////////////////////////////////////////////////////////////////////////////////

void CInstantTerrainMapper::Initialize()
{
    ReadPara();


    instant_terrain_map = new CInstantTerrainMap;
    instant_terrain_map->SetMapResolution(map_resolution);
    instant_terrain_map->SetMapSize(map_rows, map_cols);
    instant_terrain_map->Initialize();



    observed_terrain_height_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    observed_std_height_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    predicted_terrain_height_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    normal_mat = cv::Mat::zeros(map_rows, map_cols, CV_32FC3);
    slope_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_VALUE;
    roi_mat = cv::Mat::zeros(map_rows, map_cols, CV_8UC1);
}

void CInstantTerrainMapper::ResetIntermediateVariables()
{
    //    observed_terrain_height_mat = cv::Mat::zeros(map_rows, map_cols, CV_64FC1) + INVALID_HEIGHT;
    observed_terrain_height_mat.setTo(INVALID_HEIGHT);
    predicted_terrain_height_mat.setTo(INVALID_HEIGHT);
    observed_std_height_mat.setTo(INVALID_HEIGHT);
    //    normal_mat.setTo(cv::Vec3f(0.0,0.0,0.0));
    normal_mat.setTo(0);

    slope_mat.setTo(INVALID_VALUE);
    roi_mat.setTo(0);
}

void CInstantTerrainMapper::Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Vector3d t_w_l, Eigen::Matrix3d R_w_l, INT64 timestamp)
{
    //    double t=tic();

    // Eigen::Matrix3d R_w_l = YPR2RotationMatrix(double3D(xyzypr[3], xyzypr[4], xyzypr[5]));
    // Eigen::Vector3d t_w_l = Eigen::Vector3d(xyzypr[0], xyzypr[1], xyzypr[2]);

    residual_x = std::floor(t_w_l(0) / map_resolution) * map_resolution - t_w_l(0);      // t_lidar_mapgrid
    residual_y = std::floor(t_w_l(1) / map_resolution) * map_resolution - t_w_l(1);

    //////////////////////////////////////////////////////////////////////////////////////

    Eigen::Vector3d local_point, rotated_local_point;
    double local_height;

    instant_terrain_map->Reset();
    for(size_t i=0; i<point_cloud->size(); i++) {

        local_point(0) = point_cloud->points[i].x;
        local_point(1) = point_cloud->points[i].y;
        local_point(2) = point_cloud->points[i].z;

        rotated_local_point = R_w_l * local_point;

        local_height = rotated_local_point(2);

        int32_t c = map_cols / 2 + std::floor((rotated_local_point(0) - residual_x) / map_resolution);
        int32_t r = map_rows / 2 - 1 - std::floor((rotated_local_point(1) - residual_y) / map_resolution);
        if(r<0 || r>=map_rows || c<0 || c>=map_cols)
            continue;

        int idx = r*map_cols + c;
        CInstantTerrainCell *this_cell = instant_terrain_map->cells + idx;
        this_cell->AddObservation(local_height);
    }
    instant_terrain_map->ComputeStatistics();

    ////////////////////////////////////////////////////////////////////////////////////

    ResetIntermediateVariables();


    //    ofstream fout("debug.txt");
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            int idx = r * map_cols + c;
            CInstantTerrainCell *this_cell = instant_terrain_map->cells + idx;
            if(this_cell->n > 0) {
                if(this_cell->std_height<0.1) {

                    float local_x = (c - map_cols/2) * map_resolution + map_resolution/2 + residual_x;
                    float local_y = (map_rows/2 - 1 - r) * map_resolution + map_resolution/2 + residual_y;
                    float dist_xy = sqrt(local_x*local_x + local_y*local_y);

                    float pitch = atan2(this_cell->min_height, dist_xy);
                    //                if(pitch < PI/6.0)
                    if(pitch < 0)
                        observed_terrain_height_mat.at<double>(r,c) = this_cell->min_height;
                    //                fout<<local_x<<" "<<local_y<<" "<<this_cell->min_height<<" "<<pitch<<std::endl;
                }else if(this_cell->std_height < 1){
                    if(this_cell->max_height - this_cell->min_height > 2.0)
                        continue;

                    observed_std_height_mat.at<double>(r,c) = this_cell->std_height;
                }
            }
        }
    }
    //查看std——mat
    //VisualizeMat(observed_std_height_mat, "std_mat", false);
    //    fout.close();

    //        if(b_debug_mode)
    //            VisualizeMatWithInvalidValue(observed_terrain_height_mat, INVALID_HEIGHT, "observed_terrain_height", true);

    //        if(b_debug_mode) {
    //            bgk_smoother->Run(observed_terrain_height_mat, predicted_terrain_height_mat, INVALID_HEIGHT, false);
    //            VisualizeInstantMap();
    //        }

    bgk_smoother->Run(observed_terrain_height_mat, predicted_terrain_height_mat, INVALID_HEIGHT, true);
    if(b_debug_mode) {
        std::cout<<"After 1st BGK: "<<std::endl;
        CalculateErrorStatistics(observed_terrain_height_mat, predicted_terrain_height_mat);
    }

    if(b_debug_mode)
        VisualizeInstantMap(point_cloud, R_w_l);

    CalculateNormal(predicted_terrain_height_mat, normal_mat, map_resolution);
    //    CalculateNormalV2(predicted_terrain_height_mat, normal_mat, map_resolution, normal_radius, normal_valid_points_num_thresh);

    CalculateSlope(normal_mat, slope_mat);



    /// throw away those unstable points to avoid their influence on the ROI search
    const float k_pred_obs_diff_thresh = 0.15;   // difference between prediction and observation
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            if(observed_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {

                if(predicted_terrain_height_mat.at<double>(r,c) == INVALID_HEIGHT)
                    observed_terrain_height_mat.at<double>(r,c) = INVALID_HEIGHT;
                else {
                    if(fabs(predicted_terrain_height_mat.at<double>(r,c) - observed_terrain_height_mat.at<double>(r,c))>k_pred_obs_diff_thresh)
                        observed_terrain_height_mat.at<double>(r,c) = INVALID_HEIGHT;
                }
            }
        }
    }



    static bool b_first_run = true;
    if(b_first_run || accumed_terrain_mapper == NULL) {
        CalculateROIFromCurrentFrame(slope_mat, observed_terrain_height_mat, roi_mat, map_resolution, lidar_height_to_ground, slope_angle_thresh);
        b_first_run = false;
    }
    else {
//        VisualizeAccumedMap(point_cloud);
        if(!CalculateROIFromAccumedMap(slope_mat, observed_terrain_height_mat, roi_mat, map_resolution, lidar_height_to_ground, slope_angle_thresh))
            CalculateROIFromCurrentFrame(slope_mat, observed_terrain_height_mat, roi_mat, map_resolution, lidar_height_to_ground, slope_angle_thresh);
    }



    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            if(observed_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {

                if(predicted_terrain_height_mat.at<double>(r,c) == INVALID_HEIGHT)
                    observed_terrain_height_mat.at<double>(r,c) = INVALID_HEIGHT;
                else {
                    if(roi_mat.at<UINT8>(r,c) == 0 || fabs(predicted_terrain_height_mat.at<double>(r,c) - observed_terrain_height_mat.at<double>(r,c))>k_pred_obs_diff_thresh)
                        observed_terrain_height_mat.at<double>(r,c) = INVALID_HEIGHT;
                }
            }
        }
    }



    if(b_debug_mode)
    {
        predicted_terrain_height_mat.setTo(INVALID_HEIGHT);
        bgk_smoother->Run(observed_terrain_height_mat, predicted_terrain_height_mat, INVALID_HEIGHT, false);
        std::cout<<"After 2nd BGK: "<<std::endl;
        CalculateErrorStatistics(observed_terrain_height_mat, predicted_terrain_height_mat);
        VisualizeInstantMap(point_cloud, R_w_l);
    }
    
    //VisualizeInstantMap(point_cloud, R_w_l);


    //    toc(t);


    if(b_debug_mode)
        VisualizeInstantMap2(point_cloud, R_w_l);
}


void CInstantTerrainMapper::Run(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, double *xyzypr, INT64 timestamp)
{
    //    double t=tic();

    Eigen::Matrix3d R_w_l = YPR2RotationMatrix(double3D(xyzypr[3], xyzypr[4], xyzypr[5]));
    Eigen::Vector3d t_w_l = Eigen::Vector3d(xyzypr[0], xyzypr[1], xyzypr[2]);

    residual_x = std::floor(t_w_l(0) / map_resolution) * map_resolution - t_w_l(0);      // t_lidar_mapgrid
    residual_y = std::floor(t_w_l(1) / map_resolution) * map_resolution - t_w_l(1);

    //////////////////////////////////////////////////////////////////////////////////////

    Eigen::Vector3d local_point, rotated_local_point;
    double local_height;

    instant_terrain_map->Reset();
    for(size_t i=0; i<point_cloud->size(); i++) {

        local_point(0) = point_cloud->points[i].x;
        local_point(1) = point_cloud->points[i].y;
        local_point(2) = point_cloud->points[i].z;

        rotated_local_point = R_w_l * local_point;

        local_height = rotated_local_point(2);

        int32_t c = map_cols / 2 + std::floor((rotated_local_point(0) - residual_x) / map_resolution);
        int32_t r = map_rows / 2 - 1 - std::floor((rotated_local_point(1) - residual_y) / map_resolution);
        if(r<0 || r>=map_rows || c<0 || c>=map_cols)
            continue;

        int idx = r*map_cols + c;
        CInstantTerrainCell *this_cell = instant_terrain_map->cells + idx;
        this_cell->AddObservation(local_height);
    }
    instant_terrain_map->ComputeStatistics();

    ////////////////////////////////////////////////////////////////////////////////////

    ResetIntermediateVariables();


    //    ofstream fout("debug.txt");
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            int idx = r * map_cols + c;
            CInstantTerrainCell *this_cell = instant_terrain_map->cells + idx;
            if(this_cell->n > 0) {
                if(this_cell->std_height<0.1) {

                    float local_x = (c - map_cols/2) * map_resolution + map_resolution/2 + residual_x;
                    float local_y = (map_rows/2 - 1 - r) * map_resolution + map_resolution/2 + residual_y;
                    float dist_xy = sqrt(local_x*local_x + local_y*local_y);

                    float pitch = atan2(this_cell->min_height, dist_xy);
                    //                if(pitch < PI/6.0)
                    if(pitch < 0)
                        observed_terrain_height_mat.at<double>(r,c) = this_cell->min_height;
                    //                fout<<local_x<<" "<<local_y<<" "<<this_cell->min_height<<" "<<pitch<<std::endl;
                }else if(this_cell->std_height < 1){
                    if(this_cell->max_height - this_cell->min_height > 2.0)
                        continue;

                    observed_std_height_mat.at<double>(r,c) = this_cell->std_height;
                }
            }
        }
    }
    //查看std——mat
    //VisualizeMat(observed_std_height_mat, "std_mat", false);
    //    fout.close();

    //        if(b_debug_mode)
    //            VisualizeMatWithInvalidValue(observed_terrain_height_mat, INVALID_HEIGHT, "observed_terrain_height", true);

    //        if(b_debug_mode) {
    //            bgk_smoother->Run(observed_terrain_height_mat, predicted_terrain_height_mat, INVALID_HEIGHT, false);
    //            VisualizeInstantMap();
    //        }

    bgk_smoother->Run(observed_terrain_height_mat, predicted_terrain_height_mat, INVALID_HEIGHT, true);
    if(b_debug_mode) {
        std::cout<<"After 1st BGK: "<<std::endl;
        CalculateErrorStatistics(observed_terrain_height_mat, predicted_terrain_height_mat);
    }

    if(b_debug_mode)
        VisualizeInstantMap(point_cloud, R_w_l);

    CalculateNormal(predicted_terrain_height_mat, normal_mat, map_resolution);
    //    CalculateNormalV2(predicted_terrain_height_mat, normal_mat, map_resolution, normal_radius, normal_valid_points_num_thresh);

    CalculateSlope(normal_mat, slope_mat);



    /// throw away those unstable points to avoid their influence on the ROI search
    const float k_pred_obs_diff_thresh = 0.15;   // difference between prediction and observation
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            if(observed_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {

                if(predicted_terrain_height_mat.at<double>(r,c) == INVALID_HEIGHT)
                    observed_terrain_height_mat.at<double>(r,c) = INVALID_HEIGHT;
                else {
                    if(fabs(predicted_terrain_height_mat.at<double>(r,c) - observed_terrain_height_mat.at<double>(r,c))>k_pred_obs_diff_thresh)
                        observed_terrain_height_mat.at<double>(r,c) = INVALID_HEIGHT;
                }
            }
        }
    }



    static bool b_first_run = true;
    if(b_first_run || accumed_terrain_mapper == NULL) {
        CalculateROIFromCurrentFrame(slope_mat, observed_terrain_height_mat, roi_mat, map_resolution, lidar_height_to_ground, slope_angle_thresh);
        b_first_run = false;
    }
    else {
//        VisualizeAccumedMap(point_cloud);
        if(!CalculateROIFromAccumedMap(slope_mat, observed_terrain_height_mat, roi_mat, map_resolution, lidar_height_to_ground, slope_angle_thresh))
            CalculateROIFromCurrentFrame(slope_mat, observed_terrain_height_mat, roi_mat, map_resolution, lidar_height_to_ground, slope_angle_thresh);
    }



    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            if(observed_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {

                if(predicted_terrain_height_mat.at<double>(r,c) == INVALID_HEIGHT)
                    observed_terrain_height_mat.at<double>(r,c) = INVALID_HEIGHT;
                else {
                    if(roi_mat.at<UINT8>(r,c) == 0 || fabs(predicted_terrain_height_mat.at<double>(r,c) - observed_terrain_height_mat.at<double>(r,c))>k_pred_obs_diff_thresh)
                        observed_terrain_height_mat.at<double>(r,c) = INVALID_HEIGHT;
                }
            }
        }
    }



    if(b_debug_mode)
    {
        predicted_terrain_height_mat.setTo(INVALID_HEIGHT);
        bgk_smoother->Run(observed_terrain_height_mat, predicted_terrain_height_mat, INVALID_HEIGHT, false);
        std::cout<<"After 2nd BGK: "<<std::endl;
        CalculateErrorStatistics(observed_terrain_height_mat, predicted_terrain_height_mat);
        VisualizeInstantMap(point_cloud, R_w_l);
    }
    
    //VisualizeInstantMap(point_cloud, R_w_l);


    //    toc(t);


    if(b_debug_mode)
        VisualizeInstantMap2(point_cloud, R_w_l);
}

void CInstantTerrainMapper::CalculateErrorStatistics(cv::Mat &observed_terrain_height_mat, cv::Mat &predicted_terrain_height_mat)
{

    double sum_diff = 0;
    double sum_squared_diff = 0;
    double max_diff = -DBL_MAX;
    int num = 0;
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            if(observed_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT && predicted_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {
                double diff = fabs(observed_terrain_height_mat.at<double>(r,c) - predicted_terrain_height_mat.at<double>(r,c));
                if(diff > max_diff)
                    max_diff = diff;
                sum_diff += diff;
                sum_squared_diff += diff * diff;
                num++;
            }
        }
    }

    double mean_diff = sum_diff / num;

    double variance_height = (sum_squared_diff + mean_diff*mean_diff*num - 2*mean_diff*sum_diff)/double(num-1);
    double std_diff = sqrt(variance_height);

    std::cout<<"Valid points num: "<<num<<std::endl;
    std::cout<<"Max diff: "<<max_diff<<std::endl;
    std::cout<<"Avg diff: "<<mean_diff<<std::endl;
    std::cout<<"Std diff: "<<std_diff<<std::endl<<std::endl;

}


void CInstantTerrainMapper::CalculateSlope(cv::Mat &normal_mat, cv::Mat &slope_mat)
{
    double greeness, redness;
    cv::Mat colored_mat = cv::Mat::zeros(normal_mat.rows, normal_mat.cols, CV_8UC3);
    for(int r = 0; r<colored_mat.rows; r++) {
        for(int c=0; c<colored_mat.cols; c++) {
            cv::Vec3f pt_normal = normal_mat.at<cv::Vec3f>(r,c);
            if(pt_normal[0] != 0 || pt_normal[1] != 0 || pt_normal[2] != 0) {
                double angle = acos(pt_normal[2]); // theta = acos(a .* b / a.norm * b.norm). The angle between normal and z-axis (0,0,1).
                angle = fabs(angle - PI/2.0);  // [0, pi/2] -> [pi/2, 0]; [pi/2, pi] -> [0, pi/2]

                if(0) {
                    greeness = angle / (PI / 2.0);
                    redness = 1 - greeness;
                }
                else {
                    double visualize_angle_range = PI/12.0;
                    angle = PI/2 - angle;

                    slope_mat.at<double>(r,c) = angle * 180.0 / PI;

                    redness = angle / visualize_angle_range;
                    if(redness>1.0)
                        redness = 1.0;
                    greeness = 1.0 - redness;
                }

                colored_mat.at<cv::Vec3b>(r, c) = cv::Vec3b(0, greeness*255.0, redness*255.0);
            }
        }
    }

    if(b_debug_mode) {
        VisualizeMat(colored_mat, "normal_mat", true);
        VisualizeMatWithInvalidValue(slope_mat, INVALID_VALUE, "slope_mat", true);
    }
}

void CInstantTerrainMapper::CalculateNormal(cv::Mat &terrain_map, cv::Mat &normal_mat, double map_resolution_)
{
    // Connectivity computation based on Normal Vector
    for(int r=1; r<terrain_map.rows-1; r++) {
        for(int c=1; c<terrain_map.cols-1; c++) {
            if(terrain_map.at<double>(r,c) == INVALID_VALUE)
                c++;
            else {
                double right_height = terrain_map.at<double>(r,c+1);
                if(right_height == INVALID_VALUE)
                    c+=2;
                else {
                    double bottom_height = terrain_map.at<double>(r+1,c);
                    double left_height = terrain_map.at<double>(r,c-1);
                    double up_height = terrain_map.at<double>(r-1,c);

                    if(bottom_height != INVALID_VALUE && left_height != INVALID_VALUE && up_height != INVALID_VALUE) {
                        double3D right_vector(map_resolution_, 0, right_height);
                        double3D bottom_vector(0, -map_resolution_, bottom_height);
                        double3D left_vector(-map_resolution_, 0, left_height);
                        double3D up_vector(0, map_resolution_, up_height);

                        double3D normal = (right_vector - left_vector).crossProduct(up_vector - bottom_vector);
                        normal.normalise();
                        //                        double angle = acos(normal.z); // theta = acos(a .* b / a.norm * b.norm). The angle between normal and z-axis (0,0,1).
                        //                        angle = fabs(angle - PI/2);  // [0, pi/2] -> [pi/2, 0]; [pi/2, pi] -> [0, pi/2]
                        //                        normal_angle[idx] = (PI/2 - angle) * 180.0 / PI;

                        normal_mat.at<cv::Vec3f>(r,c) = cv::Vec3f(normal.x, normal.y, normal.z);
                    }
                }
            }
        }
    }
}

void CInstantTerrainMapper::CalculateROIFromCurrentFrame(cv::Mat &slope_mat, cv::Mat &observed_terrain_height_mat, cv::Mat &chosen_mat,
                                         double map_resolution_, float kLidarHeight, float slope_angle_thresh)
{

    if(b_debug_mode)
        std::cout<<"CalculateROIFromCurrentFrame!"<<std::endl;

    // k_initial_seed_search_radius = 6.0;
    float delta_height_thresh = 0.3;

    float t_lidar_ground = -kLidarHeight; // ground height in lidar coordinate

    int map_rows = slope_mat.rows;
    int map_cols = slope_mat.cols;

    std::vector<short2D> seed_array;
    seed_array.resize(slope_mat.rows * slope_mat.cols);

    int start_idx = 0;
    int end_idx = 0;
    for(short r=0; r<map_rows; r++) {
        for(short c=0; c<map_cols; c++) {
            float dist = sqrt((r - map_rows/2)*(r - map_rows/2) + (c - 1 - map_cols/2)*(c - 1 - map_cols/2)) * map_resolution_;
            if(dist<k_initial_seed_search_radius) {
                float local_height = observed_terrain_height_mat.at<double>(r, c);
                if(local_height != INVALID_HEIGHT && slope_mat.at<double>(r,c)!=INVALID_VALUE) {
                    if(fabs(local_height - t_lidar_ground) < delta_height_thresh &&
                            slope_mat.at<double>(r,c) < slope_angle_thresh) {
                        seed_array[end_idx++] = short2D(r,c);
                        chosen_mat.at<UINT8>(r,c) = 1;
                    }
                }
            }
        }
    }

    if(end_idx == 0) {
        std::cout<<"Error occured in function CalculateROI. No seed points are found!"<<std::endl;
        std::abort();
    }

    if(b_debug_mode) {
        cv::namedWindow("chosen_mat",cv::WINDOW_NORMAL);
        cv::imshow("chosen_mat",chosen_mat*255);
        cv::waitKey(-1);
    }

    INT8 idx_array[8] = {1,0,-1,0,0,1,0,-1};
    while(start_idx < end_idx) {
        short this_r = seed_array[start_idx].x;
        short this_c = seed_array[start_idx].y;

        for(int i=0; i<4; i++) {
            int r = this_r + idx_array[2*i];
            int c = this_c + idx_array[2*i + 1];
            if(r>=0 && r<slope_mat.rows && c>=0 && c<slope_mat.cols) {
                if(!chosen_mat.at<UINT8>(r,c)) {
                    if(slope_mat.at<double>(r,c) < slope_angle_thresh && slope_mat.at<double>(r,c)!=INVALID_VALUE) {
                        seed_array[end_idx++] = short2D(r,c);
                        chosen_mat.at<UINT8>(r,c) = 1;
                    }
                }
            }
        }
        start_idx++;
    }

    if(b_debug_mode) {
        cv::namedWindow("chosen_mat",cv::WINDOW_NORMAL);
        cv::imshow("chosen_mat",chosen_mat*255);
        cv::waitKey(-1);
    }
}


bool CInstantTerrainMapper::CalculateROIFromAccumedMap(cv::Mat &slope_mat, cv::Mat &observed_terrain_height_mat, cv::Mat &chosen_mat,
                                         double map_resolution_, float kLidarHeight, float slope_angle_thresh)
{

    if(b_debug_mode)
        std::cout<<"CalculateROIFromAccumedMap!"<<std::endl;

    float k_initial_seed_search_radius = 8.0;
    float delta_height_thresh = 0.15;

    int map_rows = slope_mat.rows;
    int map_cols = slope_mat.cols;

    std::vector<short2D> seed_array;
    seed_array.resize(slope_mat.rows * slope_mat.cols);

    int start_idx = 0;
    int end_idx = 0;
    for(short r=0; r<map_rows; r++) {
        for(short c=0; c<map_cols; c++) {
            float dist = sqrt((r - map_rows/2)*(r - map_rows/2) + (c - 1 - map_cols/2)*(c - 1 - map_cols/2)) * map_resolution_;
            if(dist<k_initial_seed_search_radius) {
                float local_height = observed_terrain_height_mat.at<double>(r, c);

                if(local_height != INVALID_HEIGHT) {
                    if(accumed_terrain_mapper->accumed_predicted_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {
                        float predicted_local_height = accumed_terrain_mapper->accumed_predicted_terrain_height_mat.at<double>(r,c)
                                - accumed_terrain_mapper->t_w_l(2) + accumed_terrain_mapper->reference_height;
                        if(fabs(local_height - predicted_local_height) < delta_height_thresh
                                && slope_mat.at<double>(r,c) < slope_angle_thresh) {
                            seed_array[end_idx++] = short2D(r,c);
                            chosen_mat.at<UINT8>(r,c) = 1;
                        }
                    }
                }
            }
        }
    }

    if(end_idx == 0) {
        std::cout<<"No seed points are found in CalculateROIFromAccumedMap!"<<std::endl;
        return false;
    }

    if(b_debug_mode) {
        cv::namedWindow("chosen_mat",cv::WINDOW_NORMAL);
        cv::imshow("chosen_mat",chosen_mat*255);
        cv::waitKey(-1);
    }

    INT8 idx_array[8] = {1,0,-1,0,0,1,0,-1};
    while(start_idx < end_idx) {
        short this_r = seed_array[start_idx].x;
        short this_c = seed_array[start_idx].y;

        for(int i=0; i<4; i++) {
            int r = this_r + idx_array[2*i];
            int c = this_c + idx_array[2*i + 1];
            if(r>=0 && r<slope_mat.rows && c>=0 && c<slope_mat.cols) {
                if(!chosen_mat.at<UINT8>(r,c)) {
                    if(slope_mat.at<double>(r,c) < slope_angle_thresh && slope_mat.at<double>(r,c)!=INVALID_VALUE) {
                        seed_array[end_idx++] = short2D(r,c);
                        chosen_mat.at<UINT8>(r,c) = 1;
                    }
                }
            }
        }
        start_idx++;
    }

    if(b_debug_mode) {
        cv::namedWindow("chosen_mat",cv::WINDOW_NORMAL);
        cv::imshow("chosen_mat",chosen_mat*255);
        cv::waitKey(-1);
    }

    return true;
}

void CInstantTerrainMapper::VisualizeInstantMap(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l)
{
    /// debug the terrain modeling result

    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("InstantMap");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr predicted_terrain_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr observed_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB o;

    Eigen::Vector3d local_point, rotated_local_point;
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
            double predicted_height = predicted_terrain_height_mat.at<double>(r,c);
            float diff = fabs(predicted_height - rotated_local_point(2));
            if(diff<0.15) {
                o.r = 0; o.g = 255; o.b = 0;
            }
            else {
                o.r = 255; o.g = 0 ; o.b = 0;
            }
            observed_point_cloud->points.push_back(o);
        }
    }

    for(int r=0; r<predicted_terrain_height_mat.rows; r++) {
        for(int c=0; c<predicted_terrain_height_mat.cols; c++) {
            if(predicted_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {
                o.x = (c - predicted_terrain_height_mat.cols/2) * map_resolution + map_resolution/2;
                o.y = (predicted_terrain_height_mat.rows/2 - 1 - r) * map_resolution + map_resolution/2;
                o.z = predicted_terrain_height_mat.at<double>(r,c);
                o.r = 128;
                o.g = 128;
                o.b = 128;
                predicted_terrain_cloud->points.push_back(o);
            }
        }
    }



    vis->removeAllPointClouds();
    vis->removeAllShapes();


    vis->addPointCloud(predicted_terrain_cloud, "predicted_terrain_cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "predicted_terrain_cloud");
    vis->addPointCloud(observed_point_cloud, "observed_point_cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "observed_point_cloud");

    pcl::PointXYZ p1, p2;
    float radius = 10;
    p1.z = o.z;  p2.z = o.z;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line1");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line2");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line3");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line4");

    radius = 20;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line5");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line6");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line7");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line8");

    radius = 30;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line9");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line10");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line11");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line12");

    //vis->spinOnce(100);
    vis->spin();
}




void CInstantTerrainMapper::VisualizeInstantMap2(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, Eigen::Matrix3d R_w_l)
{
    /// debug the cell's data storing capability

    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("InstantMap2");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr observed_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapstored_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB o;

    Eigen::Vector3d local_point, rotated_local_point;
    for(size_t i=0; i<point_cloud->size(); i++) {
        local_point(0) = point_cloud->points[i].x;
        local_point(1) = point_cloud->points[i].y;
        local_point(2) = point_cloud->points[i].z;

        rotated_local_point = R_w_l * local_point;

        o.x = rotated_local_point(0);
        o.y = rotated_local_point(1);
        o.z = rotated_local_point(2);
        o.r = 255;
        o.g = 0;
        o.b = 0;
        observed_point_cloud->points.push_back(o);
    }


    float cell_data[5*16];
    int num;
    for(int r=0; r<map_rows; r++) {
        for(int c=0; c<map_cols; c++) {
            int idx = r * map_cols + c;
            CInstantTerrainCell *this_cell = instant_terrain_map->cells + idx;
            if(this_cell->n > 0) {
                this_cell->PopData(cell_data, num);
                for(size_t i=0; i<num; i++) {
                    o.x = (c - observed_terrain_height_mat.cols/2) * map_resolution + map_resolution/2 + residual_x;
                    o.y = (observed_terrain_height_mat.rows/2 - 1 - r) * map_resolution + map_resolution/2 + residual_y;
                    o.z = cell_data[i];
                    o.r = 0;
                    o.g = 0;
                    o.b = 255;
                    mapstored_point_cloud->points.push_back(o);
                }

            }
        }
    }





    //////////////////////////////// error analyzer ///////////////////////////////

    int number = observed_point_cloud->points.size();
    float *M_data = new float [number*3];
    int iter = 0;
    for(int i=0; i<number; i++) {
        M_data[iter++] = observed_point_cloud->points[i].x;
        M_data[iter++] = observed_point_cloud->points[i].y;
        M_data[iter++] = observed_point_cloud->points[i].z;
    }
    FH_kdtree::KDTree *M_tree = new FH_kdtree::KDTree(M_data, 3, number);
    M_tree->build_tree(number);
    std::vector<float>         query(3);
    FH_kdtree::KDTreeResultVector result;
    int valid_number = 0;
    double sum_dist = 0;
    double max_dist = -DBL_MAX;
    for(int i=0; i<mapstored_point_cloud->points.size(); i++) {
        query[0] = mapstored_point_cloud->points[i].x;
        query[1] = mapstored_point_cloud->points[i].y;
        query[2] = mapstored_point_cloud->points[i].z;

        M_tree->n_nearest(query,1,result);
        // model point
        float nearest_x = M_tree->the_data[result[0].idx*3];
        float nearest_y = M_tree->the_data[result[0].idx*3+1];
        float nearest_z = M_tree->the_data[result[0].idx*3+2];

        double dist = sqrt((query[0] - nearest_x)*(query[0] - nearest_x) + (query[1] - nearest_y)*(query[1] - nearest_y) + (query[2] - nearest_z)*(query[2] - nearest_z));
        if(dist > max_dist)
            max_dist = dist;

        sum_dist = sum_dist + dist;
        valid_number++;
    }
    double mean_error = sum_dist/valid_number;
    std::cout<<"Mean Error: "<<mean_error<<std::endl;
    std::cout<<"Max dist error: "<<max_dist<<std::endl;
    delete[] M_data;
    delete M_tree;
    ///////////////////////////////////////////////////////////////////




    vis->removeAllPointClouds();
    vis->removeAllShapes();

    vis->addPointCloud(mapstored_point_cloud, "mapstored_point_cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "mapstored_point_cloud");
    vis->addPointCloud(observed_point_cloud, "observed_point_cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "observed_point_cloud");

    pcl::PointXYZ p1, p2;
    float radius = 10;
    p1.z = o.z;  p2.z = o.z;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line1");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line2");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line3");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line4");

    radius = 20;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line5");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line6");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line7");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line8");

    radius = 30;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line9");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line10");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line11");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line12");

    vis->spin();
}



void CInstantTerrainMapper::VisualizeAccumedMap(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
{
    /// debug the terrain modeling result

    if(vis==NULL)
        vis = new pcl::visualization::PCLVisualizer ("AccumedMap");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr predicted_terrain_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr observed_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB o;



    for(int r=0; r<observed_terrain_height_mat.rows; r++) {
        for(int c=0; c<observed_terrain_height_mat.cols; c++) {
            if(observed_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {
                o.x = (c - observed_terrain_height_mat.cols/2) * map_resolution + map_resolution/2;
                o.y = (observed_terrain_height_mat.rows/2 - 1 - r) * map_resolution + map_resolution/2;

                float predicted_height = accumed_terrain_mapper->accumed_predicted_terrain_height_mat.at<double>(r,c)
                        - accumed_terrain_mapper->t_w_l(2) + accumed_terrain_mapper->reference_height;
                float observed_height = observed_terrain_height_mat.at<double>(r,c);


                o.z = observed_height;

                if(fabs(predicted_height - observed_height)<0.15) {
                    o.r = 0;                o.g = 255;                o.b = 0;
                }
                else {
                    o.r = 255;                o.g = 0;                o.b = 0;
                }
                observed_point_cloud->points.push_back(o);
            }
        }
    }



    for(int r=0; r<predicted_terrain_height_mat.rows; r++) {
        for(int c=0; c<predicted_terrain_height_mat.cols; c++) {
            if(accumed_terrain_mapper->accumed_predicted_terrain_height_mat.at<double>(r,c) != INVALID_HEIGHT) {
                o.x = (c - predicted_terrain_height_mat.cols/2) * map_resolution + map_resolution/2;
                o.y = (predicted_terrain_height_mat.rows/2 - 1 - r) * map_resolution + map_resolution/2;
                o.z = accumed_terrain_mapper->accumed_predicted_terrain_height_mat.at<double>(r,c)
                        - accumed_terrain_mapper->t_w_l(2) + accumed_terrain_mapper->reference_height;
                o.r = 128;
                o.g = 128;
                o.b = 128;
                predicted_terrain_cloud->points.push_back(o);
            }
        }
    }



    vis->removeAllPointClouds();
    vis->removeAllShapes();


    vis->addPointCloud(predicted_terrain_cloud, "predicted_terrain_cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "predicted_terrain_cloud");
    vis->addPointCloud(observed_point_cloud, "observed_point_cloud");
    vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "observed_point_cloud");

    pcl::PointXYZ p1, p2;
    float radius = 10;
    p1.z = o.z;  p2.z = o.z;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line1");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line2");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line3");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line4");

    radius = 20;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line5");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line6");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line7");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line8");

    radius = 30;
    p1.x = radius; p1.y = radius;
    p2.x = radius; p2.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line9");
    p1.x = -radius; p1.y = -radius;    vis->addLine(p1, p2, 1, 1, 0, "line10");
    p2.x = -radius; p2.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line11");
    p1.x = radius; p1.y = radius;    vis->addLine(p1, p2, 1, 1, 0, "line12");

    vis->spin();
}


//void CInstantTerrainMapper::CalculateNormalV2(cv::Mat &terrain_map, cv::Mat &normal_mat, double map_resolution_, double normal_radius_, int normal_valid_points_num_thresh_)
//{
//    double kPredictionKernelRadius = normal_radius_;
//    int kPredictionGridSize = std::round(kPredictionKernelRadius / map_resolution_);
//    int k_valid_points_num_thresh = normal_valid_points_num_thresh_;

//    INT8 *row_idx_array = new INT8[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];
//    INT8 *col_idx_array = new INT8[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];
//    double *x_array = new double[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];
//    double *y_array = new double[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];

//    int valid_idx_num = 0;
//    for(int i=0; i<2*kPredictionGridSize + 1; i++) {    // row index
//        for(int j=0; j<2*kPredictionGridSize + 1; j++) {    // col index
//            double dist = sqrt((i - kPredictionGridSize)*(i - kPredictionGridSize) + (j - kPredictionGridSize)*(j - kPredictionGridSize)) * map_resolution;
//            if( dist <= kPredictionKernelRadius ) {
//                row_idx_array[valid_idx_num] = i - kPredictionGridSize;
//                col_idx_array[valid_idx_num] = j - kPredictionGridSize;

//                x_array[valid_idx_num] = col_idx_array[valid_idx_num] * map_resolution_;
//                y_array[valid_idx_num] = -row_idx_array[valid_idx_num] * map_resolution_;

//                valid_idx_num++;
//            }
//        }
//    }

//    for(int r=kPredictionGridSize; r<terrain_map.rows-kPredictionGridSize; r++) {
//        for(int c=kPredictionGridSize; c<terrain_map.cols-kPredictionGridSize; c++) {
//            if(terrain_map.at<double>(r,c) != INVALID_Z) {

//                Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
//                Eigen::Vector3d center(0, 0, 0);
//                Eigen::Vector3d pt;
//                int pt_size = 0;
//                double ref_height = terrain_map.at<double>(r,c);

//                for(int i=0; i<valid_idx_num; i++) {
//                    int rr = r + row_idx_array[i];
//                    int cc = c + col_idx_array[i];

//                    if(terrain_map.at<double>(rr,cc) != INVALID_Z) {

//                        pt(0) = x_array[i];
//                        pt(1) = y_array[i];
//                        pt(2) = terrain_map.at<double>(rr,cc) - ref_height;

//                        covMat += pt * pt.transpose();
//                        center += pt;

//                        pt_size++;
//                    }
//                }
//                if(pt_size>=k_valid_points_num_thresh) {
//                    center /= pt_size;
//                    covMat = covMat / pt_size - center * center.transpose();
//                    /* saes.eigenvalues()[2] is the biggest */
//                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
//                    double feat_eigen_ratio = saes.eigenvalues()[2] / saes.eigenvalues()[0];

//                    //                    if (!std::isnan(feat_eigen_ratio) && !std::isinf(feat_eigen_ratio) && feat_eigen_ratio >= normal_feat_eigen_ratio_thresh) {
//                    if (!std::isnan(feat_eigen_ratio) && !std::isinf(feat_eigen_ratio)) {
//                        Eigen::Matrix3d U = saes.eigenvectors();
//                        //                        normal_mat.at<cv::Vec3f>(r,c) = cv::Vec3f(saes.eigenvectors()[0][0], saes.eigenvectors()[0][1], saes.eigenvectors()[0][2]);
//                        normal_mat.at<cv::Vec3f>(r,c) = cv::Vec3f(U(0,0), U(1,0), U(2,0));  // the eigenvector corresponding to the smallest eigenvalue
//                    }
//                }

//            }
//        }
//    }
//}
