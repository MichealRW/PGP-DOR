
#ifndef __BGK_SMOOTHER_H__
#define __BGK_SMOOTHER_H__


#include <opencv2/opencv.hpp>

#include "CommenDefinition.h"



class BGK_Smoother
{
public:
    BGK_Smoother();
    ~BGK_Smoother();

    void SetMapResolution(double resolution_);
    double map_resolution;

    void SetMapSize(int map_rows_, int map_cols_);
    int map_cols, map_rows, map_size;

    void SetPredictionKernelRadius(double k_prediction_kernel_radius_);
    double k_prediction_kernel_radius;

    void SetValidPointsNumThresh(int k_valid_points_num_thresh_);
    int k_valid_points_num_thresh;

    void SetBilateralFilterStdValue(double k_bf_height_std_);
    double k_bf_height_std;

    void Initialize();

    int kPredictionGridSize;

    INT8 *row_idx_array;
    INT8 *col_idx_array;
    double *k_array;

    double *valid_k_array;
    double *valid_height_array;

    int valid_idx_num;

    double *sum_k;
    double *sum_ky;
    int *valid_point_num;

    double *new_weight;
    double LUT[200];

    void Run(cv::Mat &input_mat, cv::Mat &result_mat, double invalid_value, bool b_perform_bf = true);
    void Run(cv::Mat &input_mat, cv::Mat &result_mat, cv::Mat &conf_mat, double invalid_value);
};



#endif
