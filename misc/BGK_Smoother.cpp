


#include "BGK_Smoother.h"






BGK_Smoother::BGK_Smoother()
{
}

BGK_Smoother::~BGK_Smoother()
{
    delete [] row_idx_array;
    delete [] col_idx_array;
    delete [] k_array;
    delete [] valid_k_array;
    delete [] valid_height_array;

    delete [] new_weight;

    delete [] sum_k;
    delete [] sum_ky;
    delete [] valid_point_num;
}

void BGK_Smoother::SetMapResolution(double map_resolution_)
{
    map_resolution = map_resolution_;
}


void BGK_Smoother::SetMapSize(int map_rows_, int map_cols_)
{
    map_rows = map_rows_;
    map_cols = map_cols_;
    map_size = map_rows * map_cols;
}


void BGK_Smoother::SetPredictionKernelRadius(double k_prediction_kernel_radius_)
{
    k_prediction_kernel_radius = k_prediction_kernel_radius_;
}

void BGK_Smoother::SetValidPointsNumThresh(int k_valid_points_num_thresh_)
{
    k_valid_points_num_thresh = k_valid_points_num_thresh_;
}

void BGK_Smoother::SetBilateralFilterStdValue(double k_bf_height_std_)
{
    k_bf_height_std = k_bf_height_std_;
}


void BGK_Smoother::Initialize()
{
    kPredictionGridSize = int(k_prediction_kernel_radius / map_resolution);

    row_idx_array = new INT8[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];
    col_idx_array = new INT8[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];
    k_array = new double[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];

    valid_k_array = new double[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];
    valid_height_array = new double[(2*kPredictionGridSize+1) * (2*kPredictionGridSize+1)];

    valid_idx_num = 0;
    for(int i=0; i<2*kPredictionGridSize + 1; i++) {    // row index
        for(int j=0; j<2*kPredictionGridSize + 1; j++) {    // col index
            double dist = sqrt((i - kPredictionGridSize)*(i - kPredictionGridSize) + (j - kPredictionGridSize)*(j - kPredictionGridSize)) * map_resolution;
            if( dist <= k_prediction_kernel_radius ) {
                row_idx_array[valid_idx_num] = i - kPredictionGridSize;
                col_idx_array[valid_idx_num] = j - kPredictionGridSize;

                double d = dist / (k_prediction_kernel_radius + 0.01);
                double k = (2 + cos(2.0f * PI * d)) / 3 * (1 - d) + 1/(2*PI) * sin(2.0f * PI * d);
                k_array[valid_idx_num] = k;

                valid_idx_num++;
            }
        }
    }

    new_weight = new double [200 * valid_idx_num];
    for(int i=0; i<200; i++) {
        LUT[i] = exp(-0.01*double(i)*0.01*double(i) / (2*k_bf_height_std*k_bf_height_std));
        for(int k=0; k<valid_idx_num; k++)
            new_weight[i*valid_idx_num + k] = LUT[i]*k_array[k];
    }


    sum_k = new double [map_size];
    sum_ky = new double [map_size];
    valid_point_num = new int [map_size];
}


void BGK_Smoother::Run(cv::Mat &input_mat, cv::Mat &result_mat, double invalid_value, bool b_perform_bf)
{

    assert(input_mat.rows == map_rows);
    assert(input_mat.cols == map_cols);

    assert(input_mat.type() == CV_64FC1);


    result_mat = cv::Mat::zeros(input_mat.rows, input_mat.cols, CV_64FC1) + invalid_value;

    for(int i=0; i<map_size; i++) {
        sum_k[i] = 0;
        sum_ky[i] = 0;
        valid_point_num[i] = 0;
    }

    for(int r=0; r<input_mat.rows; r++) {
        for(int c=0; c<input_mat.cols; c++) {
            if(input_mat.at<double>(r,c) != invalid_value){
                for(int i=0; i<valid_idx_num; i++) {
                    int rr = r + row_idx_array[i];
                    int cc = c + col_idx_array[i];
                    if(rr >=0 && rr<map_rows && cc>=0 && cc<map_cols) {
                        int idx_neighboor = rr*map_cols + cc;
                        sum_ky[idx_neighboor] += k_array[i]*input_mat.at<double>(r,c);

                        sum_k[idx_neighboor] += k_array[i];
                        valid_point_num[idx_neighboor]++;
                    }
                }
            }
        }
    }

    for(int r=0; r<input_mat.rows; r++) {
        for(int c=0; c<input_mat.cols; c++) {
            int idx =  r*input_mat.cols + c;
            if(sum_k[idx]>0 && valid_point_num[idx]>k_valid_points_num_thresh) {
                result_mat.at<double>(r,c) = sum_ky[idx] / sum_k[idx];
            }
        }
    }


    if(b_perform_bf) {
        //////////////////////////////////////////////////////////////////////////////////////
        // Bilateral Filtering (Edge-preserving)
        for(int i=0; i<map_size; i++) {
            sum_k[i] = 0;
            sum_ky[i] = 0;
            valid_point_num[i] = 0;
        }

        for(int r=0; r<input_mat.rows; r++) {
            for(int c=0; c<input_mat.cols; c++) {
                if(input_mat.at<double>(r,c) != invalid_value){
                    for(int i=0; i<valid_idx_num; i++) {
                        int rr = r + row_idx_array[i];
                        int cc = c + col_idx_array[i];

                        if(rr >=0 && rr<map_rows && cc>=0 && cc<map_cols && result_mat.at<double>(rr,cc) != invalid_value) {
                            int idx_neighboor = rr*map_cols + cc;
                            double diff = fabs(input_mat.at<double>(r,c) - result_mat.at<double>(rr,cc));
                            int LUT_idx = std::min(199, int(diff * 100.0f));
                            sum_ky[idx_neighboor] += new_weight[LUT_idx*valid_idx_num+i]*input_mat.at<double>(r,c);
                            sum_k[idx_neighboor] += new_weight[LUT_idx*valid_idx_num+i];
                            valid_point_num[idx_neighboor]++;
                        }

                    }
                }

            }
        }

        result_mat = cv::Mat::zeros(input_mat.rows, input_mat.cols, CV_64FC1) + invalid_value;


        for(int r=0; r<input_mat.rows; r++) {
            for(int c=0; c<input_mat.cols; c++) {
                int idx =  r*input_mat.cols + c;
                if(sum_k[idx]>0 && valid_point_num[idx]>k_valid_points_num_thresh) {
                    result_mat.at<double>(r,c) = sum_ky[idx] / sum_k[idx];
                }
            }
        }
    }
}

void BGK_Smoother::Run(cv::Mat &input_mat, cv::Mat &result_mat, cv::Mat &conf_mat, double invalid_value)
{

    Run(input_mat, result_mat, invalid_value, false);

//////////////////////////////////////////////////////////////////////////////////////////
    for(int r=0; r<input_mat.rows; r++) {
        for(int c=0; c<input_mat.cols; c++) {
            int idx =  r*input_mat.cols + c;
            if(sum_k[idx]>0 && valid_point_num[idx]>k_valid_points_num_thresh) {
                conf_mat.at<double>(r,c) = sum_k[idx];
            }
        }
    }
}
