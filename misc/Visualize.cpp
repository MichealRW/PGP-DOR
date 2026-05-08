
#include "Visualize.h"

namespace {

bool g_b_enable_visualization = false;

}  // namespace

void SetVisualizationEnabled(bool b_enable_visualization)
{
    g_b_enable_visualization = b_enable_visualization;
}

bool GetVisualizationEnabled()
{
    return g_b_enable_visualization;
}


void VisualizeMat(const cv::Mat &img, char *window_name, bool b_pause)
{
    if (!GetVisualizationEnabled()) {
        return;
    }
    cv::namedWindow(window_name,cv::WINDOW_NORMAL);
    cv::imshow(window_name,img);
    if(b_pause)
        cv::waitKey(-1);
    else
        cv::waitKey(4);
}



void VisualizeFloatMatrix(const cv::Mat &float_matrix, float min_height, float max_height, char *window_name, bool b_pause)
{
    if (!GetVisualizationEnabled()) {
        return;
    }
    cv::Mat img = cv::Mat::zeros(float_matrix.rows,float_matrix.cols,CV_8UC3);

    for(int r=0;r<img.rows;r++) {
        for(int c=0;c<img.cols;c++) {
            float ratio = (float_matrix.at<float>(r,c) - min_height) / (max_height - min_height);
            if(ratio<0) ratio = 0;
            if(ratio>1) ratio = 1;
            int color_idx = ratio * 639;
            img.at<cv::Vec3b>(r,c) = cv::Vec3b(jet_color_map[color_idx][2], jet_color_map[color_idx][1], jet_color_map[color_idx][0]);  // bgr instead of rgb
        }
    }

    cv::namedWindow(window_name,cv::WINDOW_NORMAL);
    cv::imshow(window_name,img);
    if(b_pause)
        cv::waitKey(-1);
    else
        cv::waitKey(4);
}




void VisualizeFloatMatrixWithInvalidValue(const cv::Mat &mat_data, float invalid_value, char *window_name, bool b_pause)
{
    if (!GetVisualizationEnabled()) {
        return;
    }
    int rows = mat_data.rows;
    int cols = mat_data.cols;

    cv::Mat img = cv::Mat::zeros(rows,cols,CV_8UC3);

    float min_value = FLT_MAX, max_value = FLT_MIN;

    for(int r=0; r<rows; r++) {
        for(int c=0; c<cols; c++) {
            float value = mat_data.at<float>(r,c);
            if(value != invalid_value) {
                if(value>max_value)
                    max_value = value;
                if(value<min_value)
                    min_value = value;
            }
        }
    }

    for(int r=0; r<rows; r++) {
        for(int c=0; c<cols; c++) {
            float value = mat_data.at<float>(r,c);
            if(value != invalid_value) {
                float ratio = (value - min_value) / (max_value - min_value);
                if(ratio<0) ratio = 0;
                if(ratio>1) ratio = 1;
                int color_idx = ratio * 639;
                img.at<cv::Vec3b>(r,c) = cv::Vec3b(jet_color_map[color_idx][2], jet_color_map[color_idx][1], jet_color_map[color_idx][0]);  // bgr instead of rgb
            }
        }
    }

    cv::namedWindow(window_name,cv::WINDOW_NORMAL);
    cv::imshow(window_name,img);
    if(b_pause)
        cv::waitKey(-1);
    else
        cv::waitKey(4);
}



void VisualizeDoubleMatrixWithInvalidValue(double *data, int rows, int cols, double invalid_value, char *window_name, bool b_pause)
{
    if (!GetVisualizationEnabled()) {
        return;
    }
    cv::Mat img = cv::Mat::zeros(rows,cols,CV_8UC3);

    double min_value = DBL_MAX, max_value = DBL_MIN;

    for(int r=0; r<rows; r++) {
        for(int c=0; c<cols; c++) {
            int idx =  r*cols + c;
            if(data[idx] != invalid_value) {
                if(data[idx]>max_value)
                    max_value = data[idx];
                if(data[idx]<min_value)
                    min_value = data[idx];
            }
        }
    }

    for(int r=0; r<rows; r++) {
        for(int c=0; c<cols; c++) {
            int idx =  r*cols + c;
            if(data[idx] != invalid_value) {
                float ratio = (data[idx] - min_value) / (max_value - min_value);
                if(ratio<0) ratio = 0;
                if(ratio>1) ratio = 1;
                int color_idx = ratio * 639;
                img.at<cv::Vec3b>(r,c) = cv::Vec3b(jet_color_map[color_idx][2], jet_color_map[color_idx][1], jet_color_map[color_idx][0]);  // bgr instead of rgb
            }
        }
    }

    cv::namedWindow(window_name,cv::WINDOW_NORMAL);
    cv::imshow(window_name,img);
    if(b_pause)
        cv::waitKey(-1);
    else
        cv::waitKey(4);
}


void VisualizeDoubleMatrixWithInvalidValue(double *data, int rows, int cols, double invalid_value, double min_value, double max_value, char *window_name, bool b_pause)
{
    if (!GetVisualizationEnabled()) {
        return;
    }
    cv::Mat img = cv::Mat::zeros(rows,cols,CV_8UC3);
    for(int r=0; r<rows; r++) {
        for(int c=0; c<cols; c++) {
            int idx =  r*cols + c;
            if(data[idx] != invalid_value) {
                float ratio = (data[idx] - min_value) / (max_value - min_value);
                if(ratio<0) ratio = 0;
                if(ratio>1) ratio = 1;
                int color_idx = ratio * 639;
                img.at<cv::Vec3b>(r,c) = cv::Vec3b(jet_color_map[color_idx][2], jet_color_map[color_idx][1], jet_color_map[color_idx][0]);  // bgr instead of rgb
            }
        }
    }

    cv::namedWindow(window_name,cv::WINDOW_NORMAL);
    cv::imshow(window_name,img);
    if(b_pause)
        cv::waitKey(-1);
    else
        cv::waitKey(4);
}


void VisualizeDoubleMatrixWithInvalidValue(const cv::Mat &mat_data, double invalid_value, char *window_name, bool b_pause)
{
    if (!GetVisualizationEnabled()) {
        return;
    }
    int rows = mat_data.rows;
    int cols = mat_data.cols;

    cv::Mat img = cv::Mat::zeros(rows,cols,CV_8UC3);

    double min_value = DBL_MAX, max_value = DBL_MIN;

    for(int r=0; r<rows; r++) {
        for(int c=0; c<cols; c++) {
            double value = mat_data.at<double>(r,c);
            if(value != invalid_value) {
                if(value>max_value)
                    max_value = value;
                if(value<min_value)
                    min_value = value;
            }
        }
    }

    for(int r=0; r<rows; r++) {
        for(int c=0; c<cols; c++) {
            double value = mat_data.at<double>(r,c);
            if(value != invalid_value) {
                float ratio = (value - min_value) / (max_value - min_value);
                if(ratio<0) ratio = 0;
                if(ratio>1) ratio = 1;
                int color_idx = ratio * 639;
                img.at<cv::Vec3b>(r,c) = cv::Vec3b(jet_color_map[color_idx][2], jet_color_map[color_idx][1], jet_color_map[color_idx][0]);  // bgr instead of rgb
            }
        }
    }

    cv::namedWindow(window_name,cv::WINDOW_NORMAL);
    cv::imshow(window_name,img);
    if(b_pause)
        cv::waitKey(-1);
    else
        cv::waitKey(4);
}


void VisualizeMatWithInvalidValue(const cv::Mat &mat_data, double invalid_value, char *window_name, bool b_pause)
{
    int cols = mat_data.cols;
    size_t mat_step = mat_data.step;

    if(mat_step / cols == 8)
        VisualizeDoubleMatrixWithInvalidValue(mat_data, invalid_value, window_name, b_pause);
    else if(mat_step / cols == 4)
        VisualizeFloatMatrixWithInvalidValue(mat_data, invalid_value, window_name, b_pause);
    else {
        std::cout<<"Unsupported type for function VisualizeMatWithInvalidValue."<<std::endl;
        abort();
    }
}


void VisualizeRangedMatWithInvalidValue(const cv::Mat &mat_data, double invalid_value, double min_value, double max_value, char *window_name, bool b_pause)
{
    if (!GetVisualizationEnabled()) {
        return;
    }
    int cols = mat_data.cols;
    size_t mat_step = mat_data.step;

    bool is_double;
    if(mat_step / cols == 8)
        is_double = true;
    else if(mat_step / cols == 4)
        is_double = false;
    else {
        std::cout<<"Unsupported type for function VisualizeMatWithInvalidValue."<<std::endl;
        abort();
    }

    cv::Mat img = cv::Mat::zeros(mat_data.rows,mat_data.cols,CV_8UC3);
    float ratio;
    for(int r=0;r<img.rows;r++) {
        for(int c=0;c<img.cols;c++) {
            if(is_double)
                ratio = (mat_data.at<double>(r,c) - min_value) / (max_value - min_value);
            else
                ratio = (mat_data.at<float>(r,c) - min_value) / (max_value - min_value);

            if(ratio<0) ratio = 0;
            if(ratio>1) ratio = 1;
            int color_idx = ratio * 639;
            img.at<cv::Vec3b>(r,c) = cv::Vec3b(jet_color_map[color_idx][2], jet_color_map[color_idx][1], jet_color_map[color_idx][0]);  // bgr instead of rgb
        }
    }

    cv::namedWindow(window_name,cv::WINDOW_NORMAL);
    cv::imshow(window_name,img);
    if(b_pause)
        cv::waitKey(-1);
    else
        cv::waitKey(4);
}



