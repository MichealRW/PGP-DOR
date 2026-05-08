#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <stdio.h>
#include <stdlib.h>

#include "opencv2/opencv.hpp"

#include "Colors.h"

void SetVisualizationEnabled(bool b_enable_visualization);
bool GetVisualizationEnabled();

void VisualizeMat(const cv::Mat &mat_data, char *window_name, bool b_pause);

void VisualizeFloatMatrix(const cv::Mat &float_matrix, float min_height, float max_height, char *window_name, bool pause);

void VisualizeFloatMatrixWithInvalidValue(const cv::Mat &mat_data, float invalid_value, char *window_name, bool pause);


void VisualizeDoubleMatrixWithInvalidValue(double *data, int rows, int cols, double invalid_value, char *window_name, bool b_pause);

void VisualizeDoubleMatrixWithInvalidValue(double *data, int rows, int cols, double invalid_value, double min_value, double max_value, char *window_name, bool b_pause);

void VisualizeDoubleMatrixWithInvalidValue(const cv::Mat &mat_data, double invalid_value, char *window_name, bool b_pause);

void VisualizeMatWithInvalidValue(const cv::Mat &mat_data, double invalid_value, char *window_name, bool b_pause = true);

void VisualizeRangedMatWithInvalidValue(const cv::Mat &mat_data, double invalid_value, double min_height, double max_height, char *window_name, bool b_pause = true);




#endif //
