



#ifndef COORDTRANSFORMATION_H
#define COORDTRANSFORMATION_H

#include "Eigen/Core"

#include "CommenDefinition.h"

Eigen::Matrix3d YPR2RotationMatrixV2(double3D ypr);
Eigen::Matrix3d YPR2RotationMatrix(double3D ypr);
Eigen::Matrix3d YPR2RotationMatrixZYX(double3D ypr);

void R2Ang_3D_Our(Eigen::Matrix3d &R, Eigen::Vector3d &angle);
void R2Ang_3D_Our(Eigen::Matrix3f &R, Eigen::Vector3f &angle);

//void R2Ang_3D_Lidar(Eigen::Matrix3f &R, Eigen::Vector3f &angle);

void R2Ang_3D_Loam(Eigen::Matrix3f &R, Eigen::Vector3f &angle);

void loamtransform2NormalTr(float *transform_, Eigen::Matrix4f &Tr);

void NormalTr2loamtransform(Eigen::Matrix4f Tr, float* transform_);


void loamtransformSum2NormalTr(float *transformSum_, Eigen::Matrix4f &Tr);


void NormalTr2loamtransformSum(Eigen::Matrix4f &Tr, float *transformSum_);

#endif





