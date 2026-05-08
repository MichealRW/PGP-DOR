
#include "CoordTransformation.h"





Eigen::Matrix3d YPR2RotationMatrixV2(double3D ypr)
{
    //    R = Rot_z(yaw) * Rot_y(pitch) * Rot_x(roll)

    double eulerZ = ypr.x;
    double eulerY = ypr.y;
    double eulerX = ypr.z;

    double ci = cos(eulerX);
    double cj = cos(eulerY);
    double ch = cos(eulerZ);
    double si = sin(eulerX);
    double sj = sin(eulerY);
    double sh = sin(eulerZ);
    double cc = ci * ch;
    double cs = ci * sh;
    double sc = si * ch;
    double ss = si * sh;

    Eigen::Matrix3d R;
    R(0,0) = cj * ch; R(0,1) = sj * sc - cs; R(0,2) = sj * cc + ss;
    R(1,0) = cj * sh; R(1,1) = sj * ss + cc; R(1,2) = sj * cs - sc;
    R(2,0) = -sj; R(2,1) = cj * si; R(2,2) = cj * ci;

    return R;
}



Eigen::Matrix3d YPR2RotationMatrix(double3D ypr)
{

    Eigen::Matrix3d Rx,Ry,Rz,R_w_l;
    double crz = cos(ypr.x);
    double srz = sin(ypr.x);
    double crx = cos(ypr.y);
    double srx = sin(ypr.y);
    double cry = cos(ypr.z);
    double sry = sin(ypr.z);
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_w_l = Rz*Rx*Ry; // R_azimuth * R_pitch * R_roll

    return R_w_l;
}

Eigen::Matrix3d YPR2RotationMatrixZYX(double3D ypr)
{

    Eigen::Matrix3d Rx,Ry,Rz,R_w_l;
    double crz = cos(ypr.x);
    double srz = sin(ypr.x);
    double cry = cos(ypr.y);
    double sry = sin(ypr.y);
    double crx = cos(ypr.z);
    double srx = sin(ypr.z);
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_w_l = Rz*Ry*Rx; // R_azimuth * R_pitch * R_roll

    return R_w_l;
}


void R2Ang_3D_Our(Eigen::Matrix3d &R, Eigen::Vector3d &angle)
{
    // Z1X2Y3, applicable for our coordinate (X->right, Y->forward, Z->upward)
    //    angle[0] = yaw; angle[1] = pitch; angle[2] = roll
    angle(1) = asin(R(2,1));
    double cp = cos(angle(1));
    angle(2) = -atan2((R(2,0)/cp),(R(2,2)/cp));
    angle(0) = -atan2((R(0,1)/cp),(R(1,1)/cp));
}


void R2Ang_3D_Our(Eigen::Matrix3f &R, Eigen::Vector3f &angle)
{
    // Z1X2Y3, applicable for our coordinate (X->right, Y->forward, Z->upward)
    //    angle[0] = yaw; angle[1] = pitch; angle[2] = roll
    angle(1) = asin(R(2,1));
    float cp = cos(angle(1));
    angle(2) = -atan2((R(2,0)/cp),(R(2,2)/cp));
    angle(0) = -atan2((R(0,1)/cp),(R(1,1)/cp));
}

//void R2Ang_3D_Lidar(Eigen::Matrix3f &R, Eigen::Vector3f &angle)
//{
//    // Z1Y2X3, applicable for lidar coordinate (X->forward, Y->left, Z->upward)
//    //    angle[0] = yaw; angle[1] = pitch; angle[2] = roll
//    angle(1) = -asin(R(2,0));
//    float cp = cos(angle(1));
//    angle(2) = atan2((R(2,1)/cp),(R(2,2)/cp));
//    angle(0) = atan2((R(1,0)/cp),(R(0,0)/cp));
//}

void R2Ang_3D_Loam(Eigen::Matrix3f &R, Eigen::Vector3f &angle)
{
    // Y1X2Z3, applicable for loam coordinate (X->left, Y->upward, Z->forward)
    //    angle[0] = yaw; angle[1] = pitch; angle[2] = roll
    angle(1) = -asin(R(1,2));
    float cp = cos(angle(1));
    angle(2) = atan2((R(1,0)/cp),(R(1,1)/cp));
    angle(0) = atan2((R(0,2)/cp),(R(2,2)/cp));
}


void loamtransform2NormalTr(float *transform_, Eigen::Matrix4f &Tr)
{
    // transform_: Tr_BA in loam coordinate (left-up-forward)
    // Tr: Tr_AB in our coordinate (right-forward-up)

    double cry = cos(-transform_[1]);
    double sry = sin(-transform_[1]);
    double crx = cos(-transform_[0]);
    double srx = sin(-transform_[0]);
    double crz = cos(-transform_[2]);
    double srz = sin(-transform_[2]);

    Eigen::Matrix3f Rx,Ry,Rz,R_AB_LOAM,R;
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_AB_LOAM = Ry*Rx*Rz;

    Eigen::Vector3f t_BA,t_AB_LOAM;

    t_BA<<transform_[3],transform_[4],transform_[5];
    t_AB_LOAM = -R_AB_LOAM*t_BA;


    if(0) {
        double c_a = cos(-transform_[1]);  // aizmuth-->rot z
        double s_a = sin(-transform_[1]);
        double c_p = cos(transform_[0]);  // pitch---> rot x
        double s_p = sin(transform_[0]);
        double c_r = cos(-transform_[2]);  // roll---> rot y
        double s_r = sin(-transform_[2]);
        Ry<<c_r, 0, s_r,
                0 ,1, 0,
                -s_r, 0, c_r;
        Rx<<1, 0, 0,
                0, c_p, -s_p,
                0, s_p, c_p;
        Rz<<c_a, -s_a, 0,
                s_a, c_a, 0,
                0 , 0, 1;
        R = Rz*Rx*Ry;  // firstly azimuth, next pitch, finally roll

        Tr.setZero();
        Tr.topLeftCorner<3,3>() = R;
        Tr(0,3) = -t_AB_LOAM[0];
        Tr(1,3) = t_AB_LOAM[2];
        Tr(2,3) = t_AB_LOAM[1];
        Tr(3,3) = 1;
    }
    else {
        Eigen::Matrix3f R_loam_our;
        R_loam_our<<-1, 0, 0,
                0, 0, 1,
                0, 1, 0;
        Eigen::Matrix3f R_our_loam = R_loam_our.transpose();

        Eigen::Matrix3f R_AB_our = R_our_loam * R_AB_LOAM * R_loam_our;
        Eigen::Vector3f t_AB_our = R_our_loam * t_AB_LOAM;

        Tr.setZero();
        Tr.topLeftCorner<3,3>() = R_AB_our;
        Tr.topRightCorner<3,1>() = t_AB_our;
        Tr(3,3) = 1;
    }
}


void NormalTr2loamtransform(Eigen::Matrix4f Tr, float* transform_)
{
    // Tr: Tr_AB in our coordinate (right-forward-up)
    // transform_: Tr_BA in loam coordinate (left-up-forward)


    if(0) {
        Eigen::Vector3f ang;
        Eigen::Matrix3f R = Tr.topLeftCorner<3,3>();
        R2Ang_3D_Our(R, ang);

        transform_[0] = ang(1); // -(-pitch)
        transform_[1] = -ang(0); // -yaw
        transform_[2] = -ang(2); // -roll

        Eigen::Vector3f T_AB_LOAM, T_BA_LOAM;
        T_AB_LOAM[0] = -Tr(0,3);
        T_AB_LOAM[1] = Tr(2,3);
        T_AB_LOAM[2] = Tr(1,3);

        double cry = cos(-transform_[1]);
        double sry = sin(-transform_[1]);
        double crx = cos(-transform_[0]);
        double srx = sin(-transform_[0]);
        double crz = cos(-transform_[2]);
        double srz = sin(-transform_[2]);

        Eigen::Matrix3f Rx,Ry,Rz,R_AB;
        Ry<<cry, 0, sry,
                0 ,1, 0,
                -sry, 0, cry;
        Rx<<1, 0, 0,
                0, crx, -srx,
                0, srx, crx;
        Rz<<crz, -srz, 0,
                srz, crz, 0,
                0 , 0, 1;
        R_AB = Ry*Rx*Rz;

        T_BA_LOAM = -R_AB.transpose() * T_AB_LOAM;    // T_BA =  -R_BA * T_AB

        transform_[3] = T_BA_LOAM[0];
        transform_[4] = T_BA_LOAM[1];
        transform_[5] = T_BA_LOAM[2];
    }
    else {
        Eigen::Matrix4f Tr_loam_our;
        Tr_loam_our<<-1, 0, 0, 0,
                0, 0, 1, 0,
                0, 1, 0, 0,
                0, 0, 0, 1;
        Eigen::Matrix4f Tr_our_loam;
//        Tr_our_loam = Tr_loam_our.inverse();
        Tr_our_loam<<-1, 0, 0, 0,
                0, 0, 1, 0,
                0, 1, 0, 0,
                0, 0, 0, 1;
        Eigen::Matrix4f Tr_AB_LOAM = Tr_loam_our * Tr * Tr_our_loam;

        Eigen::Matrix3f R_AB = Tr_AB_LOAM.topLeftCorner<3,3>();
        Eigen::Vector3f t_AB = Tr_AB_LOAM.topRightCorner<3,1>();
        Eigen::Vector3f t_BA = -R_AB.transpose() * t_AB;

        Eigen::Vector3f angle_ypr;
        R2Ang_3D_Loam(R_AB, angle_ypr);
        transform_[0] = -angle_ypr(1);
        transform_[1] = -angle_ypr(0);
        transform_[2] = -angle_ypr(2);

        transform_[3] = t_BA(0);
        transform_[4] = t_BA(1);
        transform_[5] = t_BA(2);
    }
}




void loamtransformSum2NormalTr(float *transformSum_, Eigen::Matrix4f &Tr)
{
    // transformSum: Tr_w_l in loam coordinate (left-up-forward)
    // Tr: Tr_w_l in our coordinate (right-forward-up)

    double crx = cos(transformSum_[0]);
    double srx = sin(transformSum_[0]);
    double cry = cos(transformSum_[1]);
    double sry = sin(transformSum_[1]);
    double crz = cos(transformSum_[2]);
    double srz = sin(transformSum_[2]);

    Eigen::Matrix3f Rx,Ry,Rz,R_w_l_in_loam;
    Ry<<cry, 0, sry,
            0 ,1, 0,
            -sry, 0, cry;
    Rx<<1, 0, 0,
            0, crx, -srx,
            0, srx, crx;
    Rz<<crz, -srz, 0,
            srz, crz, 0,
            0 , 0, 1;
    R_w_l_in_loam = Ry*Rx*Rz;

    Eigen::Matrix3f R_loam_our, R_our_loam;
    R_loam_our<<-1, 0, 0,
            0, 0, 1,
            0, 1, 0;
    R_our_loam = R_loam_our.transpose();



    Eigen::Matrix3f R_w_l_in_our = R_our_loam * R_w_l_in_loam * R_loam_our;
    Eigen::Vector3f t_w_l_in_loam;
    t_w_l_in_loam<<transformSum_[3], transformSum_[4], transformSum_[5];

    Eigen::Vector3f t_w_l_in_our = R_our_loam * t_w_l_in_loam;

    Tr.topLeftCorner<3,3>() = R_w_l_in_our;
    Tr.topRightCorner<3,1>() = t_w_l_in_our;
    Tr(3,0) = 0; Tr(3,1) = 0; Tr(3,2) = 0; Tr(3,3) = 1;
}


void NormalTr2loamtransformSum(Eigen::Matrix4f &Tr, float *transformSum_)
{
    // Tr: Tr_w_l in our coordinate (right-forward-up)
    // transformSum: Tr_w_l in loam coordinate (left-up-forward)

    Eigen::Matrix4f Tr_loam_our;
    Tr_loam_our<<-1, 0, 0, 0,
            0, 0, 1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
    Eigen::Matrix4f Tr_our_loam;
//        Tr_our_loam = Tr_loam_our.inverse();
    Tr_our_loam<<-1, 0, 0, 0,
            0, 0, 1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;
    Eigen::Matrix4f Tr_w_l_LOAM = Tr_loam_our * Tr * Tr_our_loam;


    Eigen::Matrix3f R_w_l = Tr_w_l_LOAM.topLeftCorner<3,3>();
    Eigen::Vector3f t_w_l = Tr_w_l_LOAM.topRightCorner<3,1>();

    Eigen::Vector3f angle_ypr;
    R2Ang_3D_Loam(R_w_l, angle_ypr);
    transformSum_[0] = angle_ypr(1);
    transformSum_[1] = angle_ypr(0);
    transformSum_[2] = angle_ypr(2);

    transformSum_[3] = t_w_l(0);
    transformSum_[4] = t_w_l(1);
    transformSum_[5] = t_w_l(2);
}























