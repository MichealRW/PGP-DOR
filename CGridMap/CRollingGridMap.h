
// The following code is adapted from the Stanford Self Driving Car Code.

#ifndef CROLLINGGRIDMAP_H
#define CROLLINGGRIDMAP_H


#include "misc/CommenDefinition.h"

#include "string.h"

template <typename CellType>
class CRollingGridMap {
public:
    CRollingGridMap(double resolution_, INT32 rows, INT32 cols);

//    virtual void ReadPara() = 0;

    bool ReCenter(double x, double y);
    void Reset();

    double resolution;              // map resolution in meters
    INT32 rows_, cols_;                 // size of grid
    INT32 map_r0_, map_c0_;             // grid coordinates of lower left corner of map (X->east, Y->north)
    INT32 array_r0_, array_c0_;         // position of upper left corner in array (X->right, Y->down)
    double center_x, center_y;

    CellType *cell_;                    // actual map data
    CellType default_value_;            // default value for new cells

    CellType* GetRCLocal(INT32 r, INT32 c) const;
    CellType* GetRCLocalUnsafe(INT32 r, INT32 c) const;




    void SetResidualFromLidarPose(double lidar_pose_x, double lidar_pose_y);
    void SetResidual(double residual_x_, double residual_y_);
    double residual_x, residual_y;

    CellType* GetCell(float local_x, float local_y);
    CellType* GetCell(float local_x, float local_y, int &r, int &c);

    CellType* GetXYCell(int r, int c, float &local_x, float &local_y);
    void GetXY(int r, int c, float &local_x, float &local_y);


private:
    void AddColumnEast();
    void AddColumnWest();
    void AddRowNorth();
    void AddRowSouth();

    static INT32 Wrap(INT32 x, INT32 max) {    // change x to a value lies in [0, max). Equivalent to (x%max) ??
        if (x >= max) {
            while (x >= max)
                x -= max;
        }
        else if (x < 0) {
            while (x < 0)
                x += max;
        }
        return x;
    }
};




template <typename CellType>
CRollingGridMap<CellType>::CRollingGridMap(double resolution_, INT32 rows, INT32 cols)
{
    resolution = resolution_;
    rows_ = rows;
    cols_ = cols;

    map_r0_ = 0;
    map_c0_ = 0;
    array_r0_ = 0;
    array_c0_ = 0;

    cell_ = new CellType [rows_*cols_];

    residual_x = 0;
    residual_y = 0;

    Reset();
}

template <typename CellType>
void CRollingGridMap<CellType>::Reset()
{
    for (INT32 i = 0; i < rows_ * cols_; i++)
    memcpy(&cell_[i], &default_value_, sizeof(CellType));
}

template <typename CellType>
CellType* CRollingGridMap<CellType>::GetRCLocal(INT32 r, INT32 c) const {
    if (r < 0 || c < 0 || r >= rows_ || c >= cols_)
        return NULL;
    r = Wrap(r + array_r0_, rows_);
    c = Wrap(c + array_c0_, cols_);
    return &cell_[r * cols_ + c];
}

template <typename CellType>
CellType* CRollingGridMap<CellType>::GetRCLocalUnsafe(INT32 r, INT32 c) const {
    r = Wrap(r + array_r0_, rows_);
    c = Wrap(c + array_c0_, cols_);
    return &cell_[r * cols_ + c];
}


template <typename CellType>
void CRollingGridMap<CellType>::AddColumnEast()
{
    for (INT32 r = 0; r < rows_; r++) {
        CellType* cell = &cell_[r * cols_ + array_c0_];
        memcpy(cell, &default_value_, sizeof(CellType));
    }

    map_c0_++;
    array_c0_++;
    if (array_c0_ == cols_)
        array_c0_ = 0;
}

template <typename CellType>
void CRollingGridMap<CellType>::AddColumnWest()
{
    INT32 new_array_c0 = array_c0_ - 1;
    if (new_array_c0 < 0) {new_array_c0 = cols_ - 1;}

    for (INT32 r = 0; r < rows_; r++) {
        CellType* cell = &cell_[r * cols_ + new_array_c0];
        memcpy(cell, &default_value_, sizeof(CellType));
    }

    map_c0_--;
    array_c0_ = new_array_c0;
}

template <typename CellType>
void CRollingGridMap<CellType>::AddRowSouth() {
    for (INT32 c = 0; c < cols_; c++) {
        CellType* cell = &cell_[array_r0_ * cols_ + c];
        memcpy(cell, &default_value_, sizeof(CellType));
    }

    map_r0_--;
    array_r0_++;
    if (array_r0_ == rows_)
        array_r0_ = 0;
}

template <typename CellType>
void CRollingGridMap<CellType>::AddRowNorth() {
    INT32 new_array_r0 = array_r0_ - 1;
    if (new_array_r0 < 0)
        new_array_r0 = rows_ - 1;

    for (INT32 c = 0; c < cols_; c++) {
        CellType* cell = &cell_[new_array_r0 * cols_ + c];
        memcpy(cell, &default_value_, sizeof(CellType));
    }

    map_r0_++;
    array_r0_ = new_array_r0;
}

template <typename CellType>
bool CRollingGridMap<CellType>::ReCenter(double x, double y)
{
    center_x = x;
    center_y = y;

    INT32 corner_r = std::floor(y / resolution) - rows_ / 2;
    INT32 corner_c = std::floor(x / resolution) - cols_ / 2;

    INT32 dr = corner_r - map_r0_;
    INT32 dc = corner_c - map_c0_;

    if (dr == 0 && dc == 0)
        return false;
    if (abs(dr) >= rows_ || abs(dc) >= cols_) {
        Reset();
        map_r0_ = corner_r;
        map_c0_ = corner_c;
        array_r0_ = 0;
        array_c0_ = 0;
    }
    else {
        if (dr > 0) {
            for (INT32 i = 0; i < dr; i++)
                AddRowNorth();
        }
        else if (dr < 0) {
            for (INT32 i = 0; i < abs(dr); i++)
                AddRowSouth();
        }
        if (dc > 0) {
            for (INT32 i = 0; i < dc; i++)
                AddColumnEast();
        }
        else if (dc < 0) {
            for (INT32 i = 0; i < abs(dc); i++)
                AddColumnWest();
        }
    }
    return true;
}


////////////////////////////////// added on 202220818 /////////////////////////////////////

template <typename CellType>
void CRollingGridMap<CellType>::SetResidual(double residual_x_, double residual_y_)
{
    residual_x = residual_x_;
    residual_y = residual_y_;
}


template <typename CellType>
void CRollingGridMap<CellType>::SetResidualFromLidarPose(double lidar_pose_x, double lidar_pose_y)
{
    residual_x = std::floor(lidar_pose_x / resolution) * resolution - lidar_pose_x;      // t_lidar_mapgrid
    residual_y = std::floor(lidar_pose_y / resolution) * resolution - lidar_pose_y;
}





template <typename CellType>
CellType* CRollingGridMap<CellType>::GetCell(float local_x, float local_y)
{

    int32_t c = cols_ / 2 + std::floor((local_x - residual_x) / resolution);
    int32_t r = rows_ / 2 - 1 - std::floor((local_y - residual_y) / resolution);
    if(r<0 || r>=rows_ || c<0 || c>=cols_)
        return NULL;
    else
        return GetRCLocalUnsafe(r,c);
}

template <typename CellType>
CellType* CRollingGridMap<CellType>::GetCell(float local_x, float local_y, int32_t &r, int32_t &c)
{

    c = cols_ / 2 + std::floor((local_x - residual_x) / resolution);
    r = rows_ / 2 - 1 - std::floor((local_y - residual_y) / resolution);
    if(r<0 || r>=rows_ || c<0 || c>=cols_)
        return NULL;
    else
        return GetRCLocalUnsafe(r,c);
}



template <typename CellType>
CellType* CRollingGridMap<CellType>::GetXYCell(int r, int c, float &local_x, float &local_y) {
    if(r<0 || r>=rows_ || c<0 || c>=cols_)
        return NULL;

    GetXY(r, c, local_x, local_y);
    return GetRCLocalUnsafe(r,c);
}

template <typename CellType>
void CRollingGridMap<CellType>::GetXY(int r, int c, float &local_x, float &local_y)
{
    local_x = (c - cols_/2) * resolution + resolution/2 + residual_x;
    local_y = (rows_/2 - 1 - r) * resolution + resolution/2 + residual_y;
}



#endif




