// The following code is adapted from the Stanford Self Driving Car Code.

#ifndef CEXPANDABLEMAP_H
#define CEXPANDABLEMAP_H

#include "string.h"
#include "misc/CommenDefinition.h"

template <typename CellType>
class CExpandableMap{
public:
    CExpandableMap(double resolution_, INT32 rows, INT32 rols);

    void Reset();

    double resolution;              // map resolution in meters
    INT32 rows_, cols_;                 // size of grid
    INT32 map_r0_, map_c0_;             // grid coordinates of lower left corner of map (X->east, Y->north)
    //INT32 array_r0_, array_c0_;         // position of upper left corner in array (X->right, Y->down)
    double center_x, center_y;


    void SetCenter(double center_x_, double center_y_);
    void SetResidual(double residual_x_, double residual_y_);
    
    double residual_x, residual_y;

    CellType *cell_;                    // actual map data
    CellType default_value_;            // default value for new cells

    CellType* GetRCLocal(INT32 r, INT32 c) const;
    CellType* GetRCLocalUnsafe(INT32 r, INT32 c) const;



    CellType* GetCell(float local_x, float local_y);
    CellType* GetCell(float local_x, float local_y, int &r, int &c);

    CellType* GetXYCell(int r, int c, float &local_x, float &local_y);
    void GetXY(int r, int c, float &local_x, float &local_y);

private:


};



template <typename CellType>
CExpandableMap<CellType>::CExpandableMap(double resolution_, INT32 rows, INT32 cols)
{
    resolution = resolution_;
    rows_ = rows;
    cols_ = cols;

    map_r0_ = 0;
    map_c0_ = 0;
    //array_r0_ = 0;
    //array_c0_ = 0;

    cell_ = new CellType [rows_*cols_];

    residual_x = 0;
    residual_y = 0;

    Reset();
}

template <typename CellType>
void CExpandableMap<CellType>::Reset()
{
    for (INT32 i = 0; i < rows_ * cols_; i++)
    memcpy(&cell_[i], &default_value_, sizeof(CellType));
}

template <typename CellType>
CellType* CExpandableMap<CellType>::GetRCLocal(INT32 r, INT32 c) const {
    if (r < 0 || c < 0 || r >= rows_ || c >= cols_)
        return NULL;

    return &cell_[r * cols_ + c];
}

template <typename CellType>
CellType* CExpandableMap<CellType>::GetRCLocalUnsafe(INT32 r, INT32 c) const {
    return &cell_[r * cols_ + c];
}


template <typename CellType>
CellType* CExpandableMap<CellType>::GetCell(float local_x, float local_y)
{

    int32_t c = cols_ / 2 + std::floor((local_x - residual_x) / resolution);
    int32_t r = rows_ / 2 - 1 - std::floor((local_y - residual_y) / resolution);
    if(r<0 || r>=rows_ || c<0 || c>=cols_)
        return NULL;
    else
        return GetRCLocalUnsafe(r,c);
}

template <typename CellType>
CellType* CExpandableMap<CellType>::GetXYCell(int r, int c, float &local_x, float &local_y) {
    if(r<0 || r>=rows_ || c<0 || c>=cols_)
        return NULL;

    GetXY(r, c, local_x, local_y);
    return GetRCLocalUnsafe(r,c);
}

template <typename CellType>
void CExpandableMap<CellType>::GetXY(int r, int c, float &local_x, float &local_y)
{
    local_x = (c - cols_/2) * resolution + resolution/2 + residual_x;
    local_y = (rows_/2 - 1 - r) * resolution + resolution/2 + residual_y;
}



////////////////////////////// add //////////////////////////////
template <typename CellType>
void CExpandableMap<CellType>::SetCenter(double center_x_, double center_y_)
{
    center_x = center_x_;
    center_y = center_y_;
}

template <typename CellType>
void CExpandableMap<CellType>::SetResidual(double residual_x_, double residual_y_)
{
    residual_x = residual_x_;
    residual_y = residual_y_;
}

#endif