#ifndef COMMEN_DEFINITION_H
#define COMMEN_DEFINITION_H

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <iostream>
#include <omp.h>

#ifndef __INT64__
#define __INT64__
typedef  signed     long        INT64;
#endif

#ifndef __UINT64__
#define __UINT64__
typedef  unsigned   long        UINT64;
#endif

#ifndef __INT32__
#define __INT32__
typedef  signed     int         INT32;
#endif

#ifndef __UINT32__
#define __UINT32__
typedef  unsigned   int         UINT32;
#endif

#ifndef __INT16__
#define __INT16__
typedef  signed     short       INT16;
#endif

#ifndef __UINT16__
#define __UINT16__
typedef  unsigned   short       UINT16;
#endif

#ifndef __INT8__
#define __INT8__
typedef  signed     char        INT8;
#endif

#ifndef __UINT8__
#define __UINT8__
typedef  unsigned   char        UINT8;
#endif


#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif


#define numberOfCores 1

#define INVALID_VALUE (-123456)

#define INVALID_Z (-123456)

#define INVALID_HEIGHT (-123456)

#define INVALID_RANGE 123456

enum CELL_ATTIBUTE{
    Type_Unknown = 0,
    Type_Terrain = 1,
    Type_Gray = 2,
    Type_Obstacle = 3,
    Type_Hanging = 4,
    Type_Negative = 5
};


enum CON_Type{
    Cell_Unknown = 0,
    Cell_Valid   = 1,
    Cell_Invalid = 2
};

// INT32 int_floor(double x);
// INT32 int_floor(float x);
// inline UINT16 counter_diff(UINT16 last, UINT16 now);

inline UINT16 counter_diff(UINT16 last, UINT16 now) {
  if (now < last) {
    return (USHRT_MAX - (last - now));
  }
  return (now - last);
}

// inline INT32 int_floor(double x) {
//    return ((INT32) (x + 10000000.0)) - 10000000;
//}

inline INT32 Int_floor(float x) { return ((INT32)(x + 10000000.0)) - 10000000; }

std::string FH_any_to_string(int val);
double tic();
void toc(double t);
double toc_output(double t);


template <class T1, class T2>
float Euc_2D_dis(T1 a, T2 b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

template <class T1, class T2>
float Euc_3D_dis(T1 a, T2 b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
              (a.z - b.z) * (a.z - b.z));
}

class short2D {
 public:
  short2D();
  short2D(short _x, short _y);

  short2D &operator=(const short2D &M);

  short2D operator+(const short2D &M);
  short2D operator-(const short2D &M);

  float norm();

  short x;
  short y;
};

class int2D {
 public:
  int2D();
  int2D(int _x, int _y);

  int2D &operator=(const int2D &M);

  int2D operator+(const int2D &M);
  int2D operator-(const int2D &M);

  float norm();

  int x;
  int y;
};

class float2D {
 public:
  float2D();
  float2D(double _x, double _y);
  float2D(float _x, float _y);
  float2D(int _x, int _y);

  float2D operator+(const float2D &M);
  float2D operator-(const float2D &M);
  float2D operator*(const float value);
  float2D operator/(const float value);

  float norm();
  void setZero();
  float x;
  float y;
};

class float3D {
 public:
  float3D();
  float3D(float _x, float _y, float _z);
  float3D operator+(const float3D &M);
  float3D operator-(const float3D &M);
  float3D operator*(const float value);
  float3D operator/(const float value);

  float norm();
  void setZero();

  inline float3D crossProduct(const float3D &rkVector) const {
    return float3D(y * rkVector.z - z * rkVector.y,
                   z * rkVector.x - x * rkVector.z,
                   x * rkVector.y - y * rkVector.x);
  }

  inline float normalise() {
    float fLength = sqrt(x * x + y * y + z * z);

    // Will also work for zero-sized vectors, but will change nothing
    // We're not using epsilons because we don't need to.
    // Read http://www.ogre3d.org/forums/viewtopic.php?f=4&t=61259
    if (fLength > float(0.0f)) {
      float fInvLength = 1.0f / fLength;
      x *= fInvLength;
      y *= fInvLength;
      z *= fInvLength;
    }

    return fLength;
  }

  //    union
  //    {
  //        struct
  //        {
  //            float x;
  //            float y;
  //            float z;
  //        };
  //        float data[3];
  //    };

  float x;
  float y;
  float z;

  inline const float &operator()(unsigned int i) const {
    //      return data[i];
    if (i == 0)
      return x;
    else if (i == 1)
      return y;
    else
      return z;
  }

  inline void operator/=(float factor) {
    x /= factor;
    y /= factor;
    z /= factor;
  }
};

class double2D {
 public:
  double2D();
  double2D(double _x, double _y);
  void setZero();
  double2D operator+(const double2D &M);
  double2D operator-(const double2D &M);
  double2D operator*(const double value);
  double2D operator/(const double value);

  double norm();

  double x;
  double y;
};

class double3D {
 public:
  double3D();
  double3D(double _x, double _y, double _z);
  double x;
  double y;
  double z;

  inline double3D crossProduct( const double3D& rkVector ) const
  {
      return double3D(
          y * rkVector.z - z * rkVector.y,
          z * rkVector.x - x * rkVector.z,
          x * rkVector.y - y * rkVector.x);
  }

  inline double innerProduct( const double3D& rkVector ) const
  {
      return double(x*rkVector.x + y*rkVector.y + z*rkVector.z);
  }

  inline double normalise()
  {
      double fLength = sqrt( x * x + y * y + z * z );

      // Will also work for zero-sized vectors, but will change nothing
      // We're not using epsilons because we don't need to.
      // Read http://www.ogre3d.org/forums/viewtopic.php?f=4&t=61259
      if ( fLength > double(0.0f) )
      {
          double fInvLength = 1.0f / fLength;
          x *= fInvLength;
          y *= fInvLength;
          z *= fInvLength;
      }

      return fLength;
  }

  double3D operator+(const double3D &M);
  double3D operator-(const double3D &M);
  double3D operator*(const double value);
  double3D operator/(const double value);

  double norm();
  void setZero();
};

#endif  // FH_COMMEN_DEFINITION_H
