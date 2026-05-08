

#include "CommenDefinition.h"

std::string FH_any_to_string(int val) {
  char buf[10];
  const int radix = 10;

  char *p;
  int a;  // every digit
  int len;
  char *b;  // start of the digit char
  char temp;

  p = buf;

  if (val < 0) {
    *p++ = '-';
    val = 0 - val;
  }

  b = p;

  do {
    a = val % radix;
    val /= radix;

    *p++ = a + '0';

  } while (val > 0);

  len = (int)(p - buf);

  *p-- = 0;

  // swap
  do {
    temp = *p;
    *p = *b;
    *b = temp;
    --p;
    ++b;

  } while (b < p);

  std::string tmp_str = std::string(buf);
  return tmp_str;
}

double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec) / 1000000.);
}

void toc(double t) {
  double s = tic();
  std::cout << std::max(0., (s - t) * 1000) << " ms" << std::endl;
}

double toc_output(double t) {
    double s = tic();
    std::cout<<std::max(0., (s-t)*1000)<<" ms"<<std::endl;
    return std::max(0., (s-t)*1000);
}


short2D::short2D() {
  x = 0;
  y = 0;
}

short2D::short2D(short _x, short _y) {
  x = _x;
  y = _y;
}

short2D &short2D::operator=(const short2D &M) {
  x = M.x;
  y = M.y;
  return *this;
}

short2D short2D::operator+(const short2D &M) {
  const short2D &A = *this;
  const short2D &B = M;

  short2D C;

  C.x = A.x + B.x;
  C.y = A.y + B.y;

  return C;
}

short2D short2D::operator-(const short2D &M) {
  const short2D &A = *this;
  const short2D &B = M;

  short2D C;

  C.x = A.x - B.x;
  C.y = A.y - B.y;

  return C;
}

float short2D::norm() {
  return sqrt(float(x) * float(x) + float(y) * float(y));
}

int2D::int2D() {
  x = 0;
  y = 0;
}

int2D::int2D(int _x, int _y) {
  x = _x;
  y = _y;
}

int2D &int2D::operator=(const int2D &M) {
  x = M.x;
  y = M.y;
  return *this;
}

int2D int2D::operator+(const int2D &M) {
  const int2D &A = *this;
  const int2D &B = M;

  int2D C;

  C.x = A.x + B.x;
  C.y = A.y + B.y;

  return C;
}

int2D int2D::operator-(const int2D &M) {
  const int2D &A = *this;
  const int2D &B = M;

  int2D C;

  C.x = A.x - B.x;
  C.y = A.y - B.y;

  return C;
}

float int2D::norm() { return sqrt(float(x) * float(x) + float(y) * float(y)); }

float2D::float2D() {
  x = 0;
  y = 0;
}

float2D::float2D(double _x, double _y) {
  x = float(_x);
  y = float(_y);
}

float2D::float2D(float _x, float _y) {
  x = _x;
  y = _y;
}

float2D::float2D(int _x, int _y) {
  x = float(_x);
  y = float(_y);
}

float2D float2D::operator+(const float2D &M) {
  const float2D &A = *this;
  const float2D &B = M;

  float2D C;

  C.x = A.x + B.x;
  C.y = A.y + B.y;

  return C;
}

float2D float2D::operator-(const float2D &M) {
  const float2D &A = *this;
  const float2D &B = M;

  float2D C;

  C.x = A.x - B.x;
  C.y = A.y - B.y;

  return C;
}

float2D float2D::operator*(const float value) {
  const float2D &A = *this;

  float2D C;

  C.x = A.x * value;
  C.y = A.y * value;

  return C;
}

float2D float2D::operator/(const float value) {
  const float2D &A = *this;

  float2D C;

  C.x = A.x / value;
  C.y = A.y / value;

  return C;
}

float float2D::norm() { return sqrt(x * x + y * y); }

void float2D::setZero() {
  x = 0;
  y = 0;
}

float3D::float3D() {
  x = 0;
  y = 0;
  z = 0;
}

float3D::float3D(float _x, float _y, float _z) {
  x = _x;
  y = _y;
  z = _z;
}

float3D float3D::operator+(const float3D &M) {
  const float3D &A = *this;
  const float3D &B = M;

  float3D C;

  C.x = A.x + B.x;
  C.y = A.y + B.y;
  C.z = A.z + B.z;

  return C;
}

float3D float3D::operator-(const float3D &M) {
  const float3D &A = *this;
  const float3D &B = M;

  float3D C;

  C.x = A.x - B.x;
  C.y = A.y - B.y;
  C.z = A.z - B.z;

  return C;
}

float3D float3D::operator*(const float value) {
  const float3D &A = *this;

  float3D C;

  C.x = A.x * value;
  C.y = A.y * value;
  C.z = A.z * value;

  return C;
}

float3D float3D::operator/(const float value) {
  const float3D &A = *this;

  float3D C;

  C.x = A.x / value;
  C.y = A.y / value;
  C.z = A.z / value;

  return C;
}

float float3D::norm() { return sqrt(x * x + y * y + z * z); }

void float3D::setZero() {
  x = 0;
  y = 0;
  z = 0;
}

double2D::double2D() {
  x = 0;
  y = 0;
}

double2D::double2D(double _x, double _y) {
  x = _x;
  y = _y;
}

void double2D::setZero() {
  x = 0;
  y = 0;
}

double2D double2D::operator+(const double2D &M) {
  const double2D &A = *this;
  const double2D &B = M;

  double2D C;

  C.x = A.x + B.x;
  C.y = A.y + B.y;

  return C;
}

double2D double2D::operator-(const double2D &M) {
  const double2D &A = *this;
  const double2D &B = M;

  double2D C;

  C.x = A.x - B.x;
  C.y = A.y - B.y;

  return C;
}

double2D double2D::operator*(const double value) {
  const double2D &A = *this;

  double2D C;

  C.x = A.x * value;
  C.y = A.y * value;

  return C;
}

double2D double2D::operator/(const double value) {
  const double2D &A = *this;

  double2D C;

  C.x = A.x / value;
  C.y = A.y / value;

  return C;
}

double double2D::norm() { return sqrt(x * x + y * y); }

double3D::double3D() {
  x = 0;
  y = 0;
  z = 0;
}

double3D::double3D(double _x, double _y, double _z) {
  x = _x;
  y = _y;
  z = _z;
}


double3D double3D::operator+ (const double3D &M) {
  const double3D &A = *this;
  const double3D &B = M;

  double3D C;

  C.x = A.x + B.x;
  C.y = A.y + B.y;
  C.z = A.z + B.z;

  return C;
}

double3D double3D::operator- (const double3D &M) {
  const double3D &A = *this;
  const double3D &B = M;

  double3D C;

  C.x = A.x - B.x;
  C.y = A.y - B.y;
  C.z = A.z - B.z;

  return C;
}

double3D double3D::operator* (const double value) {
  const double3D &A = *this;

  double3D C;

  C.x = A.x * value;
  C.y = A.y * value;
  C.z = A.z * value;

  return C;
}

double3D double3D::operator/ (const double value) {
  const double3D &A = *this;

  double3D C;

  C.x = A.x / value;
  C.y = A.y / value;
  C.z = A.z / value;

  return C;
}

double double3D::norm()
{
    return sqrt(x*x + y*y + z*z);
}

void double3D::setZero()
{
    x=0;y=0;z=0;
}
