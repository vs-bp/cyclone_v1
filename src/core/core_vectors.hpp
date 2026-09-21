#pragma once
#include <Arduino.h>

/* -------------------------------------------------------------------------- */
/*                                 Vector Math                                */
/* -------------------------------------------------------------------------- */
struct vec4 { 
  // Definition.
  float w, x, y, z; 
  
  // Constructor.
  vec4() { w = 0; x = 0; y = 0; z = 0; }
  vec4(float s) { x = s; y = s; z = s; w = s; }
  vec4(float _w, float _x, float _y, float _z) { w = _w; x = _x; y = _y; z = _z; }
  
  // Operator overloads.
  vec4 operator+(const vec4 rhs)  const { return vec4(w+rhs.w, x+rhs.x, y+rhs.y, z+rhs.z); }
  vec4 operator-(const vec4 rhs)  const { return vec4(w-rhs.w, x-rhs.x, y-rhs.y, z-rhs.z); }
  vec4 operator*(const vec4 rhs)  const { return vec4(w*rhs.w, x*rhs.x, y*rhs.y, z*rhs.z); }
  vec4 operator/(const vec4 rhs)  const { return vec4(w/rhs.w, x/rhs.x, y/rhs.y, z/rhs.z); }
  vec4 operator*(const float rhs) const { return vec4(w*rhs, x*rhs, y*rhs, z*rhs); }
  vec4 operator/(const float rhs) const { return vec4(w/rhs, x/rhs, y/rhs, z/rhs); }
  vec4 operator-()                const { return vec4(-w, -x, -y, -z); }

  // Helper functions.
  float magnitude()     const { return sqrt(w*w + x*x + y*y + z*z); }
  vec4 normalize() const { return *this / this->magnitude(); }
  vec4 elabs()     const { return vec4(fabs(w), fabs(x), fabs(y), fabs(z)); }
  float elmax()    const { return fmax(fmax(w,x),fmax(y,z)); }
  float elmin()    const { return fmin(fmin(w,x),fmin(y,z)); }
};
struct vec3 { 
  // Definition.
  float x, y, z;
  
  // Constructor.
  vec3() { x = 0; y = 0; z = 0; }
  vec3(float s) { x = s; y = s; z = s; }
  vec3(float _x, float _y, float _z) { x = _x; y = _y; z = _z; }
  
  // Operator overloads.
  vec3 operator+(const vec3 rhs)  const { return vec3(x+rhs.x, y+rhs.y, z+rhs.z); }
  vec3 operator-(const vec3 rhs)  const { return vec3(x-rhs.x, y-rhs.y, z-rhs.z); }
  vec3 operator*(const vec3 rhs)  const { return vec3(x*rhs.x, y*rhs.y, z*rhs.z); }
  vec3 operator/(const vec3 rhs)  const { return vec3(x/rhs.x, y/rhs.y, z/rhs.z); }
  vec3 operator*(const float rhs) const { return vec3(x*rhs, y*rhs, z*rhs); }
  vec3 operator/(const float rhs) const { return vec3(x/rhs, y/rhs, z/rhs); }
  vec3 operator-()                const { return vec3(-x, -y, -z); }

  // Helper functions.
  float magnitude()     const { return sqrt(x*x + y*y + z*z); }
  vec3 normalize() const { return *this / this->magnitude(); }
  vec3 elabs()     const { return vec3(fabs(x), fabs(y), fabs(z)); }
  float elmax()    const { return fmax(fmax(x,y),z); }
  float elmin()    const { return fmin(fmin(x,y),z); }
};
struct vec2 { 
  // Definition.
  float x, y;
  
  // Constructor.
  vec2() { x = 0; y = 0; }
  vec2(float s) { x = s; y = s; }
  vec2(float _x, float _y) { x = _x; y = _y; }
  
  // Operator overloads.
  vec2 operator+(const vec2 rhs) const { return vec2(x+rhs.x, y+rhs.y); }
  vec2 operator-(const vec2 rhs) const { return vec2(x-rhs.x, y-rhs.y); }
  vec2 operator*(const vec2 rhs) const { return vec2(x*rhs.x, y*rhs.y); }
  vec2 operator/(const vec2 rhs) const { return vec2(x/rhs.x, y/rhs.y); }
  vec2 operator*(const float rhs) const{ return vec2(x*rhs, y*rhs); }
  vec2 operator/(const float rhs) const{ return vec2(x/rhs, y/rhs); }
  vec2 operator-() const { return vec2(-x, -y); }

  // Helper functions.
  float magnitude()     const { return sqrt(x*x + y*y); }
  vec2 normalize() const { return *this / this->magnitude(); }
  vec2 elabs()     const { return vec2(fabs(x), fabs(y)); }
  float elmax()    const { return fmax(x,y); }
  float elmin()    const { return fmin(x,y); }
};
struct mat2 {
  // Definition.
  vec2 r[2];
  
  // Constructor.
  mat2() { r[0] = vec2(0.0); r[1] = vec2(0.0); }
  mat2(const mat2& other) { r[0] = other.r[0]; r[1] = other.r[1]; }
  mat2(const float s) { r[0].x = s; r[0].y = 0.0; r[1].x = 0.0; r[1].y = s; }
  mat2(const vec2 r0, const vec2 r1) { r[0] = r0; r[1] = r1; }
  
  // Operator overloads.
  mat2 operator+(const mat2 rhs) const { return mat2(r[0] + rhs.r[0], r[1] + rhs.r[1]); }
  mat2 operator-(const mat2 rhs) const { return mat2(r[0] - rhs.r[0], r[1] - rhs.r[1]); }
  mat2 operator*(const float rhs) const { return mat2(r[0] * rhs, r[1] * rhs); }
  mat2 operator/(const float rhs) const { return mat2(r[0] / rhs, r[1] / rhs); }
  vec2 operator*(const vec2 rhs) const { return (vec2(r[0].x, r[1].x) * rhs.x) + (vec2(r[0].y, r[1].y) * rhs.y); }
  mat2 operator*(const mat2 rhs) const { return mat2(
    vec2(r[0].x*rhs.r[0].x + r[0].y*rhs.r[1].x, r[0].x*rhs.r[0].y + r[0].y*rhs.r[1].y),
    vec2(r[1].x*rhs.r[0].x + r[1].y*rhs.r[1].x, r[1].x*rhs.r[0].y + r[1].y*rhs.r[1].y)
  ); }

  // Helper functions.
  mat2 transpose() const { return mat2(vec2(r[0].x, r[1].x), vec2(r[0].y, r[1].y)); }
  mat2 inverse() const { return mat2(vec2(r[1].y, -r[0].y), vec2(-r[1].x, r[0].x))*(1.0/(r[0].x*r[1].y-r[0].y*r[1].x)); }
};
struct mat3 {
  // Definition.
  vec3 r[3];
  
  // Constructor.
  mat3() { r[0] = vec3(0.0); r[1] = vec3(0.0); r[2] = vec3(0.0); }
  mat3(const mat3& other) { r[0] = other.r[0]; r[1] = other.r[1]; r[2] = other.r[2]; }
  mat3(const float s) { r[0].x = s; r[0].y = 0.0; r[1].x = 0.0; r[1].y = s; r[2].z = s; }
  mat3(const vec3 r0, const vec3 r1, const vec3 r2) { r[0] = r0; r[1] = r1; r[2] = r2; }
  
  // Operator overloads.
  mat3 operator+(const mat3 rhs) const { return mat3(r[0] + rhs.r[0], r[1] + rhs.r[1], r[2] + rhs.r[2]); }
  mat3 operator-(const mat3 rhs) const { return mat3(r[0] - rhs.r[0], r[1] - rhs.r[1], r[2] - rhs.r[2]); }
  mat3 operator*(const float rhs) const { return mat3(r[0] * rhs, r[1] * rhs, r[2] * rhs); }
  mat3 operator/(const float rhs) const { return mat3(r[0] / rhs, r[1] / rhs, r[2] / rhs); }
  vec3 operator*(const vec3 rhs) const { return vec3(
    r[0].x*rhs.x + r[0].y*rhs.y + r[0].z*rhs.z,
    r[1].x*rhs.x + r[1].y*rhs.y + r[1].z*rhs.z,
    r[2].x*rhs.x + r[2].y*rhs.y + r[2].z*rhs.z
  ); }
  // TODO Unimplemented.
  // mat3 operator*(const mat3 rhs) const { return mat3(
  //   vec2(r[0].x*rhs.r[0].x + r[0].y*rhs.r[1].x, r[0].x*rhs.r[0].y + r[0].y*rhs.r[1].y),
  //   vec2(r[1].x*rhs.r[0].x + r[1].y*rhs.r[1].x, r[1].x*rhs.r[0].y + r[1].y*rhs.r[1].y)
  // ); }

  // Helper functions.
  // TODO Unimplemented.
  mat3 transpose() const { return mat3(
    vec3(r[0].x, r[1].x, r[2].x),
    vec3(r[0].y, r[1].y, r[2].y),
    vec3(r[0].z, r[1].z, r[2].z)
  ); }
  // mat2 inverse() const { return mat2(vec2(r[1].y, -r[0].y), vec2(-r[1].x, r[0].x))*(1.0/(r[0].x*r[1].y-r[0].y*r[1].x)); }
};

// -====================- Filtering -====================-
// 2D Liner model kalman filter.
struct lkf2 {
  const mat2 I = mat2(vec2(1.0, 0.0), vec2(0.0, 1.0));

  vec2 x1;
  mat2 p1;

  mat2 q;
  mat2 r;

  lkf2(const mat2 _q, const mat2 _r, const vec2 _x1) {
    x1 = _x1;
    p1 = I;
    q = _q;
    r = _r;
  }
  void update(mat2 A, vec2 B, vec2 z1) {
    vec2 x1m = A*x1 + B;
    mat2 p1m = A*A*p1 + q;
    mat2 k1 = p1m * (p1 + r).inverse();
    x1 = (I-k1)*x1m + k1*z1;
    p1 = (I-k1)*p1m;
  }
};