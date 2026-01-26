#include "vectors.hpp"
#include <cmath>
#include <cfloat>

#define CMP(x, y)                      \
    (fabs((x) - (y)) <= FLT_EPSILON * \
     fmaxf(1.0f,                       \
     fmaxf(fabsf(x), fabsf(y)))        \
    )


vec2 operator+(const vec2& l, const vec2& r) 
{
    return {l.x + r.x, l.y + r.y};
}

vec3 operator+(const vec3& l, const vec3& r)
{
    return {l.x + r.x, l.y + r.y, l.z + r.z};
}

vec2 operator-(const vec2& l, const vec2& r) 
{
    return {l.x - r.x, l.y - r.y};
}

vec3 operator-(const vec3& l, const vec3& r)
{
    return {l.x - r.x, l.y - r.y, l.z - r.z};
}

vec2 operator*(const vec2& l, const vec2& r)
{
    return {l.x * r.x, l.y * r.y};
}

vec3 operator*(const vec3& l, const vec3& r)
{
    return {l.x * r.x, l.y * r.y, l.z * r.z};
}

vec2 operator*(const vec2& l, float r)
{
    return {l.x * r, l.y * r};
}

vec3 operator*(const vec3& l, float r)
{
    return {l.x * r, l.y * r, l.z * r};
}

bool operator==(const vec2& l, const vec2& r)
{
    return CMP(l.x, r.x) && CMP(l.x, r.x);
}

bool operator==(const vec3& l, const vec3& r)
{
    return CMP(l.x, r.x) && CMP(l.x, r.x) && CMP(l.z, r.z);
}

bool operator!=(const vec2& l, const vec2& r) 
{
    return !(l == r);
}

bool operator!=(const vec3& l, const vec3& r)
{
    return !(l == r);
}

float Dot(const vec2& l, const vec2& r)
{
    return l.x * r.x + l.y * r.y;
}

float Dot(const vec3& l, const vec3& r)
{
    return l.x * r.x + l.y * r.y + l.z * r.z;
}


float Magnitude(const vec2& v)
{
    return sqrtf(Dot(v, v));
}

float Magnitude(const vec3& v)
{
    return sqrtf(Dot(v, v));
}

float MagnitudeSqr(const vec2& v)
{
    return Dot(v, v);
}

float MagnitudeSqr(const vec3& v)
{
    return Dot(v, v);
}

float Distance(const vec2& p1, const vec2& p2)
{
    vec2 t = p1 - p2;
    return Magnitude(t);
}

float Distance(const vec3& p1, const vec3& p2)
{
    vec3 t = p1 - p2;
    return Magnitude(t);
}


void Normalize(vec2& v)
{
    v = v * (1.0f / Magnitude(v));
}

void Normalize(vec3& v)
{
    v = v * (1.0f / Magnitude(v));
}

vec2 Normalized(const vec2& v)
{
    return v * (1.0f / Magnitude(v));
}

vec3 Normalized(const vec3& v)
{
    return v * (1.0f / Magnitude(v));
}

vec3 Cross(const vec3& l, const vec3& r)
{
    vec3 result;
    result.x = l.y * r.z - l.z * r.y;
    result.y = l.z * r.x - l.x * r.z;
    result.z = l.x * r.y - l.y * r.x;

    return result;
}

float Angle(const vec2& l, const vec2& r)
{
    float m = sqrt(MagnitudeSqr(l) * MagnitudeSqr(r));
    return acos(Dot(l, r) / m);
}

float Angle(const vec3& l, const vec3& r)
{
    float m = sqrt(MagnitudeSqr(l) * MagnitudeSqr(r));
    return acos(Dot(l, r) / m);
}

vec2 Project(const vec2& length, const vec2& dir)
{
    float dot = Dot(length, dir);
    float magSqr  = Magnitude(dir);

    return dir * (dot / magSqr);
}

vec3 Project(const vec3& length, const vec3& dir)
{
    float dot = Dot(length, dir);
    float magSqr  = Magnitude(dir);

    return dir * (dot / magSqr);
}

vec2 Perpendicular(const vec2& length, const vec2& dir)
{
    return length - Project(length, dir);
}

vec3 Perpendicular(const vec3& length, const vec3& dir)
{
    return length - Project(length, dir);
}

vec2 Reflection(const vec2& sourceVector, const vec2& normal)
{
    float d = Dot(sourceVector, normal);
    return  sourceVector - normal * (d * 2.0f);
}

vec3 Reflection(const vec3& sourceVector, const vec3& normal)
{
    float d = Dot(sourceVector, normal);
    return  sourceVector - normal * (d * 2.0f);
}
