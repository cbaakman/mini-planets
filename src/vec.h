/* Copyright (C) 2017 Coos Baakman
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.
  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:
  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef VEC_H
#define VEC_H

#include <memory>
#include <math.h>
#define PI 3.1415926535897932384626433832795

#include <GL/glew.h>
#include <GL/gl.h>

#include "log.h"

template <int N>
struct vec
{
    GLfloat v[N];

    GLfloat &operator[](const int i)
    {
        return v[i];
    }
    const GLfloat &operator[](const int i) const
    {
        return v[i];
    }
    GLfloat Length2(void) const
    {
        GLfloat l2 = 0.0f;
        for (int i = 0; i < N; i++)
            l2 += v[i] * v[i];
        return l2;
    }
    GLfloat Length(void) const
    {
        return sqrt(Length2());
    }
    GLfloat Unit(void) const
    {
        GLfloat l = Length();
        if (l <= 0.0f)
            return *this;

        vec<N> u;
        for (int i = 0; i < N; i++)
            u.v[i] = v[i] / l;
        return u;
    }
};

template <int N>
inline GLfloat *operator&(const vec<N> &v)
{
    return (GLfloat *)&v.v;
}


/*
    vec3 is a vector of length 3, used for 3D space calculations.
 */

template <>
struct vec<3>
{
    // Coordinates: x, y, and z, or v[0], v[1] and v[2]
    union
    {
        struct
        {
            GLfloat x, y, z;
        };
        GLfloat v[3];
    };

    vec(void) {}
    vec(GLfloat _x, GLfloat _y, GLfloat _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    vec(const vec<3> &v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
    }

    GLfloat &operator[](const int i)
    {
        return v[i];
    }
    const GLfloat &operator[](const int i) const
    {
        return v[i];
    }
    /*
        Length2 is the squared length, it's
        computationally less expensive than Length
     */
    GLfloat Length2(void) const
    {
        return (x * x + y * y + z * z);
    }
    GLfloat Length(void) const
    {
        return sqrt(Length2());
    }

    vec Unit(void) const
    {
        // The unit vector has length 1.0

        GLfloat l = Length();
        if (l > 0)
            return vec<3> (x / l, y / l, z / l);
        else
            return *this;
    }

    /*
        vec3 objects can be added on, subtracted and
        multiplied/divided by a scalar.
     */
};
typedef vec<3> vec3;

template <>
struct vec<2>
{
    union
    {
        struct
        {
            GLfloat x;
            union
            {
                GLfloat y;
                GLfloat z;
            };
        };
        GLfloat v[2];
    };
    vec(void) {}
    vec(GLfloat _x, GLfloat _y)
    {
        x = _x;
        y = _y;
    }

    GLfloat &operator[] (const int i)
    {
        return v[i];
    }
    const GLfloat &operator[] (const int i) const
    {
        return v[i];
    }

    GLfloat Length2(void) const
    {
        return(x * x + y * y);
    }
    GLfloat Length(void) const
    {
        return sqrt(Length2());
    }

    vec Unit(void) const
    {
        GLfloat l = Length();
        if (l > 0)
            return vec<2> (x / l, y / l);
        else
            return *this;
    }

    GLfloat Angle(void) const // with x-axis
    {
        return atan2(y, x);
    }

    /**
     * Rotates this vector in counter clockwise direction.
     * :param angle: angle in radians
     */
    vec Rotate(const GLfloat angle) const
    {
        vec r;
        const GLfloat c = cos(angle),
                      s = sin(angle);

        r.x = c * x - s * y;
        r.y = s * x + c * y;
        return r;
    }
};
typedef vec<2> vec2;

template <int N>
GLfloat Distance2(const vec<N> v1, const vec<N> v2)
{
    return (v1 -  v2).Length2();
}
template <int N>
GLfloat Distance(const vec<N> v1, const vec<N> v2)
{
    return (v1 -  v2).Length();
}

template <int N>
inline vec<N> operator-(const vec<N> &v)
{
    vec<N> r;
    for (int i = 0; i < N; i++)
        r[i] = -v[i];
    return r;
}
template <int N>
inline vec<N> operator-(const vec<N> &v1, const vec<N> &v2)
{
    vec<N> r;
    for (int i = 0; i < N; i++)
        r[i] = v1[i] - v2[i];
    return r;
}
template <int N>
inline vec<N> operator+(const vec<N> &v1, const vec<N> &v2)
{
    vec<N> r;
    for (int i = 0; i < N; i++)
        r[i] = v1[i] + v2[i];
    return r;
}
template <int N>
inline vec<N> &operator-=(vec<N> &v1, const vec<N> &v2)
{
    for (int i = 0; i < N; i++)
        v1[i] -= v2[i];
    return v1;
}
template <int N>
inline vec<N> &operator+=(vec<N> &v1, const vec<N> &v2)
{
    for (int i = 0; i < N; i++)
        v1[i] += v2[i];
    return v1;
}
template <int N>
inline bool operator==(const vec<N> &v1, const vec<N> &v2)
{
    for (int i = 0; i < N; i++)
        if (v1[i] != v2[i])
            return false;
    return true;
}
template <int N>
inline bool operator!=(const vec<N> &v1, const vec<N> &v2)
{
    for (int i = 0; i < N; i++)
        if (v1[i] != v2[i])
            return true;
    return false;
}
template <int N, typename Number>
inline vec<N> operator/(const vec<N> &v, const Number scalar)
{
    vec<N> r;
    for (int i = 0; i < N; i++)
        r[i] = GLfloat(v[i] / scalar);
    return r;
}
template <int N, typename Number>
inline vec<N> operator*(const vec<N> &v, const Number scalar)
{
    vec<N> r;
    for (int i = 0; i < N; i++)
        r[i] = GLfloat(v[i] * scalar);
    return r;
}
template <int N, typename Number>
inline vec<N> &operator*=(vec<N> &v, const Number scalar)
{
    for (int i = 0; i < N; i++)
        v[i] *= scalar;
    return v;
}
template <int N, typename Number>
inline vec<N> &operator/=(vec<N> &v, const Number scalar)
{
    for (int i = 0; i < N; i++)
        v[i] /= scalar;
    return v;
}

template <int N, typename Number>
inline vec<N> operator*(const Number s, const vec<N> &v)
{
    return v * s;
}
inline vec3 Cross(const vec3 &v1, const vec3 &v2)
{
    return vec3(
        v1.y * v2.z - v2.y * v1.z,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x
        );
}
template <int N>
inline GLfloat Dot(const vec<N> &v1,const vec<N> &v2)
{
    GLfloat dot = 0.0;
    for (int i = 0; i < N; i++)
        dot += v1[i] * v2[i];
    return dot;
}
template <int N>
inline GLfloat Angle(const vec<N> &v1, const vec<N> &v2)
{
    GLfloat dot = Dot(v1.Unit(), v2.Unit());

    if (dot > 0.99999999)
        return 0.0f;
    else if (dot < -0.99999999)
        return PI;

    GLfloat a = acos(dot);

    // Return an angle between -PI and PI.
    while (a > PI)
        a -= 2 * PI;
    while (a <= -PI)
        a += 2 * PI;

    return a;
} // In radians
template <int N>
inline vec<N> Projection(const vec<N> &v, const vec<N> &on_v)
{
    return on_v * Dot(v, on_v) / on_v.Length2();
}
template <int N>
inline vec<N> ClosestPointOnLine(const vec<N> &l1, const vec<N> &l2,
                                 const vec<N> &point)
{
    vec<N> l1p = point - l1;
    vec<N> l12 = l2 - l1;
    GLfloat d = l12.Length();
    l12 = l12 / d;
    GLfloat t = Dot(l12, l1p);
    if (t <= 0)
        return l1;
    if (t >= d)
        return l2;
    return l1 + t * l12;
}

struct Plane {

    vec3 n; // plane normal, should have length 1.0
    GLfloat d; // shortest distance from(0,0,0) to plane

    // n * d is a point on the plane
};

inline Plane Flip(const Plane  &plane)
{
    Plane r;
    r.n = -plane.n;
    r.d = -plane.d;

    return r;
}

struct Triangle {

    vec3 p[3];

    Triangle() {}

    Triangle(const vec3 &p0, const vec3 &p1, const vec3 &p2)
    {
        p[0] = p0;
        p[1] = p1;
        p[2] = p2;
    }

    vec3 Center() const
    {
        vec3 pb = 0.5f * (p[1] - p[0]),
             pz = pb + 0.333333f * (p[2] - pb);

        return pz;
    }

    Plane GetPlane() const
    {
        Plane plane;
        plane.n = Cross(p[1] - p[0], p[2] - p[0]).Unit();
        plane.d = -Dot(p[0], plane.n);
        return plane;
    }
};


inline bool SameSide(const vec3 &p1, const vec3 &p2, const vec3 &a, const vec3 &b)
{
    vec3 cp1 = Cross(b - a, p1 - a),
         cp2 = Cross(b - a, p2 - a);

    // make it -0.00001f instead of 0.0f to cover up the error in floating point
    return (Dot(cp1, cp2) >= -0.00001f);
}

inline bool PointInsideTriangle(const Triangle &t, const vec3 &p)
{
    return (SameSide(p, t.p[0], t.p[1], t.p[2]) &&
            SameSide(p ,t.p[1], t.p[0], t.p[2]) &&
            SameSide(p, t.p[2], t.p[0], t.p[1]));
}
inline GLfloat DistanceFromPlane(const vec3& p, const Plane &plane)
{
    return Dot(plane.n, p) + plane.d;
}
inline vec3 PlaneProjection(const vec3& p, const Plane &plane)
{
    return p - DistanceFromPlane(p, plane) * plane.n;
}
inline vec2 LineIntersection(const vec2& a1, const vec2& a2,
                             const vec2& b1, const vec2& b2)
{
    GLfloat d = (a1.x - a2.x) * (b1.y - b2.y) - (a1.y - a2.y) * (b1.x - b2.x),
            a = a1.x * a2.y - a1.y * a2.x,
            b = b1.x * b2.y - b1.y * b2.x;

    vec2 r((a * (b1.x - b2.x) - b * (a1.x - a2.x)) / d,
           (a * (b1.y - b2.y) - b * (a1.y - a2.y)) / d);
    return r;
}

#endif  // VEC_H
