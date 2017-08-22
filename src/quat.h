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

#ifndef QUAT_H
#define QUAT_H

#include <math.h>
#include <cmath>
#include "vec.h"


#define QUAT_ID Quaternion(1.0f, 0.0f, 0.0f, 0.0f)

/*
    Quaternions are used to represent rotations around arbitrary axes.
    For further info, see:
    https://en.wikipedia.org/wiki/Quaternion
    https://en.wikipedia.org/wiki/Slerp
 */

struct Quaternion {

    union
    {
        struct
        {
            GLfloat w, x, y, z;
        };
        GLfloat q[4];
    };

    Quaternion(void) {}
    Quaternion(const GLfloat _w, const GLfloat _x, const GLfloat _y, const GLfloat _z)
    {
        w = _w;
        x = _x;
        y = _y;
        z = _z;
    }
    Quaternion(const Quaternion &other)
    {
        for (int i=0; i < 4; i++)
            q[i] = other.q[i];
    }

    // Length2 is computationally less expensive than Length
    GLfloat Length2(void) const
    {
        GLfloat length2 = 0.0f;
        for (int i = 0; i < 4; i++)
            length2 += q[i] * q[i];

        return length2;
    }
    GLfloat Length(void) const
    {
        return sqrt(Length2());
    }

    Quaternion operator/(const GLfloat scalar) const
    {
        Quaternion m;
        for (int i = 0; i < 4; i++)
            m.q[i] = q[i] / scalar;
        return m;
    }

    Quaternion Unit(void) const
    {
        GLfloat l = Length();
        if (l > 0)
            return (*this) / l;
        else
            return (*this);
    }

    Quaternion operator-(void) const
    {
        return Quaternion(-w, -x, -y, -z);
    }

    Quaternion Conjugate(void) const
    {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion Inverse(void) const
    {
        return Conjugate() / Length2();
    }
};

inline GLfloat Dot(const Quaternion &q1, const Quaternion &q2)
{
    GLfloat v = 0.0f;
    for (int i = 0; i < 4; i++)
        v += q1.q[i] * q2.q[i];
    return v;
}

inline Quaternion Cross(const Quaternion &q1, const Quaternion &q2)
{
    return Quaternion(q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
                      q1.y * q2.z - q1.z * q2.y + q1.x * q2.w + q1.w * q2.x,
                      q1.z * q2.x - q1.x * q2.z + q1.y * q2.w + q1.w * q2.y,
                      q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z);
}

inline Quaternion operator*(const Quaternion &q1, const Quaternion &q2)
{
    return Cross(q1, q2);
}

inline Quaternion operator+(const Quaternion &q1, const Quaternion &q2)
{
    Quaternion sum;
    for (int i = 0; i < 4; i++)
        sum.q[i] = q1.q[i] + q2.q[i];
    return sum;
}

inline Quaternion operator-(const Quaternion &q1, const Quaternion &q2)
{
    Quaternion dif;
    for (int i = 0; i < 4; i++)
        dif.q[i] = q1.q[i] - q2.q[i];
    return dif;
}

inline Quaternion operator*(const Quaternion &q, const GLfloat scalar)
{
    Quaternion m;
    for (int i = 0; i < 4; i++)
        m.q[i] = q.q[i] * scalar;
    return m;
}

inline Quaternion operator*(const GLfloat scalar, const Quaternion &q)
{
    return q * scalar;
}

/**
 * Rotates v by q.
 */
inline vec3 Rotate(const Quaternion &q, const vec3 &v)
{
    Quaternion r = (q * Quaternion(0.0f, v.x, v.y, v.z)) * q.Inverse();

    return vec3(r.x, r.y, r.z);
}

/**
 * Derives the rotation quaternion to
 * get to 'to' from 'from'.
 */
inline Quaternion Rotation(const vec3 from, const vec3 to)
{
    GLfloat dot = Dot(from.Unit(), to.Unit()),
          a, w;
    vec3 axis;

    if (dot < -0.99999999)
    {
        /*
            We can't calculate the axis if the vectors are in opposite directions.
            So take an arbitrary orthogonal axis.
            If the vectors are parallel, we won't need the axis.
         */

        if (std::abs(to.x) > std::abs(to.z))

            axis = vec3(-to.y, -to.x, 0.0f);
        else
            axis = vec3(0.0f, to.z, to.y);

        axis = axis.Unit();
        w = 0.0f;
    }
    else
    {
        a = acos(dot) / 2;
        w = cos(a);

        axis = sin(a) * Cross(from, to).Unit();
    }

    return Quaternion(w, axis.x, axis.y, axis.z);
}

inline GLfloat Angle(const Quaternion &q1, const Quaternion &q2)
{
    GLfloat dot = Dot(q1.Unit(), q2.Unit());

    if (dot > 0.99999999)
        return 0.0f;
    else if (dot < -0.99999999)
        return PI;

    return acos(dot);
}

inline Quaternion Slerp(const Quaternion &start, const Quaternion &end, GLfloat s)
{
    Quaternion start_u = start.Unit(),
               end_u = end.Unit();

    GLfloat w1, w2, theta, sTheta,

          dot = Dot(start_u, end_u);

    if (dot < 0.0f)
    {
        // assure shortest path (<= 180 degrees)
        start_u = -start_u;
        dot = Dot(start_u, end_u);
    }

    if (dot > 0.9995) // too similar
    {
        w1 = 1.0f - s;
        w2 = s;
    }
    else
    {
        theta = acos(dot),
        sTheta = sin(theta);

        w1 = sin((1.0f - s) * theta) / sTheta;
        w2 = sin(s * theta) / sTheta;
    }

    return start_u * w1 + end_u * w2;
}

#endif  // QUAT_H
