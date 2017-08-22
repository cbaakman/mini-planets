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

#ifndef MATRIX_H
#define MATRIX_H

#include "vec.h"
#include "quat.h"


union matrix4
{
    /*
       Remember: openGL uses column-major matrices.
       So the first index is the column, second is row.
     */
    GLfloat m[4][4];

    // Here the first number is row, second is column.
    struct
    {
        GLfloat m11, m21, m31, m41,
                m12, m22, m32, m42,
                m13, m23, m33, m43,
                m14, m24, m34, m44;
    };

    matrix4(void) {}

    matrix4(const GLfloat Am11, const GLfloat Am12, const GLfloat Am13, const GLfloat Am14,
            const GLfloat Am21, const GLfloat Am22, const GLfloat Am23, const GLfloat Am24,
            const GLfloat Am31, const GLfloat Am32, const GLfloat Am33, const GLfloat Am34,
            const GLfloat Am41, const GLfloat Am42, const GLfloat Am43, const GLfloat Am44)
    {
        m11 = Am11; m12 = Am12; m13 = Am13; m14 = Am14;
        m21 = Am21; m22 = Am22; m23 = Am23; m24 = Am24;
        m31 = Am31; m32 = Am32; m33 = Am33; m34 = Am34;
        m41 = Am41; m42 = Am42; m43 = Am43; m44 = Am44;
    }

    matrix4(const matrix4 &n)
    {
        int i, j;
        for (i = 0; i < 4; i++)
            for (j = 0; j < 4; j++)
                m[j][i] = n.m[j][i];
    }

    const GLfloat &El(const int row, const int column) const
    {
        return m[column][row];
    }
    GLfloat &El(const int row, const int column)
    {
        return m[column][row];
    }

    matrix4 operator*(const matrix4 &n)
    {
        matrix4 r;
        int i, j, k;
        for (i = 0; i < 4; i++)
            for (j = 0; j < 4; j++)
            {
                r.El(i, j) = 0.0f;
                for (k = 0; k < 4; k++)
                    r.El(i, j) += El(i, k) * n.El(k, j);
            }
        return r;
    }
};

inline GLfloat *operator&(const matrix4 &m)
{
    return (GLfloat *)&m.m;
}

inline vec3 operator*(const matrix4& m, const vec3 &v)
{
    vec3 r;
    int i, j;

    for (i = 0; i < 3; i++)
    {
        r.v[i] = m.El(i, 3);
        for (j = 0; j < 3; j++)
            r.v[i] += m.El(i, j) * v.v[j];
    }

    return r;
}

inline GLfloat Determinant(const matrix4 &m)
{
    GLfloat fA0 = m.m11 * m.m22 - m.m21 * m.m12,
            fA1 = m.m11 * m.m32 - m.m31 * m.m12,
            fA2 = m.m11 * m.m42 - m.m41 * m.m12,
            fA3 = m.m21 * m.m32 - m.m31 * m.m22,
            fA4 = m.m21 * m.m42 - m.m41 * m.m22,
            fA5 = m.m31 * m.m42 - m.m41 * m.m32,
            fB0 = m.m13 * m.m24 - m.m23 * m.m14,
            fB1 = m.m13 * m.m34 - m.m33 * m.m14,
            fB2 = m.m13 * m.m44 - m.m43 * m.m14,
            fB3 = m.m23 * m.m34 - m.m33 * m.m24,
            fB4 = m.m23 * m.m44 - m.m43 * m.m24,
            fB5 = m.m33 * m.m44 - m.m43 * m.m34;

    return (fA0 * fB5 - fA1 * fB4 + fA2 * fB3 +
            fA3 * fB2 - fA4 * fB1 + fA5 * fB0);
}

inline matrix4 MatID(void)
{
    matrix4 r;
    int i, j;

    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            r.El(i, j) = (i == j)? 1.0f : 0.0f;

    return r;
}
inline matrix4 MatInverse(const matrix4 &m)
{
    GLfloat fA0 = m.m11 * m.m22 - m.m21 * m.m12,
            fA1 = m.m11 * m.m32 - m.m31 * m.m12,
            fA2 = m.m11 * m.m42 - m.m41 * m.m12,
            fA3 = m.m21 * m.m32 - m.m31 * m.m22,
            fA4 = m.m21 * m.m42 - m.m41 * m.m22,
            fA5 = m.m31 * m.m42 - m.m41 * m.m32,
            fB0 = m.m13 * m.m24 - m.m23 * m.m14,
            fB1 = m.m13 * m.m34 - m.m33 * m.m14,
            fB2 = m.m13 * m.m44 - m.m43 * m.m14,
            fB3 = m.m23 * m.m34 - m.m33 * m.m24,
            fB4 = m.m23 * m.m44 - m.m43 * m.m24,
            fB5 = m.m33 * m.m44 - m.m43 * m.m34;

    GLfloat det = fA0 * fB5 - fA1 * fB4 + fA2 * fB3 +
                  fA3 * fB2 - fA4 * fB1 + fA5 * fB0;

    matrix4 kInv;
    kInv.m11 =  m.m22 * fB5 - m.m32 * fB4 + m.m42 * fB3;
    kInv.m12 = -m.m12 * fB5 + m.m32 * fB2 - m.m42 * fB1;
    kInv.m13 =  m.m12 * fB4 - m.m22 * fB2 + m.m42 * fB0;
    kInv.m14 = -m.m12 * fB3 + m.m22 * fB1 - m.m32 * fB0;
    kInv.m21 = -m.m21 * fB5 + m.m31 * fB4 - m.m41 * fB3;
    kInv.m22 =  m.m11 * fB5 - m.m31 * fB2 + m.m41 * fB1;
    kInv.m23 = -m.m11 * fB4 + m.m21 * fB2 - m.m41 * fB0;
    kInv.m24 =  m.m11 * fB3 - m.m21 * fB1 + m.m31 * fB0;
    kInv.m31 =  m.m24 * fA5 - m.m34 * fA4 + m.m44 * fA3;
    kInv.m32 = -m.m14 * fA5 + m.m34 * fA2 - m.m44 * fA1;
    kInv.m33 =  m.m14 * fA4 - m.m24 * fA2 + m.m44 * fA0;
    kInv.m34 = -m.m14 * fA3 + m.m24 * fA1 - m.m34 * fA0;
    kInv.m41 = -m.m23 * fA5 + m.m33 * fA4 - m.m43 * fA3;
    kInv.m42 =  m.m13 * fA5 - m.m33 * fA2 + m.m43 * fA1;
    kInv.m43 = -m.m13 * fA4 + m.m23 * fA2 - m.m43 * fA0;
    kInv.m44 =  m.m13 * fA3 - m.m23 * fA1 + m.m33 * fA0;

    GLfloat fInvDet = ((float)1.0) / det;
    kInv.m11 *= fInvDet;
    kInv.m21 *= fInvDet;
    kInv.m31 *= fInvDet;
    kInv.m41 *= fInvDet;
    kInv.m12 *= fInvDet;
    kInv.m22 *= fInvDet;
    kInv.m32 *= fInvDet;
    kInv.m42 *= fInvDet;
    kInv.m13 *= fInvDet;
    kInv.m23 *= fInvDet;
    kInv.m33 *= fInvDet;
    kInv.m43 *= fInvDet;
    kInv.m14 *= fInvDet;
    kInv.m24 *= fInvDet;
    kInv.m34 *= fInvDet;
    kInv.m44 *= fInvDet;

    return kInv;
}
inline matrix4 MatTranslation(const vec3& v)
{
    matrix4 m = MatID();
    int i;

    for (i = 0; i < 3; i++)
        m.El(i, 3) = v.v[i];

    return m;
}
inline matrix4 MatRotX(const GLfloat angle) // radians
{
    matrix4 m;
    const GLfloat c = cos(angle);
    const GLfloat s = sin(angle);

    m.m11 = 1.0f; m.m12 = 0.0f; m.m13 = 0.0f; m.m14 = 0.0f;
    m.m21 = 0.0f; m.m22 = c;    m.m23 = -s;   m.m24 = 0.0f;
    m.m31 = 0.0f; m.m32 = s;    m.m33 = c;    m.m34 = 0.0f;
    m.m41 = 0.0f; m.m42 = 0.0f; m.m43 = 0.0f; m.m44 = 1.0f;

    return m;
}
inline matrix4 MatRotY(const GLfloat angle) // radians
{
    matrix4 m;
    const GLfloat c = cos(angle);
    const GLfloat s = sin(angle);

    m.m11 = c;    m.m12 = 0.0f; m.m13 = s;    m.m14 = 0.0f;
    m.m21 = 0.0f; m.m22 = 1.0f; m.m23 = 0.0f; m.m24 = 0.0f;
    m.m31 = -s;   m.m32 = 0.0f; m.m33 = c;    m.m34 = 0.0f;
    m.m41 = 0.0f; m.m42 = 0.0f; m.m43 = 0.0f; m.m44 = 1.0f;

    return m;
}
inline matrix4 MatRotZ(const GLfloat angle) // radians
{
    matrix4 m;
    const GLfloat c = cos(angle);
    const GLfloat s = sin(angle);

    m.m11 = c;    m.m12 = -s;   m.m13 = 0.0f; m.m14 = 0.0f;
    m.m21 = s;    m.m22 = c;    m.m23 = 0.0f; m.m24 = 0.0f;
    m.m31 = 0.0f; m.m32 = 0.0f; m.m33 = 1.0f; m.m34 = 0.0f;
    m.m41 = 0.0f; m.m42 = 0.0f; m.m43 = 0.0f; m.m44 = 1.0f;

    return m;
}
inline matrix4 MatQuat(const Quaternion &q) // also a rotation matrix
{
    GLfloat f = 2.0f / q.Length2();

    matrix4 m;
    GLfloat y2 = q.y * q.y, z2 = q.z * q.z, x2 = q.x * q.x,
            xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z,
            xw = q.x * q.w, yw = q.y * q.w, zw = q.z * q.w;

    m.m11 = 1.0f - f * (y2 + z2); m.m12 =        f * (xy - zw); m.m13 =        f * (xz + yw); m.m14 = 0.0f;
    m.m21 =        f * (xy + zw); m.m22 = 1.0f - f * (x2 + z2); m.m23 =        f * (yz - xw); m.m24 = 0.0f;
    m.m31 =        f * (xz - yw); m.m32 =        f * (yz + xw); m.m33 = 1.0f - f * (x2 + y2); m.m34 = 0.0f;
    m.m41 = 0.0f;                 m.m42 = 0.0f;                 m.m43 = 0.0f;                 m.m44 = 1.0f;

    return m;
}
inline matrix4 MatRotAxis(const vec3& axis, float angle) // radians
{
    matrix4 m;
    vec3 a = axis.Unit();
    const GLfloat c = cos(angle),
                  ivc = 1.0f - c,
                  s = sin(angle),
                  y2 = a.y * a.y,
                  x2 = a.x * a.x,
                  z2 = a.z * a.z,
                  xy = a.x * a.y,
                  xz = a.x * a.z,
                  yz = a.y * a.z;

    m.m11 = ivc * x2 + c;       m.m12 = ivc * xy - s * a.z; m.m13 = ivc * xz + s * a.y; m.m14 = 0.0f;
    m.m21 = ivc * xy + s * a.z; m.m22 = ivc * y2 + c;       m.m23 = ivc * yz - s * a.x; m.m24 = 0.0f;
    m.m31 = ivc * xz - s * a.y; m.m32 = ivc * yz + s * a.x; m.m33 = ivc * z2 + c;       m.m34 = 0.0f;
    m.m41 = 0.0f;               m.m42 = 0.0f;               m.m43 = 0.0f;               m.m44 = 1.0f;

    return m;
}

inline matrix4 MatFirstPerson(const vec3 &pos, const GLfloat yaw, const GLfloat pitch, const GLfloat roll)
{
    return MatInverse(MatTranslation(pos) * MatRotZ(roll) * MatRotY(yaw) * MatRotX(pitch));
}

/**
 * Works in a right-handed coordinate system.
 */
inline matrix4 MatLookAt(const vec3 &eye, const vec3 &lookat, const vec3 &up)
{
    vec3 _right,
         _up,
         _back;
    matrix4 m;

    _back = (eye - lookat).Unit();
    _right = Cross(up, _back).Unit();
    _up = Cross(_back, _right).Unit();

    m.m11 = _right.x;  m.m12 = _right.y;  m.m13 = _right.z;  m.m14 = -Dot(_right, eye);
    m.m21 = _up.x;     m.m22 = _up.y;     m.m23 = _up.z;     m.m24 = -Dot(_up, eye);
    m.m31 = _back.x;   m.m32 = _back.y;   m.m33 = _back.z;   m.m34 = -Dot(_back, eye);
    m.m41 = 0.0f;      m.m42 = 0.0f;      m.m43 = 0.0f;      m.m44 = 1.0f;

    return m;
}

/**
 * Perspective projection matrix. Z-axis points out of the screen.
 * It's right-handed.
 */
inline matrix4 MatPerspec(GLfloat view_angle, GLfloat aspect_ratio,
                          GLfloat near_viewdist, GLfloat far_viewdist)
{
    GLfloat a = 0.5f * view_angle;
    GLfloat f = 1.0f / tan(a);
    GLfloat zdiff = near_viewdist - far_viewdist;

    matrix4 m = MatID();

    m.m11 = f / aspect_ratio;
    m.m22 = f;
    m.m33 = (far_viewdist + near_viewdist) / zdiff;
    m.m34 = 2 * far_viewdist * near_viewdist / zdiff;
    m.m43 = -1.0f;
    m.m44 = 0.0f;

    return m;
}

/**
 * Orthographic projection matrix. Arguments define screen boundaries.
 */
inline matrix4 MatOrtho(GLfloat leftX, GLfloat rightX, GLfloat upY, GLfloat downY, GLfloat nearZ, GLfloat farZ)
{
    GLfloat Xdiff = rightX - leftX,
            Ydiff = upY - downY,
            Zdiff = nearZ - farZ;

    matrix4 m = MatID();

    m.m11 = 2.0f / Xdiff;
    m.m22 = 2.0f / Ydiff;
    m.m33 = 2.0f / Zdiff;
    m.m14 = -(rightX + leftX) / Xdiff;
    m.m24 = -(upY + downY) / Ydiff;
    m.m34 = -(farZ + nearZ) / Zdiff;

    return m;
}


#endif  // MATRIX_H
