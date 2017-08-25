#include <math.h>
#include <map>

#include "sphere.h"
#include "matrix.h"
#include "log.h"


struct CompareVec3
{
    bool operator()(const vec3 &v1, const vec3 &v2) const
    {
        if (v1.x == v2.x)
            if (v1.y == v2.y)
                return v1.z < v2.z;
            else
                return v1.y < v2.y;
        else
            return v1.x < v2.x;
    }
};

typedef std::map<vec3, VertexIndex, CompareVec3> VertexIndexMap;

void SphereTriangleSubdiv(VertexIndexMap &indices,
                          const vec3 &p0, const vec3 &p1, const vec3 &p2,
                          const int subdiv, SpherePointFunc pointFunc,
                                            SphereTriangleFunc triangleFunc,
                                            SphereInterpolationFunc interpolationFunc)
{
    if (subdiv <= 0)
    {
        if (triangleFunc)
            triangleFunc(indices.at(p0), indices.at(p1), indices.at(p2));
    }
    else
    {
        vec3 p01 = ((p0 + p1) / 2).Unit(),
             p12 = ((p1 + p2) / 2).Unit(),
             p02 = ((p0 + p2) / 2).Unit();

        VertexIndex i;

        // Always call the point func before the triangle func, because the indexes must be valid.

        if (indices.find(p01) == indices.end())
        {
            i = indices.size();
            indices[p01] = i;

            if (pointFunc)
                pointFunc(i, p01);

            if (interpolationFunc)
                interpolationFunc(indices.at(p0), indices.at(p1), i);
        }

        if (indices.find(p12) == indices.end())
        {
            i = indices.size();
            indices[p12] = i;

            if (pointFunc)
                pointFunc(i, p12);

            if (interpolationFunc)
                interpolationFunc(indices.at(p1), indices.at(p2), i);
        }

        if (indices.find(p02) == indices.end())
        {
            i = indices.size();
            indices[p02] = i;

            if (pointFunc)
                pointFunc(i, p02);

            if (interpolationFunc)
                interpolationFunc(indices.at(p0), indices.at(p2), i);
        }

        SphereTriangleSubdiv(indices, p0, p01, p02, subdiv - 1, pointFunc, triangleFunc, interpolationFunc);
        SphereTriangleSubdiv(indices, p01, p1, p12, subdiv - 1, pointFunc, triangleFunc, interpolationFunc);
        SphereTriangleSubdiv(indices, p02, p01, p12, subdiv - 1, pointFunc, triangleFunc, interpolationFunc);
        SphereTriangleSubdiv(indices, p02, p12, p2, subdiv - 1, pointFunc, triangleFunc, interpolationFunc);
    }
}

void IterIcoSphere(const int subdiv, SpherePointFunc pointFunc,
                                     SphereTriangleFunc triangleFunc,
                                     SphereInterpolationFunc interpolationFunc)
{
    const float t = (1.0f + sqrt(5.0f)) / 2;

    const vec3 points[12] = {vec3(-1.0f, t, 0.0f).Unit(),
                             vec3(1.0f, t, 0.0f).Unit(),
                             vec3(-1.0f, -t, 0.0f).Unit(),
                             vec3(1.0f, -t, 0.0f).Unit(),
                             vec3(0.0f, -1.0f, t).Unit(),
                             vec3(0.0f, 1.0f, t).Unit(),
                             vec3(0.0f, -1.0f, -t).Unit(),
                             vec3(0.0f, 1.0f, -t).Unit(),
                             vec3(t, 0.0f, -1.0f).Unit(),
                             vec3(t, 0.0f, 1.0f).Unit(),
                             vec3(-t, 0.0f, -1.0f).Unit(),
                             vec3(-t, 0.0f, 1.0f).Unit()};

    VertexIndexMap indices;
    VertexIndex i;

    for (i = 0; i < 12; i++)
    {
        indices[points[i]] = i;

        // Always call the point func before the triangle func, because the indexes must be valid.
        if (pointFunc)
            pointFunc(i, points[i]);
    }

    // 5 faces around point 0
    SphereTriangleSubdiv(indices, points[0], points[11], points[5], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[0], points[5], points[1], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[0], points[1], points[7], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[0], points[7], points[10], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[0], points[10], points[11], subdiv, pointFunc, triangleFunc, interpolationFunc);

    // 5 adjacent faces
    SphereTriangleSubdiv(indices, points[1], points[5], points[9], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[5], points[11], points[4], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[11], points[10], points[2], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[10], points[7], points[6], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[7], points[1], points[8], subdiv, pointFunc, triangleFunc, interpolationFunc);

    // 5 faces around point 3
    SphereTriangleSubdiv(indices, points[3], points[9], points[4], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[3], points[4], points[2], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[3], points[2], points[6], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[3], points[6], points[8], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[3], points[8], points[9], subdiv, pointFunc, triangleFunc, interpolationFunc);

    // 5 adjacent faces
    SphereTriangleSubdiv(indices, points[4], points[9], points[5], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[2], points[4], points[11], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[6], points[2], points[10], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[8], points[6], points[7], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[9], points[8], points[1], subdiv, pointFunc, triangleFunc, interpolationFunc);
}

void IterOctaSphere(const int subdiv, SpherePointFunc pointFunc,
                                      SphereTriangleFunc triangleFunc,
                                      SphereInterpolationFunc interpolationFunc)
{
    const vec3 points[6] = {vec3(0.0f, 1.0f, 0.0f),
                            vec3(0.0f, -1.0f, 0.0f),
                            vec3(1.0f, 0.0f, 0.0f),
                            vec3(-1.0f, 0.0f, 0.0f),
                            vec3(0.0f, 0.0f, 1.0f),
                            vec3(0.0f, 0.0f, -1.0f)};

    VertexIndexMap indices;
    VertexIndex i;


    for (i = 0; i < 6; i++)
    {
        indices[points[i]] = i;

        // Always call the point func before the triangle func, because the indexes must be valid.
        if (pointFunc)
            pointFunc(i, points[i]);
    }

    // 4 faces from top to equator
    SphereTriangleSubdiv(indices, points[0], points[3], points[4], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[0], points[4], points[2], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[0], points[2], points[5], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[0], points[5], points[3], subdiv, pointFunc, triangleFunc, interpolationFunc);

    // 4 faces from bottom to equator
    SphereTriangleSubdiv(indices, points[1], points[4], points[3], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[1], points[2], points[4], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[1], points[5], points[2], subdiv, pointFunc, triangleFunc, interpolationFunc);
    SphereTriangleSubdiv(indices, points[1], points[3], points[5], subdiv, pointFunc, triangleFunc, interpolationFunc);
}

/*
    vertex 0 sits next to vertex 4 and vertex 1
    vertex 1 sits next to vertex 0 and vertex 2
    etc.
 */
typedef VertexIndex IndexedPentagon[5];

void IterDodecaSphere(const int subdiv, SpherePointFunc pointFunc,
                                        SphereTriangleFunc triangleFunc,
                                        SphereInterpolationFunc interpolationFunc)
{
    vec3 points[32],
         centers[12];  // centers of the pentagon faces
    IndexedPentagon unorderedFaces[12],
                    orderedFaces[12];
    int pointsPerFace[12], k, l, l2;

    VertexIndex j = 0;
    for (k = 0; k < 12; k++)
        pointsPerFace[k] = 0;

    IterIcoSphere(0,
        [&](const VertexIndex i, const vec3 &p)
        {
            centers[i] = p.Unit();

            // The centers of the pentagons are needed to make triangles.
            points[j] = centers[i];
            j ++;
        },
        [&](const VertexIndex i0, const VertexIndex i1, const VertexIndex i2)
        {
            points[j] = (centers[i0] + centers[i1] + centers[i2]).Unit();

            unorderedFaces[i0][pointsPerFace[i0]] = j;
            pointsPerFace[i0] ++;

            unorderedFaces[i1][pointsPerFace[i1]] = j;
            pointsPerFace[i1] ++;

            unorderedFaces[i2][pointsPerFace[i2]] = j;
            pointsPerFace[i2] ++;

            j ++;
        }
    );

    // order pentagon vertices:
    for (k = 0; k < 12; k++)
    {
        vec3 pproj, xproj, yproj;
        GLfloat a,
                aDir = -1.0f;  // 1.0f for other direction
        int index,
            lstart = 0;
        vec2 vstart(0.0f, 1.0f);

        Plane pentagonPlane = {centers[k].Unit(), 0.0f};

        yproj = PlaneProjection(points[unorderedFaces[k][lstart]], pentagonPlane).Unit();
        xproj = MatRotAxis(centers[k], PI / 2) * yproj;
        for (l = 0; l < 5; l++)
        {
            pproj = PlaneProjection(points[unorderedFaces[k][l]], pentagonPlane).Unit();

            a = Angle(pproj, xproj);
            if (Dot(pproj, yproj) < 0)
                a = -a;

            // convert angle(-PI to PI) to an index(0, 1, 2, 3, 4)
            index = (int)floor(5 * (a + PI) / (2 * PI));

            orderedFaces[k][index] = unorderedFaces[k][l];
        }
    }

    VertexIndexMap indices;

    // Iterate vertices:
    for (k = 0; k < 32; k++)
    {
        indices[points[k]] = k;

        if (pointFunc)
            pointFunc(k, points[k]);
    }

    // Iterate faces:
    for (k = 0; k < 12; k++)
    {
        for (l = 0; l < 5; l++)
        {
            l2 = (l + 1) % 5;

            VertexIndex ic = (VertexIndex)k,
                        i1 = orderedFaces[k][l],
                        i2 = orderedFaces[k][l2];

            SphereTriangleSubdiv(indices,
                                 points[ic], points[i2], points[i1],
                                 subdiv, pointFunc, triangleFunc, interpolationFunc);
        }
    }
}
