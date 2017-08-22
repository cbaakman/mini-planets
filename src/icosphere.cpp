#include <math.h>

#include "icosphere.h"


void IcoSphereTriangleSubdiv(const vec3 &p0, const vec3 &p1, const vec3 &p2,
                             const int subdiv, ISPointFunc pointFunc, ISTriangleFunc triangleFunc)
{
    if (subdiv <= 0)
    {
        if (triangleFunc)
            triangleFunc(p0, p1, p2);
    }
    else
    {
        vec3 p01 = ((p0 + p1) / 2).Unit(),
             p12 = ((p1 + p2) / 2).Unit(),
             p02 = ((p0 + p2) / 2).Unit();

        if (pointFunc)
        {
            pointFunc(p01);
            pointFunc(p12);
            pointFunc(p02);
        }

        IcoSphereTriangleSubdiv(p0, p01, p02, subdiv - 1, pointFunc, triangleFunc);
        IcoSphereTriangleSubdiv(p01, p1, p12, subdiv - 1, pointFunc, triangleFunc);
        IcoSphereTriangleSubdiv(p02, p01, p12, subdiv - 1, pointFunc, triangleFunc);
        IcoSphereTriangleSubdiv(p02, p12, p2, subdiv - 1, pointFunc, triangleFunc);
    }
}

void IterIcoSphere(const int subdiv, ISPointFunc pointFunc, ISTriangleFunc triangleFunc)
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
    int i;

    if (pointFunc)
    {
        for (i = 0; i < 12; i++)
            pointFunc(points[i]);
    }

    // 5 faces around point 0
    IcoSphereTriangleSubdiv(points[0], points[11], points[5], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[0], points[5], points[1], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[0], points[1], points[7], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[0], points[7], points[10], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[0], points[10], points[11], subdiv, pointFunc, triangleFunc);

    // 5 adjacent faces
    IcoSphereTriangleSubdiv(points[1], points[5], points[9], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[5], points[11], points[4], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[11], points[10], points[2], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[10], points[7], points[6], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[7], points[1], points[8], subdiv, pointFunc, triangleFunc);

    // 5 faces around point 3
    IcoSphereTriangleSubdiv(points[3], points[9], points[4], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[3], points[4], points[2], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[3], points[2], points[6], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[3], points[6], points[8], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[3], points[8], points[9], subdiv, pointFunc, triangleFunc);

    // 5 adjacent faces
    IcoSphereTriangleSubdiv(points[4], points[9], points[5], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[2], points[4], points[11], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[6], points[2], points[10], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[8], points[6], points[7], subdiv, pointFunc, triangleFunc);
    IcoSphereTriangleSubdiv(points[9], points[8], points[1], subdiv, pointFunc, triangleFunc);
}
