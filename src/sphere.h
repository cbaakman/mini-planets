#ifndef SPHERE_H
#define SPHERE_H

#include <functional>

#include "vec.h"


typedef uint64_t VertexIndex;

typedef std::function<void (const VertexIndex, const vec3 &)> SpherePointFunc;

typedef std::function<void (const VertexIndex, const VertexIndex, const VertexIndex)> SphereTriangleFunc;

/**
 * Third index represents the resulting vertex.
 */
typedef std::function<void (const VertexIndex, const VertexIndex, const VertexIndex)> SphereInterpolationFunc;

void IterIcoSphere(const int subdiv, SpherePointFunc, SphereTriangleFunc=NULL, SphereInterpolationFunc=NULL);

void IterOctaSphere(const int subdiv, SpherePointFunc, SphereTriangleFunc=NULL, SphereInterpolationFunc=NULL);

void IterDodecaSphere(const int subdiv, SpherePointFunc, SphereTriangleFunc=NULL, SphereInterpolationFunc=NULL);

#endif  // SPHERE_H
