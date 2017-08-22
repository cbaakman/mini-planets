#ifndef ICOSPHERE_H
#define ICOSPHERE_H

#include <functional>

#include "vec.h"


typedef std::function<void (const vec3 &)> ISPointFunc;
typedef std::function<void (const vec3 &, const vec3 &, const vec3 &)> ISTriangleFunc;

void IterIcoSphere(const int subdiv, ISPointFunc=NULL, ISTriangleFunc=NULL);

#endif  // ICOSPHERE_H
