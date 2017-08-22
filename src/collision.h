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


#ifndef COLLISION_H
#define COLLISION_H

#include "vec.h"

#include <list>
#include <tuple>


/**
 *  The basic moving collision object.
 *  When it hits a triangle, it must return:
 *  1 The point on the triangle that it hit
 *  2 The point on the object that hit the triangle (relative to its start position)
 *  3 The normal of the plane it pushes against (especially when it hits an edge)
 */
class Collider {

public:
    virtual bool HitsTriangle(const Triangle &tri, const vec3 &startPoint, const vec3 &movement,
                              vec3 &closestPointOnColliderToWall, vec3 &wallContactPoint, vec3 &wallNormal) = 0;
};

/**
 *  Collides like a sphere
 */
class SphereCollider : public Collider {
private:
    vec3 relativeCenter;
    float radius;
public:
    SphereCollider(const vec3 &pivotToCenter , const float radius);

    bool HitsTriangle(const Triangle &tri, const vec3 &startPoint, const vec3 &movement,
                      vec3 &closestPointOnColliderToWall, vec3 &wallContactPoint, vec3 &wallNormal);
};

/**
 * Like an arrow pointing towards the ground. (or walls) It's not a stick collider !!
 * Kinda buggy when touching the edge between two triangles.
 */
class FeetCollider : public Collider {
private:
    vec3 toFeet;
public:
    FeetCollider(const vec3 pivotToFeet);

    bool HitsTriangle(const Triangle &tri, const vec3 &startPoint, const vec3 &movement,
                      vec3 &closestPointOnColliderToWall, vec3 &wallContactPoint, vec3 &wallNormal);
};

typedef Collider *ColliderP;

#define DEFAULT_COSINE 0.70710678118654757f // aka 45 degrees

vec3 CollisionMove(const vec3& p1, const vec3& p2,
                   const std::list<ColliderP> &colliders,
                   const std::list<Triangle> &triangles);

vec3 CollisionClosestBump(const vec3& p1, const vec3& p2,
                          const std::list<ColliderP> &colliders,
                          const std::list<Triangle> &triangles);

std::tuple<bool, Triangle, vec3> CollisionTraceBeam(const vec3& p1, const vec3 &p2,
                                                    const std::list<Triangle> &triangles);
#endif // COLLISION_H
