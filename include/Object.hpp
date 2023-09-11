#pragma once

#include "Vector.hpp"
#include "Global.hpp"
#include "Bounds3.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"

class Object
{
public:
    Object() = default;
    virtual ~Object() {}
    virtual bool intersect(const Ray &ray) = 0;
    virtual bool intersect(const Ray &ray, float &, uint32_t &) const = 0;
    virtual Intersection getIntersection(const Ray &ray) = 0;
    virtual void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &, const Vector2f &, Vector3f &, Vector2f &) const = 0;
    virtual Vector3f evalDiffuseColor(const Vector2f &) const = 0;
    virtual Bounds3 getBounds() = 0;
    virtual float getArea() = 0;
    virtual void Sample(Intersection &pos, float &pdf) = 0;
    virtual bool hasEmit() = 0;
};