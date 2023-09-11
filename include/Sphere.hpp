#pragma once

#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"

class Sphere : public Object
{
public:
    Sphere(const Vector3f &c, const float &r, Material *mt = new Material())
        : center(c), radius(r), radius2(r * r), m(mt), area(4 * M_PI * r * r)
    {
    }
    bool intersect(const Ray &ray) override;
    
    bool intersect(const Ray &ray, float &tnear, uint32_t &index) const override;
    
    Intersection getIntersection(const Ray &ray) override;


    void Sample(Intersection &pos, float &pdf) override;
    
    float getArea() { return area; }

    bool hasEmit() { return m->hasEmission(); }

    Bounds3 getBounds()
    {
        return Bounds3(Vector3f(center.x - radius, center.y - radius, center.z - radius),
                       Vector3f(center.x + radius, center.y + radius, center.z + radius));
    }
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const
    {
        N = normalize(P - center);
    }

    Vector3f evalDiffuseColor(const Vector2f &mt) const
    {
        return m->getColorAt(mt.x, mt.y);
    }

public:
    Vector3f center;
    float radius, radius2;
    Material *m;
    float area;
};