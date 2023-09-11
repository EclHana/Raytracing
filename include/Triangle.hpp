#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "Object.hpp"
#include <array>

// 三角形与光线求交
bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1,
                          const Vector3f &v2, const Vector3f &orig,
                          const Vector3f &dir, float &tnear, float &u, float &v);

class Triangle : public Object
{
public:
    Triangle(Vector3f &_v0, Vector3f &_v1, Vector3f &_v2, Material *_m = nullptr);

    bool intersect(const Ray &ray) override;
    bool intersect(const Ray &ray, float &tnear, uint32_t &index) const override;

    Intersection getIntersection(const Ray &ray) override;
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I,
                              const uint32_t &index, const Vector2f &uv,
                              Vector3f &N, Vector2f &st) const override
    {
        N = normal;
    }

    Vector3f evalDiffuseColor(const Vector2f &) const override;

    Bounds3 getBounds() override;

    void Sample(Intersection &pos, float &pdf) override;

    inline float getArea() { return area; }

    inline bool hasEmit() { return m->hasEmission(); }

public:
    Vector3f v0, v1, v2; // vertices A, B ,C
    Vector3f e1, e2;     // edge1(v1-v0), edge2(v2-v0)
    Vector3f t0, t1, t2; // texture coords
    Vector3f normal;
    float area;
    Material *m;
};

// 物体对应的三角面组
class MeshTriangle : public Object
{
public:
    MeshTriangle(const std::string filename, Material *mt = new Material());

    bool intersect(const Ray &ray) override;

    bool intersect(const Ray &ray, float &tnear, uint32_t &index) const override;

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I,
                              const uint32_t &index, const Vector2f &uv,
                              Vector3f &N, Vector2f &st) const override;

    Vector3f evalDiffuseColor(const Vector2f &st) const override;

    Intersection getIntersection(const Ray &ray) override;

    void Sample(Intersection &pos, float &pdf) override;

    Bounds3 getBounds() { return bounding_box; }

    float getArea() { return area; }

    bool hasEmit() { return m->hasEmission(); }

public:
    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel *bvh;
    float area;
    Material *m;
};