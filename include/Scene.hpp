#pragma once

#include <vector>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"
#include "AreaLight.hpp"
#include "BVH.hpp"
#include "Ray.hpp"

class Scene
{
public:
    Scene(int w, int h) : width(w), height(h)
    {
    }

    void Add(Object *object) { objects.emplace_back(object); }
	void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    const std::vector<Object *> &get_objects() const { return objects; }
    const Intersection intersect(const Ray &ray) const;

    void buildBVH();
    Vector3f castRay(const Ray &ray, int depth) const;
    void sampleLight(Intersection &pos, float &pdf) const;
    bool trace(const Ray &ray, const std::vector<Object *> &objects, float &tnear, uint32_t &index, Object *&hitObject);

public:
    int width = 1280;
    int height = 960;
    double fov = 40.0;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    int maxDepth = 1;
    float RussianRoulette = 0.8;

    std::vector<Object *> objects;
	std::vector<std::unique_ptr<Light> > lights;

    BVHAccel *bvh;
};