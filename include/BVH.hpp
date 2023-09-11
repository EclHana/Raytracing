#pragma once
#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode
{
    BVHBuildNode()
        : left(nullptr), right(nullptr), object(nullptr)
    {
    }
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object *object;
    float area;
    int splitAxis = 0;
    int firstPrimOffset = 0;
    int nPrimitives = 0;
};


// BVH二叉树加速结构
class BVHAccel
{
public:
    enum class SplitMethod
    {
        NAIVE,
        SAH
    };
    BVHAccel(std::vector<Object *> p, int maxPrimsInNode = 1, SplitMethod method = SplitMethod::NAIVE);
    ~BVHAccel() = default;

    Intersection Intersect(const Ray &ray) const;
    BVHBuildNode *root;
    void Sample(Intersection &pos, float &pdf);

private:
    Intersection getIntersection(BVHBuildNode *node, const Ray &ray) const;
    BVHBuildNode *recursiveBuild(std::vector<Object *> objects);
    void getSample(BVHBuildNode *node, float p, Intersection &pos, float &pdf);
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object *> primitives;
};

