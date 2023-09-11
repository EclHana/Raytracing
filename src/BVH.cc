#include <algorithm>
#include "BVH.hpp"

//--
BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode, SplitMethod method)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(method), primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (!primitives.empty())
    {
        root = recursiveBuild(primitives);
        time(&stop);
        double diff = difftime(stop, start);
        int hours = (int)diff / 3600;
        int mins = ((int)diff / 60) - (hours * 60);
        int secs = (int)diff - (hours * 3600) - (mins * 60);
        printf(
            "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
            hours, mins, secs);
    }
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    if (!root)
        return Intersection();
    return getIntersection(root, ray);
}
Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    if (!node || !(node->bounds.CanIntersect(ray, ray.direction_inv, {0, 0, 0})))
    {
        return Intersection();
    }
    if (!node->left && !node->right)
    {
        return node->object->getIntersection(ray);
    }
    auto hitl = getIntersection(node->left, ray);
    auto hitr = getIntersection(node->right, ray);
    return hitl.distance >= hitr.distance ? hitr : hitl;
}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects)
{
    BVHBuildNode *node = new BVHBuildNode();
    int n = objects.size();

    if (n == 1)
    {
        // 场景小的情况下，叶子节点只存一个物体，递归到这里结束
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->area = objects[0]->getArea();
    }
    else if (n == 2)
    {
        node->left = recursiveBuild(std::vector<Object *>{objects[0]});
        node->right = recursiveBuild(std::vector<Object *>{objects[1]});

        node->bounds = Bounds3::Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }
    else
    {
        // 两个以上的物体要进行分治划分，按照最长轴进行划分
        Bounds3 centroidBounds; // 中心点的包围盒
        for (int i = 0; i < n; i++)
        {
            centroidBounds = Bounds3::Union(centroidBounds, objects[i]->getBounds().Centroid());
        }
        // 找到最长轴进行划分
        int dim = centroidBounds.maxExtent();
        switch (dim)
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().x <
                               f2->getBounds().Centroid().x; });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().y <
                               f2->getBounds().Centroid().y; });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().z <
                               f2->getBounds().Centroid().z; });
            break;
        }
        auto midIt = objects.begin() + n / 2;
        node->left = recursiveBuild(std::vector<Object *>(objects.begin(), midIt));
        node->right = recursiveBuild(std::vector<Object *>(midIt, objects.end()));
        node->bounds = Bounds3::Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }
    return node;
}
void BVHAccel::getSample(BVHBuildNode *node, float p, Intersection &pos, float &pdf)
{
    if (!node->left || !node->right)
    {
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if (p < node->left->area)
        getSample(node->left, p, pos, pdf);
    else
        getSample(node->right, p - node->left->area, pos, pdf);
}
void BVHAccel::Sample(Intersection &pos, float &pdf)
{
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}