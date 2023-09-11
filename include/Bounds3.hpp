#pragma once

#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

//--
class Bounds3
{
public:
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f &p) : pMax(p), pMin(p) {}
    Bounds3(const Vector3f &p1, const Vector3f &p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    inline Vector3f Diagonal() const { return pMax - pMin; }

    // 三维最长分量
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

    // 包围盒求交
    Bounds3 Intersect(const Bounds3 &b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f &p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3 &b1, const Bounds3 &b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f &p, const Bounds3 &b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }

    inline const Vector3f &operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    bool CanIntersect(const Ray &ray, const Vector3f &invDir,
                    const std::array<int, 3> &dirisNeg) const
    {
        // invDir是dir的倒数，做乘法比除法快得多
        // ray: p=o+t*d -> t=(p-o)/d
	    Vector3f t_minTemp=(pMin-ray.origin)*invDir;
	    Vector3f t_maxTemp=(pMax-ray.origin)*invDir;
	    Vector3f t_min=Vector3f::Min(t_minTemp,t_maxTemp);
	    Vector3f t_max=Vector3f::Max(t_minTemp,t_maxTemp);

	    float t_min_time=std::max(t_min.x,std::max(t_min.y,t_min.z));
	    float t_max_time=std::min(t_max.x,std::min(t_max.y,t_max.z));
	    return t_max_time >= -std::numeric_limits<float>::epsilon()&& t_min_time <= t_max_time;
    }

    static Bounds3 Union(const Bounds3 &b1, const Bounds3 &b2)
    {
        return Bounds3(Vector3f::Min(b1.pMin, b2.pMin), Vector3f::Max(b1.pMax, b2.pMax));
    }

    static Bounds3 Union(const Bounds3 &b, const Vector3f &p)
    {
        return Bounds3(Vector3f::Min(b.pMin, p), Vector3f::Max(b.pMax, p));
    }

    // 空间两点标定包围盒
    Vector3f pMin;
    Vector3f pMax;
};