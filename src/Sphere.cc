#include "Sphere.hpp"


bool Sphere::intersect(const Ray &ray)
{
	Vector3f L = ray.origin - center;
	float a = dotProduct(ray.direction, ray.direction);
	float b = 2 * dotProduct(ray.direction, L);
	float c = dotProduct(L, L) - radius2;
	float t0, t1;
	float area = 4 * M_PI * radius2;
	if (!solveQuadratic(a, b, c, t0, t1)) return false;
	if (t0 < 0) t0 = t1;
	if (t0 < 0) return false;
	return true;
}

bool Sphere::intersect(const Ray &ray, float &tnear, uint32_t &index) const
{
    /*
        球面上一点p
            || p - center || = r;

        与光线相交
            p = O + t*dir
            || O + t*dir - center || = r;

        解这个方程即可
    */

	Vector3f L = ray.origin - center;
	float a = dotProduct(ray.direction, ray.direction);
	float b = 2 * dotProduct(ray.direction, L);
	float c = dotProduct(L, L) - radius2;
	float t0, t1;
	if (!solveQuadratic(a, b, c, t0, t1)) return false;
	if (t0 < 0) t0 = t1;
	if (t0 < 0) return false;
	tnear = t0;

	return true;
}

Intersection Sphere::getIntersection(const Ray &ray)
{
	Intersection result;
	result.happened = false;
	Vector3f L = ray.origin - center;
	float a = dotProduct(ray.direction, ray.direction);
	float b = 2 * dotProduct(ray.direction, L);
	float c = dotProduct(L, L) - radius2;
	float t0, t1;
	if (!solveQuadratic(a, b, c, t0, t1)) return result;
	if (t0 < 0) t0 = t1;
	if (t0 < 0.5f) return result;
	result.happened=true;

	result.coords = Vector3f(ray.origin + ray.direction * t0);
	result.normal = normalize(Vector3f(result.coords - center));
	result.m = this->m;
	result.obj = this;
	result.distance = t0;
	return result;
}

void Sphere::Sample(Intersection &pos, float &pdf)
{
	float theta = 2.0 * M_PI * get_random_float(), phi = M_PI * get_random_float();
	Vector3f dir(std::cos(phi), std::sin(phi)*std::cos(theta), std::sin(phi)*std::sin(theta));
	pos.coords = center + radius * dir;
	pos.normal = dir;
	pos.emit = m->getEmission();
	pdf = 1.0f / area;
}