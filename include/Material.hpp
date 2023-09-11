#pragma once

#include "Vector.hpp"
#include "Global.hpp"

enum class MaterialType
{
    diffuse,
    MicroFacet
};

class Material
{
public:
    // 类型和发光强度emission
    Material(MaterialType t = MaterialType::diffuse, Vector3f e = Vector3f(0, 0, 0))
        : m_type(t), m_emission(e)
    {
    }

    inline MaterialType getType() { return m_type; }

    inline Vector3f getColorAt(double u, double v) { return Vector3f(0, 0, 0); }

    inline Vector3f getEmission() { return m_emission; }

    inline bool hasEmission() {return m_emission.norm()>EPSILON;}

    Vector3f sample(const Vector3f& wi,const Vector3f& N);
    
    float pdf(const Vector3f& wi,const Vector3f& wo,const Vector3f& N);
    
    Vector3f eval(const Vector3f& wi,const Vector3f& wo,const Vector3f&N);

//    微表面模型
	float Trowbridge_Reitz_GGX_D(Vector3f normal, Vector3f halfVector,float a);

	float Schick_GGX(float NdotV,float k);

	float Schick_GGXSmith_G(Vector3f N,Vector3f V,Vector3f L,float k);

	float Schick_Fresnel_F(float cosTheta,float F0);

public:
    MaterialType m_type;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;

    

// 计算反射角
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
}

// Snell法则处理折射，两种情况
// 1：光线在物体内部，取正余弦 cosi = -N.I
// 2：光线在物体外面，反转法线并将折射率取反
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    Vector3f n(N);
    if (cosi < 0)
        cosi *= -1;
    else
    {
        std::swap(etai, etat);
        n = -N;
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrt(k)) * n;
}

// 计算菲涅尔系数
// I是入射光，
// N是交点法线，
// ior是材料折射系数，
// kr是光线反射的系数，
// 根据能量守恒，kt也就是折射光是1-kr
void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0)
        std::swap(etai, etat);
    float sint = etai / etat * sqrt(std::max(0.f, 1 - cosi * cosi));
    if (sint >= 1)
        kr = 1;
    else
    {
        float cost = sqrt(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        kr = (Rs * Rs + Rp * Rp) / 2;
    }
}

Vector3f toWorld(const Vector3f &a, const Vector3f &N)
{

    Vector3f B, C;
    if (std::fabs(N.x) > std::fabs(N.y))
    {
        float invlen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
	    C = Vector3f(N.z * invlen, 0.0f, -N.x *invlen);
    }
    else
    {
        float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
        C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
    }
    B = crossProduct(C, N);
    return a.x * B + a.y * C + a.z * N;
}

};

