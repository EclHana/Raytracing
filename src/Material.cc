#include "Material.hpp"


Vector3f Material::sample(const Vector3f& wi,const Vector3f& N)
{
    switch (m_type)
    {
    case MaterialType::MicroFacet:
    case MaterialType::diffuse:
        float x1 = get_random_float(),x2 = get_random_float();
        float z = std::fabs(1.0f - 2.0f*x1);
        float r = std::sqrt(1.0f - z*z) , phi = 2*M_PI*x2;
        Vector3f loaclRay(r*std::cos(phi),r*std::sin(phi),z);
        return toWorld(loaclRay,N);
    }
    return Vector3f(0,0,0);
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
	switch(m_type){
		case MaterialType::MicroFacet:
		case MaterialType::diffuse:
		{
			// uniform sample probability 1 / (2 * PI)
			if (dotProduct(wo, N) > 0.0f)
				return 0.5f / M_PI;
			else
				return 0.0f;
			break;
		}

	}
	return 0.0f;
}

// 求pos点的BRDF，也就是积分的fr
Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
	switch(m_type){
		case MaterialType::diffuse:
		{
			// calculate the contribution of diffuse   model
			float cosalpha = dotProduct(N, wo);
			if (cosalpha > 0.0f) {
				Vector3f diffuse = Kd / M_PI;
				return diffuse;
			}
			else
				return Vector3f(0.0f);
			break;
		}
		case MaterialType::MicroFacet:
		{
			float cosalpha = dotProduct(N, wo);
			if (cosalpha > 0.0001f) {
				float roughness=0.02f;
				Vector3f V = -wi;
				Vector3f L = wo;
				Vector3f H =normalize(V+L);
				float D =Trowbridge_Reitz_GGX_D(N,H,roughness);
				float G =Schick_GGXSmith_G(N,V,L,roughness);
				float F = Schick_Fresnel_F(cosalpha,0.50f);
				float diffsue=dotProduct(N,V)+0.5f;
				float divide=1.0f/(4*std::max(dotProduct(N,L),0.0001f)*std::max(dotProduct(N,V),0.0001f));
				float Specular = D*F*G*divide;
				//std::cout << D <<"  " << G <<"  "<< F<<"  "<< divide<<"  "<< Specular<<std::endl;
				return diffsue* Kd /M_PI+Ks*Specular;
			}
			else
				return Vector3f(0.0f);
			break;
		}
	}
	return Vector3f(0.0f);
}

float Material::Trowbridge_Reitz_GGX_D(Vector3f normal, Vector3f halfVector,float a)
{
	float a2=a*a;
	float NdotH =std::max(dotProduct(normal,halfVector),0.0f);
	float NdotH2 =NdotH*NdotH;
	float nom=a2;
	float denom=(NdotH2*(a2-1.0f)+1.0f);
	denom=M_PI*denom*denom;
	return nom/std::max(denom,0.00001f);
}
float Material::Schick_GGX(float NdotV,float k)
{
	float nom=NdotV;
	float denom=NdotV*(1.0f-k)+k;
	return nom/std::max(denom,0.00001f);
}
float Material::Schick_GGXSmith_G(Vector3f N,Vector3f V,Vector3f L,float k)
{
	k = std::pow(k+1.0f,2.0f)/8.0f;
	float NdotV=std::max(dotProduct(N,V),0.0f);
	float NdotL=std::max(dotProduct(N,L),0.0f);
	float ggx1=Schick_GGX(NdotV,k);
	float ggx2=Schick_GGX(NdotL,k);
	return ggx1*ggx2;
}


float Material::Schick_Fresnel_F(float cosTheta,float F0)
{
	return F0+(1.0-F0)*std::pow(1.0-cosTheta,5.0f);
}

