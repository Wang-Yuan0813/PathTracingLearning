#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};
class ShadingHelper
{
public:

	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here
		float ior = iorExt / iorInt;
		float temp = (1 - SQ(ior) * (1 - SQ(cosTheta)));
		if (temp < 0)	return 1.0f;//like a mirror
		float cosTheta_t = sqrt(temp);
		float p = (cosTheta - cosTheta_t * ior) / (cosTheta + cosTheta_t * ior);
		float v = (cosTheta * ior - cosTheta_t) / (cosTheta * ior + cosTheta_t);
		return (SQ(p) + SQ(v)) / 2;
		//return 1.0f;
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// the ior and k here are perporties of metals
		// cosTheta should be positive
		// Add code here

		auto cal = [cosTheta](float _ior, float _k) {
			float p, v;
			p = (SQ(_ior) + SQ(_k)) * SQ(cosTheta) - 2 * _ior * cosTheta + 1 - SQ(cosTheta);
			p = p / ((SQ(_ior) + SQ(_k)) * SQ(cosTheta) + 2 * _ior * cosTheta + 1 - SQ(cosTheta));
			v = SQ(_ior) + SQ(_k) - 2 * _ior * cosTheta + SQ(cosTheta);
			v = v / (SQ(_ior) + SQ(_k) + 2 * _ior * cosTheta + SQ(cosTheta));
			return std::min(1.0f, std::max(0.0f, (p + v) / 2));
			};
		return Colour(cal(ior.r, k.r), cal(ior.g, k.g), cal(ior.b, k.b));
		//return Colour(1.0f, 1.0f, 1.0f);
	}
	static float lambdaGGX(const Vec3& wi, float alpha)
	{
		// Add code here
		//return 1.0f;
		if (wi.z <= EPSILON)	return 1e6f;
		float cosTheta2 = SQ(wi.z);
		float sinTheta2 = 1 - cosTheta2;
		return (sqrt(1 + SQ(alpha) * sinTheta2 / cosTheta2) - 1) / 2;

	}
	static float Gggx(const Vec3& wi, const Vec3& wo, float alpha)
	{
		// Add code here
		//return 1.0f;

		return (1.0f / (1.0f + lambdaGGX(wi,alpha))) * (1.0f / (1.0f + lambdaGGX(wo, alpha)));
	}
	static float Dggx(const Vec3& h, float alpha)
	{
		// Add code here
		float alpha2 = SQ(alpha);//store result
		return alpha2 / SQ(M_PI * ((SQ(h.z) * (alpha2 - 1)) + 1));
	}
};
class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	virtual void printBSDFName() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		float r1 = sampler->next();
		float r2 = sampler->next();
		Vec3 wi_local = SamplingDistributions::cosineSampleHemisphere(r1, r2);
		pdf = SamplingDistributions::cosineHemispherePDF(wi_local);
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		// convert to world
		return shadingData.frame.toWorld(wi_local);

		/*Vec3 wi = Vec3(0, 1, 0);
		pdf = 1.0f;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);

		//return 1.0f;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	void printBSDFName() {
		std::cout << "DiffuseBSDF" << std::endl;
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.z <= 0.0f)
		{
			pdf = 0.0f;
			return Vec3(0.0f, 0.0f, 0.0f);
		}
		// Perfect reflection: reflect x and y, keep z
		Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
		// Convert back to world space
		Vec3 wi = shadingData.frame.toWorld(wiLocal);
		float cosThetaR = fabsf(woLocal.z);
		if (cosThetaR <= EPSILON) cosThetaR = EPSILON;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / cosThetaR;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		//return Colour(1.0f, 0.0f, 0.0f);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		// if outgoing or incoming below the surface, no contribution
		if (woLocal.z <= 0.0f || wiLocal.z <= 0.0f)
			return Colour(0.0f);

		// expected perfect mirror reflection in local space
		Vec3 wiExpected(-woLocal.x, -woLocal.y, woLocal.z);

		// compare with a small tolerance
		if (fabsf(wiLocal.x - wiExpected.x) < EPSILON &&
			fabsf(wiLocal.y - wiExpected.y) < EPSILON &&
			fabsf(wiLocal.z - wiExpected.z) < EPSILON)
		{
			float cosThetaR = fabsf(woLocal.z);
			if (cosThetaR < EPSILON) cosThetaR = EPSILON;
			return albedo->sample(shadingData.tu, shadingData.tv) / cosThetaR;
		}

		return Colour(0.0f);
		//return Colour(0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	void printBSDFName() {
		std::cout << "MirrorBSDF" << std::endl;
	}
};
class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
		std::cout << k.r << ',' << k.g << ',' << k.b << std::endl;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		float r1 = sampler->next();
		float r2 = sampler->next();
		float phi = 2 * M_PI * r2;
		float cosTheta = sqrt((1 - r1)/(r1 * (SQ(alpha) - 1) + 1));
		float sinTheta = sqrtf(std::max(0.0f, 1.0f - SQ(cosTheta))); //sqrt(1 - SQ(cosTheta));
		
		Vec3 wmLocal(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
		Vec3 wm = shadingData.frame.toWorld(wmLocal);

		Vec3 wo = shadingData.wo;
			
		if (Dot(wo, wm) <= EPSILON) {
			pdf = 0.0f;
			reflectedColour = Colour(0);
			return Vec3(0);
		}
		Vec3 wi = -wo + wm * 2.0f * Dot(wo, wm);	

		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(wo);

		if (wiLocal.z <= EPSILON || woLocal.z <= EPSILON){
			pdf = 0.0f;
			reflectedColour = Colour(0);
			return Vec3(0);
		}
		//ShadingHelper functions
		float D = ShadingHelper::Dggx(wmLocal, alpha);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		

		Colour F = ShadingHelper::fresnelConductor(std::max(EPSILON, Dot(wo, wm)), eta, k);

		reflectedColour = F * (D * G / (4.0f * std::max(EPSILON, fabs(wiLocal.z) * fabs(woLocal.z))));//try to avoid white dots
		
		if (!std::isfinite(reflectedColour.r)) reflectedColour.r = 0.0f;
		if (!std::isfinite(reflectedColour.g)) reflectedColour.g = 0.0f;
		if (!std::isfinite(reflectedColour.b)) reflectedColour.b = 0.0f;
		//try avoid colored spikes
		reflectedColour.r = std::min(reflectedColour.r, 100.0f);
		reflectedColour.g = std::min(reflectedColour.g, 100.0f);
		reflectedColour.b = std::min(reflectedColour.b, 100.0f);

		pdf = D * fabsf(wmLocal.z) / (4.0f * std::max(EPSILON, Dot(wo, wm)));
		//pdf = std::max(pdf, EPSILON);
		return wi;
	}
	
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		//return Colour(1.0f, 0.0f, 0.0f);
		Vec3 wo = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.z <= EPSILON || wiLocal.z <= EPSILON)
			return Colour(0.0f);

		Vec3 wm = wi + wo;
		if (wm.length() <= EPSILON)
			return 0.0f;
		wm = wm.normalize();
		Vec3 wmLocal = shadingData.frame.toLocal(wm);

		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		float D = ShadingHelper::Dggx(wmLocal, alpha);
		Colour F = ShadingHelper::fresnelConductor(Dot(wo, wm), eta, k);
		//std::cout << "conduct" << std::endl;
		Colour res = F * G * D / (4 * std::max(EPSILON, fabs(wiLocal.z) * fabs(woLocal.z)));;
		//try avoid colored spikes
		if (!std::isfinite(res.r)) res.r = 0.0f;
		if (!std::isfinite(res.g)) res.g = 0.0f;
		if (!std::isfinite(res.b)) res.b = 0.0f;
		res.r = std::min(res.r, 100.0f);
		res.g = std::min(res.g, 100.0f);
		res.b = std::min(res.b, 100.0f);
		return res;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/

		Vec3 wo = shadingData.wo;
		Vec3 woLocal = shadingData.frame.toLocal(wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.z <= EPSILON || wiLocal.z <= EPSILON)
			return 0.0f;

		Vec3 wm = wi + wo;
		if (wm.length() <= EPSILON)
			return 0.0f;
		wm = wm.normalize();
		Vec3 wmLocal = shadingData.frame.toLocal(wm);

		float D = ShadingHelper::Dggx(wmLocal, alpha);
		return D * std::abs(wmLocal.z) / (4.0f * std::max(EPSILON, Dot(wo, wm)));
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	void printBSDFName() {
		std::cout << "ConductorBSDF" << std::endl;
	}
};
#if 1
class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf){
		//in Glass, we have certain direction of light.
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.isZero()) { pdf = 0.0f; return Vec3(0.0f); }

		// whether it is entering or exiting
		float eta_i = woLocal.z > 0.0f ? extIOR : intIOR;
		float eta_t = woLocal.z > 0.0f ? intIOR : extIOR;
		float cosThetaI = fabsf(woLocal.z);

		// Fresnel reflectance for dielectric (how much light will be transmission)
		float F = ShadingHelper::fresnelDielectric(cosThetaI, eta_i, eta_t);

		// sample decision
		float u = sampler->next();

		// reflection, samilar to mirror
		if (u < F) {
			// perfect mirror
			Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z); 
			// Convert back to world space
			Vec3 wi = shadingData.frame.toWorld(wiLocal);
			float cosThetaR = fabsf(woLocal.z);
			//multiply F
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (F / cosThetaR);
			return wi;
		}
		// transmission
		else {
			Vec3 N(0.0f, 0.0f, 1.0f);
			Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
			float phit = atan2(wiLocal.y, wiLocal.x);
			float cosThetai = fabs(wiLocal.z);
			float sinThetai = sqrt(1 - SQ(cosThetai));
			//Snell's Law
			float sinThetat = (eta_i / eta_t) * sinThetai;
			if (sinThetat > 1.0f) {//total internal reflection
				Vec3 wi = shadingData.frame.toWorld(wiLocal);
				float cosThetaR = fabsf(woLocal.z);
				//multiply F
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / cosThetai;
				return wi;
			}
			float cosThetat = sqrt(1 - SQ(sinThetat));
			Vec3 wtLocal(sinThetat * cosf(phit), sinThetat * sinf(phit), woLocal.z > 0 ? -cosThetat : cosThetat);
			Vec3 wt = shadingData.frame.toWorld(wtLocal);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (SQ(eta_i / eta_t) * (1.0f - F) / cosThetat);
			pdf = 0.0f;
			return wt;
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.isZero() || wiLocal.isZero()) return Colour(0.0f);

		// reject below-surface directions (convention used elsewhere)
		if (woLocal.z == 0.0f) return Colour(0.0f);

		// prepare Fresnel and ior
		float eta_i = woLocal.z > 0.0f ? extIOR : intIOR;
		float eta_t = woLocal.z > 0.0f ? intIOR : extIOR;
		float cosThetaI = fabsf(woLocal.z);
		float F = ShadingHelper::fresnelDielectric(cosThetaI, eta_i, eta_t);

		// expected reflection
		Vec3 wiRef(-woLocal.x, -woLocal.y, woLocal.z);
		if (fabsf(wiLocal.x - wiRef.x) < EPSILON &&
			fabsf(wiLocal.y - wiRef.y) < EPSILON &&
			fabsf(wiLocal.z - wiRef.z) < EPSILON) {//first part of the function
			// perfect mirror
			Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
			// Convert back to world space
			Vec3 wi = shadingData.frame.toWorld(wiLocal);
			float cosThetaR = fabsf(woLocal.z);
			if (cosThetaR < EPSILON) cosThetaR = EPSILON;
			//multiply F
			return albedo->sample(shadingData.tu, shadingData.tv) * (F / cosThetaR);
		}
		// expected transmission: compute refracted direction and compare
		else {//second part of the function
			Vec3 N(0.0f, 0.0f, 1.0f);
			Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
			float phit = atan2(wiLocal.y, wiLocal.x);
			float cosThetai = fabs(wiLocal.z);
			float sinThetai = sqrt(1 - SQ(cosThetai));
			//Snell's Law
			float sinThetat = (eta_i / eta_t) * sinThetai;
			float cosThetat = sqrt(1 - SQ(sinThetat));
			Vec3 wtLocal(sinThetat * cosf(phit), sinThetat * sinf(phit), -cosThetat);
			Vec3 wt = shadingData.frame.toWorld(wtLocal);
			return albedo->sample(shadingData.tu, shadingData.tv) * (SQ(eta_i / eta_t) * (1.0f - F) / cosThetat);
		}
		return Colour(0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	void printBSDFName() {
		std::cout << "GlassBSDF" << std::endl;
	}
};
#else
class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;*/
		//in Glass, we have certain direction of light.
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.isZero()) { pdf = 0.0f; return Vec3(0.0f); }

		// whether it is entering or exiting
		float eta_i = woLocal.z > 0.0f ? extIOR : intIOR;
		float eta_t = woLocal.z > 0.0f ? intIOR : extIOR;
		float cosThetaI = fabsf(woLocal.z);

		// Fresnel reflectance for dielectric
		float F = ShadingHelper::fresnelDielectric(cosThetaI, eta_i, eta_t);

		// sample decision
		float u = sampler->next();

		// reflection, samilar to mirror
		if (u < F){
			Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z); // perfect mirror
			Vec3 wi = shadingData.frame.toWorld(wiLocal);
			float cosThetaR = fabsf(woLocal.z);
			if (cosThetaR < EPSILON) cosThetaR = EPSILON;
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (F / cosThetaR);
			return wi;
		}
		// transmission
		else{
			Vec3 N(0.0f, 0.0f, 1.0f);
			Vec3 I = -woLocal;
			float eta = eta_i / eta_t;
			float NdotI = Dot(N, I); //NdotI = -woLocal.z
			float k = 1.0f - SQ(eta) * (1.0f - SQ(NdotI));
			Vec3 T = I * eta  - N * (eta * NdotI + sqrtf(k));
			Vec3 wiLocal = T; // already points into transmitted medium
			Vec3 wi = shadingData.frame.toWorld(wiLocal);

			float cosThetaT = fabsf(wiLocal.z);
			if (cosThetaT < EPSILON) cosThetaT = EPSILON;

			// scale factor from rendering equation for transmission delta term:
			float etaScale = SQ(eta_t) / SQ(eta_i);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (etaScale * (1.0f - F) / cosThetaT);
			return wi;
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		if (woLocal.isZero() || wiLocal.isZero()) return Colour(0.0f);

		// reject below-surface directions (convention used elsewhere)
		if (woLocal.z == 0.0f) return Colour(0.0f);

		// prepare Fresnel and ior
		float eta_i = woLocal.z > 0.0f ? extIOR : intIOR;
		float eta_t = woLocal.z > 0.0f ? intIOR : extIOR;
		float cosThetaI = fabsf(woLocal.z);
		float F = ShadingHelper::fresnelDielectric(cosThetaI, eta_i, eta_t);

		// expected reflection
		Vec3 wiRef(-woLocal.x, -woLocal.y, woLocal.z);
		if (fabsf(wiLocal.x - wiRef.x) < EPSILON &&
			fabsf(wiLocal.y - wiRef.y) < EPSILON &&
			fabsf(wiLocal.z - wiRef.z) < EPSILON){//first part of the function
			float cosThetaR = fabsf(woLocal.z);
			if (cosThetaR < EPSILON) cosThetaR = EPSILON;
			return albedo->sample(shadingData.tu, shadingData.tv) * (F / cosThetaR);
		}
		// expected transmission: compute refracted direction and compare
		else{//second part of the function
			Vec3 N(0.0f, 0.0f, 1.0f);
			Vec3 I = -woLocal;
			float eta = eta_i / eta_t;
			float NdotI = Dot(N, I);
			float k = 1.0f - SQ(eta) * (1.0f - SQ(NdotI));
			Vec3 T = I * eta - N * (eta * NdotI + sqrtf(k));
			// compare T with wiLocal
			if (fabsf(wiLocal.x - T.x) < EPSILON &&
				fabsf(wiLocal.y - T.y) < EPSILON &&
				fabsf(wiLocal.z - T.z) < EPSILON){
				float cosThetaT = fabsf(T.z);
				if (cosThetaT < EPSILON) cosThetaT = EPSILON;
				float etaScale = SQ(eta_t) / SQ(eta_i);
				return albedo->sample(shadingData.tu, shadingData.tv) * (etaScale * (1.0f - F) / cosThetaT);
			}
		}
		return Colour(0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	void printBSDFName() {
		std::cout << "GlassBSDF" << std::endl;
	}
};
#endif 
class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		//return Colour(1.0f, 0.0f, 0.0f);
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	void printBSDFName() {
		std::cout << "DielectricBSDF" << std::endl;
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	void printBSDFName() {
		std::cout << "OrenNayarBSDF" << std::endl;
	}
};
class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code

		float r1 = sampler->next();
		float r2 = sampler->next();
		float cosTheta = pow(r1, 1.0f / (alphaToPhongExponent() + 1.0f));
		float sinTheta = sqrt(1 - SQ(cosTheta));
		float lobePhi = 2 * M_PI * r2;

		Vec3 wiLobe(sinTheta * cosf(lobePhi), sinTheta * sinf(lobePhi), cosTheta);

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
		wr.normalize();

		Frame frame;//create a frame by using wr
		frame.fromVector(wr);

		Vec3 wiLocal = frame.toWorld(wiLobe);


		pdf = (alphaToPhongExponent() + 1.0f) / (2.0f * M_PI) * pow(std::max(0.0f, Dot(wr, wiLocal)), alphaToPhongExponent());

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv)
			* ((alphaToPhongExponent() + 2.0f) / (2.0f * M_PI)) * pow(std::max(0.0f, Dot(wr, wiLocal)), alphaToPhongExponent());

		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		if (wiLocal.z <= 0)		return 0.0f;
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.isZero())	return 0.0f;
		Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
		//normalized
		wr = wr.normalize();
		wiLocal = wiLocal.normalize();
		return albedo->sample(shadingData.tu, shadingData.tv) 
			* (alphaToPhongExponent() + 2.0f) / (2.0f * M_PI) * pow(std::max(0.0f, Dot(wr, wiLocal)), alphaToPhongExponent());

	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		if (wiLocal.z <= 0)		return 0.0f;
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.isZero())	return 0.0f;
		Vec3 wr(-woLocal.x, -woLocal.y, woLocal.z);
		// ensure normalized (to be robust)
		wr = wr.normalize();
		wiLocal = wiLocal.normalize();
		return (alphaToPhongExponent() + 1) / (2 * M_PI) * pow(std::max(0.0f, Dot(wr, wiLocal)), alphaToPhongExponent());
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
	void printBSDFName() {
		std::cout << "PlasticBSDF" << std::endl;
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
	void printBSDFName() {
		std::cout << "LayeredBSDF" << std::endl;
	}
};