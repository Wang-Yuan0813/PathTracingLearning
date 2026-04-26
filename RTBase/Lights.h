#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		/*Vec3 wi = Vec3(0, 0, 1);
		pdf = 1.0f;
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);*/

		float u1 = sampler->next();
		float u2 = sampler->next();
		float r = sqrt(u1);
		float theta = 2.0f * M_PI * u2;

		float x = r * cos(theta);
		float y = r * sin(theta);
		float z = sqrt(std::max(0.0f, 1.0f - u1));

		Vec3 wiLocal(x, y, z);

		pdf = z / M_PI;  // cos(theta) / pi

		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wiLocal);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
		//emission = Colour(0.0f);
		
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;

	//for tabulated distribution
	std::vector<float> rowCDF;            
	std::vector<std::vector<float>> colCDF;
	float totalWeight = 0.0f;              

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		buildCDF();
	}
	void buildCDF() {//build distribution for importance sampling lighting based on luminance
		
		rowCDF.resize(env->height);
		colCDF.resize(env->height, std::vector<float>(env->width));
		//easy for calculating x | y
		totalWeight = 0.0f;

		for (int y = 0; y < env->height; y++){
			float v = (y + 0.5f) / env->height;
			float theta = v * M_PI;
			float sinTheta = sinf(theta);
			float rowSum = 0.0f;
			for (int x = 0; x < env->width; x++){
				float lum = env->texels[y * env->width + x].Lum();
				float w = lum * sinTheta;  

				rowSum += w;
				colCDF[y][x] = rowSum;
			}
			if (rowSum > 0.0f){
				for (int x = 0; x < env->width; x++)
					colCDF[y][x] /= rowSum;
			}
			totalWeight += rowSum;
			rowCDF[y] = totalWeight;
		}
		for (int y = 0; y < env->height; y++)
			rowCDF[y] /= totalWeight;
		std::cout << "build cdf map, totalweight = " << totalWeight << std::endl;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(wi);
		return wi;*/
		//tabulated
		//get row
		float r1 = sampler->next();
		int pixely = int(std::lower_bound(rowCDF.begin(), rowCDF.end(), r1) - rowCDF.begin());
		pixely = pixely > env->height - 1 ? env->height - 1 : pixely;
		//get col
		float r2 = sampler->next();
		int pixelx = int(std::lower_bound(colCDF[pixely].begin(), colCDF[pixely].end(), r2) - colCDF[pixely].begin());
		pixelx = pixelx > env->width - 1 ? env->width - 1 : pixelx;
		//int pixely = int(r2 * env->height);
		float u = (pixelx + 0.5f) / (float)(env->width);
		float v = (pixely + 0.5f) / (float)(env->height);
		//angle mapping
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;
		//store result
		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);
		float sinPhi = sinf(phi);
		float cosPhi = cosf(phi);
		//get direction vector
		Vec3 wi(cosPhi * sinTheta, cosTheta, sinTheta * sinPhi);
		//const static float SQPI = SQ(M_PI);//store result
		//pdf = puv / (2 * SQPI * sinTheta);//this will generate a very low pdf!
		//get puv's term
		float dPhi = 2.0f * M_PI / float(env->width);
		float dTheta = M_PI / float(env->height);
		float theta0 = (float(pixely) + 0.5f) * dTheta;
		float sinTheta0 = sinf(theta0);
		//get lumination
		float lum = env->texels[pixely * env->width + pixelx].Lum() * sinTheta0;
		//calculate puv
		float puv = lum / totalWeight;
		float cosThetaHigh = cosf(theta0 + 0.5 * dTheta);
		float cosThetaLow = cosf(theta0 - 0.5 * dTheta);
		pdf = puv / (dPhi * (cosThetaLow - cosThetaHigh));
		//colour
		reflectedColour = evaluate(wi);
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		//return SamplingDistributions::uniformSpherePDF(wi);
		const static float SQPI = SQ(M_PI);//store result
		//get angles
		float theta = acosf(wi.y);
		float phi = atan2f(wi.z, wi.x);
		if (phi < 0)	phi += 2.0f * M_PI;
		//mapping
		float u = phi / (2.0f * M_PI);
		float v = theta / M_PI;
		int pixelx = int(u * env->width);
		int pixely = int(v * env->height);
		//cal
		float dPhi = 2.0f * M_PI / float(env->width);
		float dTheta = M_PI / float(env->height);
		float theta0 = (float(pixely) + 0.5f) * dTheta;
		float sinTheta0 = sinf(theta0);
		//get lumination
		float lum = env->texels[pixely * env->width + pixelx].Lum() * sinTheta0;
		//calculate puv
		float puv = lum / totalWeight;
		float cosThetaHigh = cosf(theta0 + 0.5 * dTheta);
		float cosThetaLow = cosf(theta0 - 0.5 * dTheta);
		return puv / (dPhi * (cosThetaLow - cosThetaHigh));
		/*
		float lum = env->texels[pixely * env->width + pixelx].Lum();
		float puv = (lum * sin(theta)) / totalWeight;
		return puv / (2.0f * SQPI * sin(theta));*/
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};
class VPL {
public: 
	ShadingData shadingData;
	Colour Le;
	VPL(){}
	VPL(ShadingData _shadingData, Colour _Le): shadingData(_shadingData), Le(_Le){}

};
