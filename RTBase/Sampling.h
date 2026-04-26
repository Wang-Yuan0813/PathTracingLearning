#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};
class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)//given two random numbers between 0 and 1
	{
		// uniform on hemisphere: z = r1 in [0,1], phi = 2*pi*r2
		float z = r1;
		float phi = 2.0f * M_PI * r2;
		float r = sqrtf(std::max(0.0f, 1.0f - z * z));
		return Vec3(cosf(phi) * r, sinf(phi) * r, z);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		// uniform over hemisphere (z-up local): pdf = 1 / (2*pi) for z>0
		return (wi.z > 0.0f) ? (1.0f / (2.0f * M_PI)) : 0.0f;
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// cosine-weighted sampling via disk method
		float r = sqrtf(r1);
		float phi = 2.0f * M_PI * r2;
		float x = r * cosf(phi);
		float y = r * sinf(phi);
		float z = sqrtf(std::max(0.0f, 1.0f - (x * x + y * y)));
		return Vec3(x, y, z);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		// cosine-weighted pdf: cos(theta) / pi
		return (wi.z > 0.0f) ? (wi.z / M_PI) : 0.0f;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		float z = 1.0f - 2.0f * r1;
		float phi = 2.0f * M_PI * r2;
		float r = sqrtf(std::max(0.0f, 1.0f - z * z));
		return Vec3(cosf(phi) * r, sinf(phi) * r, z);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		return 1.0f / (4.0f * M_PI);
	}
};
//class SamplingDistributions
//{
//public:
//	static Vec3 uniformSampleHemisphere(float r1, float r2)//given two random numbers between 0 and 1
//	{
//		// Add code here
//		float z = r1;
//		float phi = 2.0f * M_PI * r2;
//		float r = sqrtf(std::max(0.0f, 1.0f - z * z));
//		return Vec3(cosf(phi) * r, sinf(phi) * r, z);
//
//		//return SphericalCoordinates::sphericalToWorld(acos(r1), 2 * M_PI * r2);
//	}
//	static float uniformHemispherePDF(const Vec3 wi)
//	{
//		// Add code here
//		return (wi.z > 0.0f) ? (1.0f / (2.0f * M_PI)) : 0.0f;
//
//		//return 1.0f / (2.0f * M_PI);
//	}
//	static Vec3 cosineSampleHemisphere(float r1, float r2)
//	{
//		// Add code here
//		float r = sqrtf(r1);
//		float phi = 2.0f * M_PI * r2;
//		float x = r * cosf(phi);
//		float y = r * sinf(phi);
//		float z = sqrtf(std::max(0.0f, 1.0f - (x * x + y * y)));
//		return Vec3(x, y, z);
//
//		//return SphericalCoordinates::sphericalToWorld(acos(sqrt(r1)), 2 * M_PI * r2);
//	}
//	static float cosineHemispherePDF(const Vec3 wi)
//	{
//		// Add code here
//		//return 1.0f / (2.0f * M_PI);
//		return (wi.z > 0.0f) ? (wi.z / M_PI) : 0.0f;
//
//	}
//	static Vec3 uniformSampleSphere(float r1, float r2)
//	{
//		// Add code here
//		float z = 1.0f - 2.0f * r1;
//		float phi = 2.0f * M_PI * r2;
//		float r = sqrtf(std::max(0.0f, 1.0f - z * z));
//		return Vec3(cosf(phi) * r, sinf(phi) * r, z);
//
//		//return SphericalCoordinates::sphericalToWorld(acos(1 - 2 * r1), 2 * M_PI * r2);
//	}
//	static float uniformSpherePDF(const Vec3& wi)
//	{
//		// Add code here
//		return 1.0f / (4.0f * M_PI);
//	}
//};