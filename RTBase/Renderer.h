#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include "Threadpool.h"

//switches
#define THREADPOOL 1 //threadpool or multithreads
#define DENOISE 1 // enable denoising or not
#define MIS 1	//enable MIS or not
#define RADIOSITY 0	//enable radiosity or not
#define LIGHTTRACING 0 //light tracing or path tracing
class RayTracer
{
private:
	const float FIREFLY_CLAMP = 100.0f;
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	//denoising
	oidn::DeviceRef device;
	oidn::FilterRef filter;
	oidn::BufferRef colorBuf;
	oidn::BufferRef outputBuf;
	float* colourPtr;
	float* outputPtr;
	std::vector<VPL> vplList;
#if THREADPOOL
	Threadpool* threadpool;//can use sysInfo to get max procsnum
#else
	std::thread** threads;
#endif
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
#if DENOISE
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());//BOX
#else
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());//BOX
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter());//GAUSSIAN
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellNetravaliFilter());//MitchellNetravaliFilter
#endif
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		
		//numProcs = 1;

#if THREADPOOL
		threadpool = new Threadpool(numProcs);
#else
		threads = new std::thread * [numProcs];
#endif
		// allocate samplers and seed each differently to avoid correlated noise
		samplers = new MTRandom[numProcs];
		unsigned int base = (unsigned int)GetTickCount();
		for (int i = 0; i < numProcs; ++i)
		{
			// re-construct with different seed (copy assignment is fine)
			samplers[i] = MTRandom(base + (unsigned int)(i * 1664525u + 1013904223u));
		}
		clear();

		//normalImage.resize(noisyImage.size());
		device = oidn::newDevice();
		device.commit();
		//buffer create
		colorBuf = device.newBuffer((unsigned int)film->width * (unsigned int)film->height * 3 * sizeof(float));
		outputBuf = device.newBuffer((unsigned int)film->width * (unsigned int)film->height * 3 * sizeof(float));
		//set filter
		filter = device.newFilter("RT");
		filter.setImage("color", colorBuf, oidn::Format::Float3, film->width, film->height);
		//filter.setImage("albedo", albedoBuf, oidn::Format::Float3, film->width, film->height);
		//filter.setImage("normal", normalBuf, oidn::Format::Float3, film->width, film->height);
		filter.setImage("output", outputBuf, oidn::Format::Float3, film->width, film->height);
		filter.set("hdr", true);
		filter.set("cleanAux", true); // use auxiliary buffers to guide denoising
		filter.commit();

		colourPtr = (float*)colorBuf.getData();
		outputPtr = (float*)outputBuf.getData();
	}
	void clear()
	{
		film->clear();
	}

	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Skip if material is pure specular
		if (shadingData.bsdf->isPureSpecular())//this position can not generate a ray connect with lights.
			return Colour(0.0f, 0.0f, 0.0f);

		float pmf = 1.0f;//pmf for this light
		Light* light = scene->sampleLight(sampler, pmf);
		if (light->isArea()) {//sample an area light
			float pdfA = 0.0f;
			Colour Le(0.0f, 0.0f, 0.0f);
			//uniform sample a point
			Vec3 pLight = light->sample(shadingData, sampler, Le, pdfA);
			
			Vec3 d = pLight - shadingData.x;
			float dist2 = d.lengthSq();
			float dist = sqrtf(dist2);
			if (dist <= EPSILON) return Colour(0.0f, 0.0f, 0.0f);
			Vec3 wi = d / dist;
			//V(xi<->xi+1)
			if (!scene->visible(shadingData.x, pLight)) return Colour(0.0f, 0.0f, 0.0f);
			//geometry
			Vec3 lightN = light->normal(shadingData, -wi);
			float cosL = std::max(0.0f, Dot(lightN, -wi));
			float cosS = std::max(0.0f, Dot(shadingData.sNormal, wi));
			if (cosL <= 0.0f || cosS <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);
			//cal bsdf term
			Colour fr = shadingData.bsdf->evaluate(shadingData, wi);
			//cal geometry term
			float G = (cosS * cosL) / dist2;
			//cal light
			Colour contrib = Le * fr * G * (1.0f / (pdfA * pmf));
#if MIS
			float p_light = pdfA;
			float p_bsdf = shadingData.bsdf->PDF(shadingData, wi) * cosL / dist2;
			float w = p_light / (p_light + p_bsdf);
			contrib = contrib * w;
#endif
			//avoid bright colour
			contrib.r = std::min(contrib.r, FIREFLY_CLAMP);
			contrib.g = std::min(contrib.g, FIREFLY_CLAMP);
			contrib.b = std::min(contrib.b, FIREFLY_CLAMP);

			return contrib;
		}
		else {//materialscene, etc
			//return Colour(0.0f);
			float pdf = 0.0f;
			Colour Le(0.0f, 0.0f, 0.0f);
			Vec3 wi = light->sample(shadingData, sampler, Le, pdf);
			if (pdf <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);

			Ray shadowRay(shadingData.x + wi * EPSILON, wi);
			IntersectionData inter = scene->traverse(shadowRay);
			//hit an object
			if (inter.t < FLT_MAX) return Colour(0.0f, 0.0f, 0.0f);

			float cosS = std::max(0.0f, Dot(shadingData.sNormal, wi));
			if (cosS <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);
			//cal bsdf term
			Colour fr = shadingData.bsdf->evaluate(shadingData, wi);
			//cal geometry term
			float G = cosS;
			//cal light
			pmf = 1.0f;//for now
			Colour contrib = Le * fr * (G / (pdf * pmf));
#if MIS
			float p_light = pdf;
			float p_bsdf = shadingData.bsdf->PDF(shadingData, wi);
			float w = p_light / (p_light + p_bsdf);
			contrib = contrib * w;
#endif
			//avoid bright colour
			contrib.r = std::min(contrib.r, FIREFLY_CLAMP);
			contrib.g = std::min(contrib.g, FIREFLY_CLAMP);
			contrib.b = std::min(contrib.b, FIREFLY_CLAMP);
			return contrib;
		}
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		Colour L(0.0f);
		//intersect scene
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t == FLT_MAX){
			Colour returnLight = pathThroughput * scene->background->evaluate(r.dir);//in MaterialScene, background == environment
			returnLight.r = std::min(returnLight.r, FIREFLY_CLAMP);//avoid white dot.
			returnLight.g = std::min(returnLight.g, FIREFLY_CLAMP);
			returnLight.b = std::min(returnLight.b, FIREFLY_CLAMP);
			return returnLight;
		}
		//build shading data
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		//hit a light, add emission
		if (shadingData.bsdf && shadingData.bsdf->isLight()){
			L += pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
		}
		L += pathThroughput * computeDirect(shadingData, sampler);

		//max depth
		const int MAX_DEPTH = 8;
		if (depth >= MAX_DEPTH)
			return L;
		//russian roulette
		float maxComp = std::max(std::max(pathThroughput.r, pathThroughput.g), pathThroughput.b);
		float rrProb = std::min(maxComp, 0.95f);
		if (rrProb <= 0.0f)
			return L;
		if (sampler->next() > rrProb)
			return L;

		// scale throughput for unbiased estimator
		Colour throughput = pathThroughput / rrProb;

		if (shadingData.bsdf && shadingData.bsdf->isPureSpecular()){//glass mirror
			Colour reflectedColour(0.0f);
			float specPdf = 0.0f;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, reflectedColour, specPdf);

			// if sample failed or returned zero direction, stop
			if (wi.isZero()) {
				//shadingData.bsdf->printBSDFName();
				//return Colour(1.0f);
				return L;
			}

			// reflectedColour
			Colour nextThroughput = throughput * reflectedColour;

			nextThroughput.r = std::min(nextThroughput.r, FIREFLY_CLAMP);//avoid white dot.
			nextThroughput.g = std::min(nextThroughput.g, FIREFLY_CLAMP);
			nextThroughput.b = std::min(nextThroughput.b, FIREFLY_CLAMP);

			//spawn next ray and continue
			Ray nextRay(shadingData.x + wi * EPSILON, wi);
			L += pathTrace(nextRay, nextThroughput, depth + 1, sampler);
			return L;
		}
		else{//others
			Colour bsdfSampleCol(0.0f);
			float pdf = 0.0f;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfSampleCol, pdf);

			if (wi.isZero() || pdf <= 0.0f)
				return L;

			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			float cosTheta = std::max(0.0f, Dot(wi, shadingData.sNormal));
			Colour contribution = f * (cosTheta / pdf);

			Colour nextThroughput = throughput * contribution;

			nextThroughput.r = std::min(nextThroughput.r, FIREFLY_CLAMP);
			nextThroughput.g = std::min(nextThroughput.g, FIREFLY_CLAMP);
			nextThroughput.b = std::min(nextThroughput.b, FIREFLY_CLAMP);

			Ray nextRay(shadingData.x + wi * EPSILON, wi);
#if MIS
			IntersectionData nextIsect = scene->traverse(nextRay);
			if (nextIsect.t < FLT_MAX) {
				ShadingData lightSd = scene->calculateShadingData(nextIsect, nextRay);
				//wheither it hits a light
				if (lightSd.bsdf && lightSd.bsdf->isLight()) {
					Vec3 lightPos = nextRay.at(nextIsect.t);
					Vec3 dir = lightPos - shadingData.x;
					float dist2 = (lightPos - shadingData.x).lengthSq();
					float dist = sqrt(dist2);
					Vec3 wi_next = dir / dist;
					//triangle area
					float pdfA = 1.0f / std::max(EPSILON, scene->triangles[(int)nextIsect.ID].area);
					Vec3 lightN = scene->triangles[(int)nextIsect.ID].gNormal();
					float cosL_next = std::max(0.0f, Dot(lightN, -wi_next));
					//to omega
					float pdf_omega = pdfA * dist2 / cosL_next;
					float pmf = 1.0f / scene->lights.size();
					float pdf_light = pdf_omega * pmf;
					//cal bsdf pdf
					float pdf_bsdf = shadingData.bsdf->PDF(shadingData, wi);
					//cal weight
					float w = pdf_bsdf / (pdf_bsdf + pdf_light);
					//get Le
					Colour Le = lightSd.bsdf->emit(lightSd, -wi);
					L += nextThroughput * Le * w;
				}
			}
#endif
			L += pathTrace(nextRay, nextThroughput, depth + 1, sampler);
			return L;
		}
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	
#if RADIOSITY
	void lightTrace(Sampler* sampler, const float& invLightPathCount) {
		//sample a light source
		float pmf = 1.0f;//pmf for this light
		Light* light = scene->sampleLight(sampler, pmf);
		Colour pathThroughput(1.0f);
		if (light->isArea()) {//sample an area light
			float pdfP, pdfD;
			//sample point from the light source
			Vec3 p = light->samplePositionFromLight(sampler, pdfP);
			//sample direction from the light source
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfD);
			wi = wi.normalize();
			//get normal
			Vec3 n = light->normal(ShadingData(), Vec3());
			//connect postion to camera
			Colour col = light->evaluate(-wi);
			col = col / (pdfP * pmf * pdfD);
			//col = col / (pdfP );
			col = col * invLightPathCount;
			//starting a light path
			Ray ray(p, p + wi);
			generateVPLs(ray, pathThroughput, col, sampler, 0);
		}
	}
	void generateVPLs(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		//intersect scene
		IntersectionData isect = scene->traverse(r);
		if (isect.t == FLT_MAX) return;
		//shadingdata
		ShadingData shadingData = scene->calculateShadingData(isect, r);
		//create vpl and add to list
		vplList.emplace_back(shadingData, Le * pathThroughput);
		//if (depth == 5) std::cout << "Le * pathThroughput" << (Le * pathThroughput).r << " " << (Le * pathThroughput).g << " " << (Le * pathThroughput).b << std::endl;
		//stop
		const int MAX_DEPTH = 8;
		if (depth >= MAX_DEPTH) return;
		//sample
		float pdf;
		Colour reflectColour;
		Vec3 wo = shadingData.bsdf->sample(shadingData, sampler, reflectColour, pdf);
		if (wo.isZero() || pdf <= 0.0f) return;
		//geometry term
		float cosTheta = std::max(0.f, wo.dot(shadingData.sNormal));
		Colour nextThroughput = pathThroughput * reflectColour * cosTheta / pdf;
		//next ray
		float maxComp = std::max({nextThroughput.r, nextThroughput.g, nextThroughput.b });
		float rrProb = std::min(maxComp, 0.95f);
		if (sampler->next() > rrProb) return;
		nextThroughput /= rrProb;

		Ray nextRay(shadingData.x + wo * EPSILON, wo);
		generateVPLs(nextRay, nextThroughput, Le, sampler, depth + 1);
	}
	Colour calculateVPLs(Ray& ray, int startindex = 0) {
		//intersect scene
		IntersectionData isect = scene->traverse(ray);
		if (isect.t == FLT_MAX)
			return scene->background->evaluate(ray.dir);
		ShadingData shadingData = scene->calculateShadingData(isect, ray);
		Colour L(0.0f);

		if (shadingData.bsdf && shadingData.bsdf->isLight()) {
			L += shadingData.bsdf->emit(shadingData, shadingData.wo);
			return L;
		}

		const float STRIDE = 200;
		for (int i = startindex; i < vplList.size(); i += (int)STRIDE) {
			Vec3 wi = vplList[i].shadingData.x - shadingData.x;
			float dist2 = wi.lengthSq();
			float dist = sqrt(dist2);
			wi = wi.normalize();
			//geometry term
			float cosSurface = std::max(0.0f, Dot(shadingData.sNormal, wi));
			float cosVPL = std::max(0.0f, Dot(vplList[i].shadingData.sNormal, -wi));
			if (cosSurface > 0.0f && cosVPL > 0.0f) {
				//vpl visible
				if (scene->visible(shadingData.x + shadingData.sNormal * EPSILON,
					vplList[i].shadingData.x + vplList[i].shadingData.sNormal * EPSILON)) {
						Colour fr = shadingData.bsdf->evaluate(shadingData, wi);
						L += fr * vplList[i].Le * cosSurface * cosVPL / dist2 * STRIDE;
				}
			}
		}
		return L;
	}
#else
	//light tracing
	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		float posX, posY;
		//check position
		if (!scene->camera.projectOntoCamera(p, posX, posY)) return;
		//construct wi (position to camera), get distance
		Vec3 wi = scene->camera.origin - p;
		float dist = wi.length();
		wi = wi.normalize();
		//visible
		if (!scene->visible(p, scene->camera.origin)) return;
		//geometry term
		float cosSurface = n.dot(wi);
		if (cosSurface <= 0.0f)	return;
		float cosTheta = scene->camera.viewDirection.dot(-wi);
		if (cosTheta <= 0.0f)	return;
		float we = 1.0f / (scene->camera.Afilm * std::pow(cosTheta, 4));
		Colour contrib = col * cosSurface * we / SQ(dist);
		//splat onto film
		film->splat(posX, posY, contrib);
	}
	void lightTrace(Sampler* sampler, const float& invLightPathCount) {
		//sample a light source
		float pmf = 1.0f;//pmf for this light
		Light* light = scene->sampleLight(sampler, pmf);
		Colour pathThroughput(1.0f);
		if (light->isArea()) {//sample an area light
			float pdfP, pdfD;
			//sample point from the light source
			Vec3 p = light->samplePositionFromLight(sampler, pdfP);
			//sample direction from the light source
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfD);
			wi = wi.normalize();
			//get normal
			Vec3 n = light->normal(ShadingData(), Vec3());
			//connect postion to camera
			//Colour col = light->evaluate(-wi) / (pdfP);
			Colour col = light->evaluate(-wi);
			//std::cout << col.r << " " << col.g << " " << col.b << std::endl;
			col = col / (pdfP * pmf * pdfD);
			//std::cout << col.r << " " << col.g << " " << col.b << std::endl;
			//connect to camera
			connectToCamera(p, n, col);
			//starting a light path
			Ray ray(p, p + wi);
			lightTracePath(ray, pathThroughput, col, sampler, 0);
		}
	}
	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		//intersect scene
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t == FLT_MAX) return;
		//shading data
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		//depth
		const int MAX_DEPTH = 8;
		if (depth >= MAX_DEPTH) return;
		//russian roulette
		float maxComp = std::max(std::max(pathThroughput.r, pathThroughput.g), pathThroughput.b);
		float rrProb = std::min(maxComp, 0.95f);
		if (rrProb <= 0.0f) return;
		if (sampler->next() > rrProb) return;
		pathThroughput /= rrProb;
		//sampling bsdf
		float pdf;
		Colour reflectColour;
		Vec3 wo = shadingData.bsdf->sample(shadingData, sampler, reflectColour, pdf);
		if (wo.isZero() || pdf <= 0.0f) return;
		float cosTheta = std::max(0.0f, Dot(shadingData.sNormal, wo));
		//update throughput
		Colour nextThroughput = pathThroughput * reflectColour * cosTheta / pdf;

		nextThroughput.r = std::min(nextThroughput.r, FIREFLY_CLAMP);
		nextThroughput.g = std::min(nextThroughput.g, FIREFLY_CLAMP);
		nextThroughput.b = std::min(nextThroughput.b, FIREFLY_CLAMP);

		if (shadingData.bsdf && !shadingData.bsdf->isPureSpecular())
			connectToCamera(shadingData.x, shadingData.sNormal, nextThroughput * Le);

		//next ray
		Ray nextRay(shadingData.x + wo * EPSILON, wo);
		lightTracePath(nextRay, nextThroughput, Le, sampler, depth + 1);
	}
#endif
	
#if LIGHTTRACING
	void render() {
		film->incrementSPP();
		const int lightPathCount = film->width * film->height / 2;
		const float invLightPathCount = 1.0f / lightPathCount;

		Sampler* sampler = &samplers[0];
		for (int i = 0; i < lightPathCount; ++i) {
			lightTrace(sampler, invLightPathCount);
			//if ((lightPathCount - i) % 1000 == 0)	std::cout << "remain:" << lightPathCount - i << std::endl;
		}
#if RADIOSITY
#if THREADPOOL //THREADPOOL
		static const unsigned int tileSize = 32;
		std::vector<std::pair<int, int>> tiles;
		const int tilesX = (film->width + tileSize - 1) / tileSize;
		const int tilesY = (film->height + tileSize - 1) / tileSize;
		tiles.reserve(tilesX * tilesY);
		for (int y = 0; y < film->height; y += tileSize) {//tile splitting
			for (int x = 0; x < film->width; x += tileSize) {
				tiles.emplace_back(x, y);
			}
		}
		for (auto& tile : tiles) {
			unsigned int w = std::min(tileSize, film->width - tile.first);
			unsigned int h = std::min(tileSize, film->height - tile.second);
			threadpool->submit([this, tile, w, h]() { 
				size_t tidHash = std::hash<std::thread::id>()(std::this_thread::get_id());
				Sampler* sampler = &samplers[tidHash % numProcs];
				
				//std::cout << "startindex = " << startindex << std::endl;
				for (unsigned int y = tile.second; y < tile.second + h; y++) {
					for (unsigned int x = tile.first; x < tile.first + w; x++) {
						int startindex = 200 * sampler->next();
						float px = x + 0.5f;
						float py = y + 0.5f;
						Ray ray = scene->camera.generateRay(px, py);
						Colour col = calculateVPLs(ray, startindex);
						film->splat(px, py, col);
						//std::cout << "remain:" << film->width - x << std::endl;
					}
				}
				std::cout << "complete one tile" << std::endl;
				});
		}
		threadpool->waitForTasks();
#else
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour col = calculateVPLs(ray);
				film->splat(px, py, col);
				std::cout << "remain:" << film->width - x << std::endl;
			}
			
		}
#endif
		std::cout << "frame finished" << std::endl;
#endif
		
#if DENOISE
		//denoising
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					size_t p = x + y * film->width;
					Colour hdr = film->film[p] / float(film->SPP);
					colourPtr[p * 3 + 0] = hdr.r;
					colourPtr[p * 3 + 1] = hdr.g;
					colourPtr[p * 3 + 2] = hdr.b;
				}
			}
		}
		filter.execute();
		//draw
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					int idx = (x + y * film->width) * 3;
					Colour col(outputPtr[idx + 0], outputPtr[idx + 1], outputPtr[idx + 2]);

					film->tonemap(x, y, col, 2.0f);
					unsigned char rc = (unsigned char)(std::min(1.0f, col.r) * 255.0f);
					unsigned char gc = (unsigned char)(std::min(1.0f, col.g) * 255.0f);
					unsigned char bc = (unsigned char)(std::min(1.0f, col.b) * 255.0f);
					canvas->draw(x, y, rc, gc, bc);
				}
			}
		}

#else
		//draw
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					int p = (x + y * film->width);
					Colour col = film->film[p] / float(film->SPP);
#if RADIOSITY
					film->tonemap(x, y, col, 2.0f);//2.0f
#else
					film->tonemap(x, y, col, 20.0f);//2.0f
#endif
					unsigned char rc = (unsigned char)(std::min(1.0f, col.r) * 255.0f);
					unsigned char gc = (unsigned char)(std::min(1.0f, col.g) * 255.0f);
					unsigned char bc = (unsigned char)(std::min(1.0f, col.b) * 255.0f);
					canvas->draw(x, y, rc, gc, bc);
				}
			}
		}
#endif
	}
#else
	void renderTile(int x0, int y0, int tileX, int tileY) {
		size_t tidHash = std::hash<std::thread::id>()(std::this_thread::get_id());
		Sampler* sampler = &samplers[tidHash % numProcs];
		for (unsigned int y = y0; y < y0 + tileY; y++){
			for (unsigned int x = x0; x < x0 + tileX; x++){
				// use pixel center for splat as film expects sample location
				Ray ray = scene->camera.generateRay(x, y);
				size_t idx = x + y * film->width;
				Colour throughput(1.0f, 1.0f, 1.0f);
				Colour col = pathTrace(ray, throughput, 0, sampler);
				film->splat(x + 0.5f, y + 0.5f, col);

			}
		}
	}
#if THREADPOOL
	void render()
	{
		film->incrementSPP();
		static const unsigned int tileSize = 32;
		std::vector<std::pair<int, int>> tiles;
		const int tilesX = (film->width + tileSize - 1) / tileSize;
		const int tilesY = (film->height + tileSize - 1) / tileSize;
		tiles.reserve(tilesX * tilesY);
		for (int y = 0; y < film->height; y += tileSize) {//tile splitting
			for (int x = 0; x < film->width; x += tileSize) {
				tiles.emplace_back(x, y);
			}
		}
		for (auto& tile : tiles) {
			unsigned int w = std::min(tileSize, film->width - tile.first);
			unsigned int h = std::min(tileSize, film->height - tile.second);
			threadpool->submit([this, tile, w, h]() { renderTile(tile.first, tile.second, w, h); });
		}
		threadpool->waitForTasks();
		
#if DENOISE
		//denoising
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					size_t p = x + y * film->width;
					Colour hdr = film->film[p] / float(film->SPP);
					colourPtr[p * 3 + 0] = hdr.r;
					colourPtr[p * 3 + 1] = hdr.g;
					colourPtr[p * 3 + 2] = hdr.b;
				}
			}
		}
		filter.execute();
		//draw
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					int idx = (x + y * film->width) * 3;
					Colour col(outputPtr[idx + 0], outputPtr[idx + 1], outputPtr[idx + 2]);

					film->tonemap(x, y, col, 2.0f);
					unsigned char rc = (unsigned char)(std::min(1.0f, col.r) * 255.0f);
					unsigned char gc = (unsigned char)(std::min(1.0f, col.g) * 255.0f);
					unsigned char bc = (unsigned char)(std::min(1.0f, col.b) * 255.0f);
					canvas->draw(x, y, rc, gc, bc);
				}
			}
		}
		
#else
		//draw
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					int p = (x + y * film->width);
					Colour col = film->film[p] / float(film->SPP);
					film->tonemap(x, y, col, 2.0f);//2.0f
					unsigned char rc = (unsigned char)(std::min(1.0f, col.r) * 255.0f);
					unsigned char gc = (unsigned char)(std::min(1.0f, col.g) * 255.0f);
					unsigned char bc = (unsigned char)(std::min(1.0f, col.b) * 255.0f);
					canvas->draw(x, y, rc, gc, bc);
				}
			}
		}
#endif
	}
#else
	void render()
	{
		film->incrementSPP();
		static const unsigned int tileSize = 32;
		std::vector<std::pair<int, int>> tiles;
		const int tilesX = (film->width + tileSize - 1) / tileSize;
		const int tilesY = (film->height + tileSize - 1) / tileSize;
		tiles.reserve(tilesX * tilesY);
		for (int y = 0; y < film->height; y += tileSize) {//tile splitting
			for (int x = 0; x < film->width; x += tileSize) {
				tiles.emplace_back(x, y);
			}
		}
		for (int i = 0; i < numProcs; i++) {//create threads by using new(this approach needs creates threads for each frame, can be optimized by using pool)
			threads[i] = new std::thread([this, i, &tiles]() {//lambda function
				for (int t = i; t < tiles.size(); t += numProcs) {
					std::pair<int, int> tile = tiles[t];//tile = (x0, y0)
					int w = std::min(tileSize, film->width - tile.first);
					int h = std::min(tileSize, film->height - tile.second);
					renderTile(tile.first, tile.second, w, h);
				}
			});
		}
		for (int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
		}
#if DENOISE
		//denoising
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					size_t p = x + y * film->width;
					Colour hdr = film->film[p] / float(film->SPP);
					colourPtr[p * 3 + 0] = hdr.r;
					colourPtr[p * 3 + 1] = hdr.g;
					colourPtr[p * 3 + 2] = hdr.b;
				}
			}
		}
		filter.execute();
		//draw
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					int idx = (x + y * film->width) * 3;
					Colour col(outputPtr[idx + 0], outputPtr[idx + 1], outputPtr[idx + 2]);

					film->tonemap(x, y, col, 2.0f);
					unsigned char rc = (unsigned char)(std::min(1.0f, col.r) * 255.0f);
					unsigned char gc = (unsigned char)(std::min(1.0f, col.g) * 255.0f);
					unsigned char bc = (unsigned char)(std::min(1.0f, col.b) * 255.0f);
					canvas->draw(x, y, rc, gc, bc);
				}
			}
		}

#else
		//draw
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				if (film->SPP > 0) {
					int p = (x + y * film->width);
					Colour col = film->film[p] / float(film->SPP);
					film->tonemap(x, y, col, 2.0f);
					unsigned char rc = (unsigned char)(std::min(1.0f, col.r) * 255.0f);
					unsigned char gc = (unsigned char)(std::min(1.0f, col.g) * 255.0f);
					unsigned char bc = (unsigned char)(std::min(1.0f, col.b) * 255.0f);
					canvas->draw(x, y, rc, gc, bc);
				}
			}
		}
#endif

	}
#endif
#endif
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};