#pragma once

#include "Core.h"
#include "Sampling.h"
class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t)
	{
		t =  -(d + Dot(n, r.o)) / Dot(n, r.dir);
		return t > 0;
	}
};



class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float invArea; 
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		invArea = 1.0f / Dot(e1.cross(e2), n);//for rayIntersect
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }
		Vec3 p = r.at(t);
		//float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;
	}

	
	//Moller - Trumbore
	float det(const Vec3& a, const Vec3& b, const Vec3& c) const
	{
		return
			a.x * (b.y * c.z - b.z * c.y) -
			a.y * (b.x * c.z - b.z * c.x) +
			a.z * (b.x * c.y - b.y * c.x);
	}
	bool rayIntersect_MT(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 T = r.o - vertices[0].p;
		float detA = det(e1, e2, -r.dir);
		float detB = det(T, e2, -r.dir);
		float detC = det(e1, T, -r.dir);
		float detD = det(e1, e2, T);
		u = detB / detA;
		v = detC / detA;
		t = detD / detA;
		if (t < 0) {return false;}
		if (u < 0 || u > 1.0f) { return false; }
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		float alpha = 1.0f - sqrtf(r1);
		float beta = sqrtf(r1) * r2;
		float gamma = 1.0f - (alpha + beta);
		// pdf
		pdf = invArea;
		//pdf = 1.0f / area;
		Vec3 samplePoint = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;
		return samplePoint;
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	void extend(const AABB& b)
	{
		max = Max(max, b.max);
		min = Min(min, b.min);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)//check whether the ray intersacts with this AABB(get t)
	{
		Vec3 Tmin = (min - r.o) * r.invDir;
		Vec3 Tmax = (max - r.o) * r.invDir;
		Vec3 Tentry = Min(Tmin, Tmax);
		Vec3 Texit = Max(Tmin, Tmax);
		float tentry = std::max(std::max(Tentry.x, Tentry.y), Tentry.z);
		float texit = std::min(std::min(Texit.x, Texit.y), Texit.z);
		t = std::min(tentry, texit);
		return (tentry <= texit && texit > 0);
	}
	// Add code here
	bool rayAABB(const Ray& r)//check whether the ray intersacts with this AABB
	{
		Vec3 s = (min - r.o) * r.invDir;
		Vec3 l = (max - r.o) * r.invDir;
		Vec3 s1 = Min(s, l);
		Vec3 l1 = Max(s, l);
		float ts = std::max(std::max(s1.x, s1.y), s1.z);
		float tl = std::min(std::min(l1.x, l1.y), l1.z);
		return (ts <= tl && tl > 0);
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	unsigned int offset;
	unsigned int num;
	// ---- Bins ----
	struct Bin
	{
		AABB box;
		int count = 0;
	};
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& outputTriangles)
	{
		// Add BVH building code here
		bounds.reset();//initialize bounds
		for (auto& tri : inputTriangles)//extend bounds to include all triangles
			for (int i = 0; i < 3; i++)
				bounds.extend(tri.vertices[i].p);

		if (inputTriangles.size() <= MAXNODE_TRIANGLES)//leaf nodes
		{

			offset = outputTriangles.size();
			num = inputTriangles.size();

			for (auto& tri : inputTriangles)// Push triangles into global output
				outputTriangles.push_back(tri);
			return;
		}

		float bestCost = FLT_MAX;
		int bestAxis = -1;
		int bestSplitBucket = -1;

		// Loop over 3 axes
		for (int axis = 0; axis < 3; axis++)
		{
			float minC = FLT_MAX;
			float maxC = -FLT_MAX;
			for (auto& tri : inputTriangles)
			{
				float c = tri.centre().coords[axis];
				if (c < minC) minC = c;
				if (c > maxC) maxC = c;
			}

			float delta = maxC - minC;
			if (delta < 1e-6f)
				continue;  // all centres the same -> cannot split on this axis

			float invDelta = 1.0f / delta;

			

			Bin bins[BUILD_BINS];

			for (auto& tri : inputTriangles)
			{
				float c = tri.centre().coords[axis];
				int b = int(((c - minC) * invDelta) * BUILD_BINS);
				if (b == BUILD_BINS) b = BUILD_BINS - 1;

				bins[b].count++;
				for (int i = 0; i < 3; i++)
					bins[b].box.extend(tri.vertices[i].p);
				//bins[b].box.expand(tri.getAABB());
			}

			AABB prefixBox[BUILD_BINS];
			AABB suffixBox[BUILD_BINS];
			int prefixCnt[BUILD_BINS] = { 0 };
			int suffixCnt[BUILD_BINS] = { 0 };

			// build prefix
			prefixBox[0] = bins[0].box;
			prefixCnt[0] = bins[0].count;
			for (int i = 1; i < BUILD_BINS; i++)
			{
				prefixBox[i] = prefixBox[i - 1];
				prefixBox[i].extend(bins[i].box);
				prefixCnt[i] = prefixCnt[i - 1] + bins[i].count;
			}

			// build suffix
			suffixBox[BUILD_BINS - 1] = bins[BUILD_BINS - 1].box;
			suffixCnt[BUILD_BINS - 1] = bins[BUILD_BINS - 1].count;
			for (int i = BUILD_BINS - 2; i >= 0; i--)
			{
				suffixBox[i] = suffixBox[i + 1];
				suffixBox[i].extend(bins[i].box);
				suffixCnt[i] = suffixCnt[i + 1] + bins[i].count;
			}

			for (int i = 0; i < BUILD_BINS - 1; i++)
			{
				float leftArea = prefixBox[i].area();
				float rightArea = suffixBox[i + 1].area();

				int leftCount = prefixCnt[i];
				int rightCount = suffixCnt[i + 1];

				//float cost = TRAVERSE_COST * bounds.area() + TRIANGLE_COST * (leftArea * leftCount + rightArea * rightCount);
				float cost = TRAVERSE_COST + TRIANGLE_COST * (leftArea * leftCount + rightArea * rightCount) / bounds.area();//cost function

				if (cost < bestCost)
				{
					bestCost = cost;
					bestAxis = axis;
					bestSplitBucket = i;
				}
			}
		}
		//use best Axis and bestSplitBucket to split triangles
		std::vector<Triangle> leftTris, rightTris;

		float minC = FLT_MAX, maxC = -FLT_MAX;
		for (auto& tri : inputTriangles)
		{
			//float c = tri.centroid()[bestAxis];
			float c = tri.centre().coords[bestAxis];
			if (c < minC) minC = c;
			if (c > maxC) maxC = c;
		}

		float delta = maxC - minC;
		float invDelta = 1.0f / delta;

		for (auto& tri : inputTriangles)
		{
			//float c = tri.centroid()[bestAxis];
			float c = tri.centre().coords[bestAxis];
			int b = int(((c - minC) * invDelta) * BUILD_BINS);
			if (b == BUILD_BINS) b = BUILD_BINS - 1;

			if (b <= bestSplitBucket)
				leftTris.push_back(tri);
			else
				rightTris.push_back(tri);
		}

		//if all triangles fall into one bin, split them in the middle.
		if (leftTris.empty() || rightTris.empty())
		{
			size_t mid = inputTriangles.size() / 2;
			leftTris.insert(leftTris.end(), inputTriangles.begin(), inputTriangles.begin() + mid);
			rightTris.insert(rightTris.end(), inputTriangles.begin() + mid, inputTriangles.end());
		}

		l = new BVHNode();
		r = new BVHNode();

		l->build(leftTris, outputTriangles);
		r->build(rightTris, outputTriangles);


		//outputTriangles = std::move(inputTriangles);
		return;
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here

		BVHNode* stack[64];
		int sp = 0;
		stack[sp++] = this; // root
		while (sp > 0) {
			BVHNode* node = stack[--sp];
			float t = 0;
			if (node == nullptr)	continue;//break? try later
			if (node->bounds.rayAABB(ray, t) == false)	continue; // no intersection
			
			if (node->l == NULL && node->r == NULL) {// If leaf node, check triangle intersections
				for (unsigned int i = 0; i < node->num; ++i) {
					const Triangle& tri = triangles[node->offset + i];//binding a new name
					float t, alpha, beta;
					if(tri.rayIntersect(ray, t, alpha, beta) && t < intersection.t) {
						intersection.t = t;
						intersection.ID = node->offset + i;
						intersection.alpha = alpha;
						intersection.beta = beta;
						intersection.gamma = 1 - (alpha + beta);
					}
				}
				continue;
			}
			float t_left = FLT_MAX, t_right = FLT_MAX;
			bool hit_left = (node->l != NULL && node->l->bounds.rayAABB(ray, t_left));
			bool hit_right = (node->r != NULL && node->r->bounds.rayAABB(ray, t_right));
			if (hit_left && hit_right) {//hit both child nodes, push the closer one first
				if (t_left < t_right) {
					stack[sp++] = node->r;
					stack[sp++] = node->l;
				}
				else {
					stack[sp++] = node->l;
					stack[sp++] = node->r;
				}
			}
			else if(hit_left) {//only hit left child node
				stack[sp++] = node->l;
			}
			else if(hit_right) {//only hit right child node
				stack[sp++] = node->r;
			}
		}
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)//simple check
	{
		// Add visibility code here
		BVHNode* stack[64];
		int sp = 0;
		stack[sp++] = this; // root

		while (sp > 0) {
			BVHNode* node = stack[--sp];
			if (node == nullptr)	continue;//break? try later
			float t = 0;
			if (node->bounds.rayAABB(ray, t) == false || t > maxT)	continue; // no intersection or beyond maxT
			if (node->l == NULL && node->r == NULL) {
				for (unsigned int i = 0; i < node->num; ++i) {
					const Triangle& tri = triangles[node->offset + i];
					float t, alpha, beta;
					if (tri.rayIntersect(ray, t, alpha, beta) && t < maxT) {
						return false; // blocked return false
					}
				}
				continue;
			}
			float t_left = FLT_MAX, t_right = FLT_MAX;
			bool hit_left = (node->l != NULL && node->l->bounds.rayAABB(ray, t_left) && t_left <= maxT);
			bool hit_right = (node->r != NULL && node->r->bounds.rayAABB(ray, t_right) && t_right <= maxT);
			if (hit_left) {
				stack[sp++] = node->l;
			}
			if (hit_right) {
				stack[sp++] = node->r;
			}
		}
		return true;
	}
};
