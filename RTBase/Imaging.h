#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;//0
	}
};
class GaussianFilter : public ImageFilter
{
private:
	float m_radius;
	float m_alpha;
	float m_expRadius;

public:
	GaussianFilter(float radius = 1.0f, float alpha = 2.0f)
		: m_radius(radius),
		m_alpha(alpha),
		m_expRadius(std::exp(-alpha * radius * radius))
	{
	}
	float filter(const float x, const float y) const override
	{
		if (fabsf(x) > m_radius || fabsf(y) > m_radius)
			return 0.0f;

		float d2 = x * x + y * y;
		return std::exp(-m_alpha * d2) - m_expRadius;
	}

	int size() const override
	{
		return int(std::ceil(m_radius));
	}
};
class MitchellNetravaliFilter : public ImageFilter
{
private:
	float m_radius;
	float m_B;
	float m_C;

	static float hm1d(float x, float B, float C)
	{
		float ax = fabsf(x);
		if (ax >= 2.0f) return 0.0f;
		float ax2 = ax * ax;
		float ax3 = ax2 * ax;
		const float inv6 = 1.0f / 6.0f;
		if (ax < 1.0f)
		{
			return inv6 * ((12.0f - 9.0f * B - 6.0f * C) * ax3
				+ (-18.0f + 12.0f * B + 6.0f * C) * ax2
				+ (6.0f - 2.0f * B));
		}
		else // 1 <= ax < 2
		{
			return inv6 * ((-B - 6.0f * C) * ax3
				+ (6.0f * B + 30.0f * C) * ax2
				+ (-12.0f * B - 48.0f * C) * ax
				+ (8.0f * B + 24.0f * C));
		}
	}

public:
	MitchellNetravaliFilter(float radius = 2.0f, float B = 1.0f / 3.0f, float C = 1.0f / 3.0f)
		: m_radius(radius), m_B(B), m_C(C)
	{}

	float filter(const float x, const float y) const override
	{
		float wx = hm1d(x, m_B, m_C);
		float wy = hm1d(y, m_B, m_C);
		return wx * wy;
	}

	int size() const override
	{
		return int(std::ceil(m_radius));
	}
};
class Film
{
public:
	Colour* film;
	//float* weight;
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;
	//example
	void splat(const float x, const float y, const Colour& L) {
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					//std::cout << "weight = " << filterWeights[used] << std::endl;
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
			//std::cout << '(' << film[indices[i]].r << ',' << film[indices[i]].g << ',' << film[indices[i]].b << ')' << std::endl;
		}
	}

	float tonemap_filmic(float x) const {
		//from lecture
		static const float A = 0.15f;
		static const float B = 0.5f;
		static const float C = 0.1f;
		static const float D = 0.2f;
		static const float E = 0.02f;
		static const float F = 0.3f;
		static const float W = 11.2f;
		return (std::max(0.0f, std::min(1.0f, ((x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F)) - E / F)));
	}
#if 1
	void tonemap(int x, int y, Colour& col, float exposure = 1.0f)
	{
		// Apply exposure and filmic tonemap + gamma
		Colour hdr = col * exposure;

		auto cal = [this](float v, float invGamma) {
			float filmic = std::max(0.f, std::min(1.f, tonemap_filmic(v)));
			return std::max(0.0f, std::min(1.0f, powf(filmic, invGamma)));
			};
		col = Colour(cal(hdr.r, 1.0f / 2.2f), cal(hdr.g, 1.0f / 2.2f), cal(hdr.b, 1.0f / 2.2f));
	}
#else
	void tonemap(int x, int y, Colour& col, float exposure = 1.0f)
	{
		int idx = y * width + x;
		if (SPP <= 0)
		{
			col = Colour(0.0f);
			return;
		}

		// use pixel-centre as sample location
		const float sampleX = x + 0.5f;
		const float sampleY = y + 0.5f;

		Colour hdrAccum(0.0f);
		float totalW = 0.0f;

		if (filter != nullptr)
		{
			int fs = filter->size();
			for (int i = -fs; i <= fs; ++i)
			{
				for (int j = -fs; j <= fs; ++j)
				{
					int px = x + j;
					int py = y + i;
					if (px < 0 || px >= (int)width || py < 0 || py >= (int)height) continue;
					float w = filter->filter((float)px - sampleX, (float)py - sampleY);
					if (w <= 0.0f) continue;
					int nidx = py * width + px;
					hdrAccum = hdrAccum + (film[nidx] / float(SPP)) * w;
					totalW += w;
				}
			}
			if (totalW > 0.0f)
				hdrAccum = hdrAccum / totalW;
			else
				hdrAccum = film[idx] / float(SPP);
		}
		else
		{
			hdrAccum = film[idx] / float(SPP);
		}
		// Apply exposure and filmic tonemap + gamma
		Colour hdr = hdrAccum * exposure;

		auto cal = [this](float v, float invGamma) {
			float filmic = std::max(0.f, std::min(1.f, tonemap_filmic(v)));
			return std::max(0.0f, std::min(1.0f, powf(filmic, invGamma)));
			};
		col = Colour(cal(hdr.r, 1.0f / 2.2f), cal(hdr.g, 1.0f / 2.2f), cal(hdr.b, 1.0f / 2.2f));
	}
#endif
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		clear();
		filter = _filter;
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};