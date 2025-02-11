#pragma once
#include "stdafx.hpp"

#define TRAX_USE_RT_CORE 1
#define TRAX_USE_HARDWARE_INTERSECTORS 0
#define TRAX_USE_COMPRESSED_WIDE_BVH 1

#define KERNEL_ARGS_ADDRESS 256ull

namespace SGKernel {

struct Hit
{
	union
	{
		struct
		{
			uint32_t id : 24;
			uint32_t a: 8;
		};
		uint32_t u32;
	};
};

struct HitPacket
{
	const static uint MAX_HITS = 14;
	float last_t;
	uint size;
	Hit hits[MAX_HITS];
};

struct Args
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;
	uint32_t* framebuffer;

	bool pregen_rays;

	rtm::Camera camera;
	rtm::Ray* rays;
	rtm::CompressedWideBVH::Node* nodes;
	rtm::SphericalGaussian* sgs;
	rtm::SphericalHarmonic* shs;
};

}