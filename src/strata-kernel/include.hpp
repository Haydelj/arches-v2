#pragma once
#include "stdafx.hpp"

#define USE_RT_CORE
#define USE_HARDWARE_INTERSECTORS
#define USE_COMPRESSED_WIDE_BVH

#define KERNEL_ARGS_ADDRESS 256ull

struct STRaTAKernelArgs
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;
	uint32_t* framebuffer;
	uint32_t raybuffer_size;
	uint32_t max_init_ray;

	bool pregen_rays;

	rtm::Camera camera;
	rtm::vec3 light_dir;
	rtm::Ray* rays;

#ifdef USE_COMPRESSED_WIDE_BVH
	rtm::CompressedWideTreeletBVH::Treelet* treelets;
#else
	rtm::WideTreeletBVH::Treelet* treelets;
#endif
	rtm::Triangle* tris;
	rtm::Hit* hit_records;
};
