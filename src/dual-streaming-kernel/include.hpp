#pragma once
#include "stdafx.hpp"

#define DS_USE_RT_CORE 1
#define DS_USE_HARDWARE_INTERSECTORS 1
#define DS_USE_COMPRESSED_WIDE_BVH 1

#define DS_KERNEL_ARGS_ADDRESS 256ull

struct DualStreamingKernelArgs
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;

	bool use_early;
	bool hit_delay;
	bool pregen_rays;

	rtm::Camera camera;
	rtm::vec3 light_dir;

	//heap data pointers
	uint32_t* framebuffer;
	rtm::Hit* hit_records;
#if DS_USE_COMPRESSED_WIDE_BVH
	rtm::CompressedWideTreeletBVH::Treelet* treelets;
#else
	rtm::WideTreeletBVH::Treelet* treelets;
#endif
	rtm::Triangle* tris;
	rtm::Ray* rays;
	uint64_t num_treelets;
};