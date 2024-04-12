#pragma once
#include "stdafx.hpp"

#define USE_RT_CORE
#define USE_HARDWARE_INTERSECTORS

#define KERNEL_ARGS_ADDRESS 256ull

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
	rtm::PackedTreelet* treelets;
	rtm::Triangle* tris;
	rtm::Ray* rays;
	uint64_t num_treelets;
};