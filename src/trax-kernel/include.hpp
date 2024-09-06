#pragma once
#include "stdafx.hpp"

#define USE_RT_CORE
#define USE_HARDWARE_INTERSECTORS

#define KERNEL_ARGS_ADDRESS 256ull

struct TRaXKernelArgs
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;
	uint32_t* framebuffer;

	bool pregen_rays;

	rtm::Camera camera;
	rtm::vec3 light_dir;
#if defined (WIDE_COMPRESSED_BVH)
	rtm::WideBVH::WideBVHNode* nodes;
#else
	rtm::PackedBVH2::Node* nodes;
#endif

	rtm::Triangle* tris;
	rtm::PackedTreelet* treelets;
	rtm::Ray* rays;
};