#pragma once
#include "stdafx.hpp"

#define TRAX_USE_RT_CORE 0
#define TRAX_USE_HARDWARE_INTERSECTORS 0
#define TRAX_USE_COMPRESSED_WIDE_BVH 1

#define TRAX_KERNEL_ARGS_ADDRESS 256ull

struct TRaXKernelArgs
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;
	uint32_t* framebuffer;

	bool pregen_rays;

	rtm::Camera camera;
	rtm::vec3 light_dir;
	rtm::Ray* rays;
	//rtm::BVH2::Node* nodes;
	rtm::WBVH::Node* nodes;
	//rtm::NVCWBVH::Node* nodes;
	//rtm::HECWBVH::Node* nodes;
	rtm::Triangle* tris;
	rtm::TriangleStrip* strips;
};
