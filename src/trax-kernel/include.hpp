#pragma once
#include "stdafx.hpp"

#define USE_RT_CORE
#define USE_HARDWARE_INTERSECTORS

#define KERNEL_ARGS_ADDRESS 256ull
struct KernelArgs
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;
	uint32_t* framebuffer;

	uint32_t samples_per_pixel;
	uint32_t max_depth;

	rtm::vec3 light_dir;
	rtm::PackedBVH2::Node* nodes;
	rtm::Triangle* tris;
	rtm::PackedTreelet* treelets;
	rtm::Camera camera;
};
