#pragma once
#include "stdafx.hpp"

#include "intersect.hpp"

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
	MeshPointers mesh;
	rtm::Camera camera;
};
