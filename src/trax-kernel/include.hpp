#pragma once
#include "stdafx.hpp"

#include "intersect.hpp"

#define KERNEL_ARGS_ADDRESS 256ull

struct TRaXKernelArgs
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;
	uint32_t* framebuffer;

	uint32_t samples_per_pixel;
	uint32_t max_depth;

	bool use_trace_ray;
	bool use_secondary_rays;

	rtm::vec3 light_dir;
	MeshPointers mesh;
	rtm::Camera camera;

	rtm::Ray* secondary_rays;
};
