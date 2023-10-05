#pragma once

#include "../../include/rtm/rtm.hpp"
#include "treelet-bvh.hpp"
#include "intersect.hpp"

struct RayWriteBuffer
{
	uint treelet;
	uint ray_id;
	rtm::Ray ray;
};

#define KERNEL_ARGS_ADDRESS 256ull

struct KernelArgs
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;

	uint32_t samples_per_pixel;
	float inverse_samples_per_pixel;
	uint32_t max_path_depth;

	rtm::Camera camera;
	rtm::vec3 light_dir;

	//heap data pointers
	uint32_t* framebuffer;
	rtm::Hit* hit_records;
	Treelet* treelets;
	rtm::Triangle* triangles;
};
