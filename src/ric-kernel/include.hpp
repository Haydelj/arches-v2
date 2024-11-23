#pragma once
#include "stdafx.hpp"

#define RIC_KERNEL_ARGS_ADDRESS 256ull

struct RICKernelArgs
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;

	bool pregen_rays;

	rtm::Camera camera;
	rtm::vec3 light_dir;

	//heap data pointers
	uint32_t* framebuffer;
	rtm::Hit* hit_records;
	rtm::Ray* rays;
	rtm::CompressedWideTreeletBVH::Treelet* treelets;
	rtm::Triangle* tris;
	uint64_t num_treelets;
};