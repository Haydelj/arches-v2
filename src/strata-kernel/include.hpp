#pragma once
#include "stdafx.hpp"

namespace STRaTAKernel {

#define USE_RT_CORE
#define USE_HARDWARE_INTERSECTORS
#define USE_COMPRESSED_WIDE_BVH

#define KERNEL_ARGS_ADDRESS 256ull

struct alignas(64) RayData
{
	rtm::Ray ray; //32
	rtm::Hit hit; //16

	uint32_t treelet_id : 12; //4
	uint32_t node_id : 20;
	uint32_t global_ray_id : 28; //4
	enum class Traversal_State : uint32_t
	{
		NONE,
		DOWN,
		UP,
		LEAF_STAY,
		OVER,
	} traversal_state : 4;

	uint64_t visited_stack{0}; //8 //3 per level 21 levels
};

struct HitReturn
{
	rtm::Hit hit;
	uint32_t index;		// id for framebuffer
};

struct Args
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
	rtm::CompressedWideTreeletBVHSTRaTA::Treelet* treelets;
#else
	rtm::WideTreeletBVHSTRaTA::Treelet* treelets;
#endif
	rtm::Triangle* tris;
	rtm::Hit* hit_records;
};

}