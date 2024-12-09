#pragma once
#include "stdafx.hpp"

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

struct STRaTAHitReturn
{
	rtm::Hit hit;
	uint32_t index;		// id for framebuffer
};
