#pragma once
#include "stdafx.hpp"

struct RayData
{
	rtm::Ray ray;
	struct RayState
	{
		uint32_t treelet_id{0};
		uint32_t treelet_child_id{0};
		uint32_t hit_id{~0u};
		float hit_t{T_MAX};
		uint32_t id;	// id for framebuffer
		enum class Traversal_State
		{
			NONE,
			DOWN,
			UP,
			OVER,
		} traversal_state;
	};
	RayState raystate;

	uint32_t traversal_stack{0};
	uint32_t visited_stack{0};
};

struct STRaTAHitReturn
{
	rtm::Hit hit;
	uint32_t index;		// id for framebuffer
};
