#pragma once
#include "stdafx.hpp"

namespace STRaTARTKernel {

struct alignas(64) RayData
{
	rtm::Ray ray; //32
	rtm::Hit hit; //16

	uint64_t global_ray_id : 26;
	uint64_t treelet_id : 20;
	uint64_t level : 6;
	uint64_t pf_mask : 8;

	rtm::RestartTrail restart_trail{}; //8 //3 per level 21 levels
};

struct HitReturn
{
	rtm::Hit hit;
	uint32_t index;		// id for framebuffer
};

}