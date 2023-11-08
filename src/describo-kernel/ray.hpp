#pragma once
#include "stdafx.hpp"

struct Ray
{
	rtm::vec3 o;
	float     t_min;
	rtm::vec3 d;
	float     t_max;
	
	float     drdt;
	float     radius;
	float     rcp_max_error;
};

struct Hit
{
	float t;
	rtm::vec2 bc;
	uint32_t id;

	uint32_t patch_index;
};
