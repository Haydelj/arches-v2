#pragma once

#include "vec3.hpp"

#define T_MAX (1.0f * (1 << 20))
#define T_MIN (1.0f / (1 << 5))

namespace rtm
{

struct Ray
{
	rtm::vec3 o;
	float     t_min;
	rtm::vec3 d;
	float     t_max;
};

struct Frustum
{
	rtm::vec3 o;
	float     t_min;
	rtm::vec3 d;
	float     t_max;
	rtm::vec3 dx;
	rtm::vec3 dy;
};

struct Hit
{
	float t;
	rtm::vec2 bc;
	uint32_t id;

	Hit() = default;
	Hit(float t, rtm::vec2 bc, uint id) : t(t), bc(bc), id(id) {}
};

}