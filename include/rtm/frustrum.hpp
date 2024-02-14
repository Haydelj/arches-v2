#pragma once

#include "vec3.hpp"

namespace rtm
{

struct Frustrum
{
	rtm::vec3 o;
	float     t_min;
	rtm::vec3 d;
	float     t_max;
	rtm::vec3 dx;
	rtm::vec3 dy;
};

}