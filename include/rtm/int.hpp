#pragma once

#include <cstdint>

typedef unsigned int uint;

inline uint f_to_u(float f)
{
	return *(uint*)&f;
}

inline float u_to_f(uint u)
{
	return *(float*)&u;
}