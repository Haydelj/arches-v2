#pragma once

#include "int.hpp"

namespace rtm 
{

class uvec2
{
public:
	uint32_t e[2];

	uvec2() = default;
	uvec2(uint32_t i) { e[0] = i;  e[1] = i; }
	uvec2(uint32_t i, uint32_t j) { e[0] = i; e[1] = j; }

	uint32_t operator[](int i) const { return e[i]; }
	uint32_t& operator[](int i) { return e[i]; }
};

}