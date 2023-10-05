#pragma once

#include "int.hpp"

namespace rtm 
{

class uvec4
{
public:
	uint32_t e[4];

	uvec4() = default;
	uvec4(uint32_t i) { e[0] = i;  e[1] = i; e[2] = i; e[3] = i; }
	uvec4(uint32_t i, uint32_t j, uint32_t k, uint32_t l) { e[0] = i; e[1] = j;  e[2] = k; e[3] = l;}

	uint32_t operator[](int i) const { return e[i]; }
	uint32_t& operator[](int i) { return e[i]; }
};

}