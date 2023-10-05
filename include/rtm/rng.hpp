#pragma once

#include "int.hpp"

#include "float.hpp"
#include "vec2.hpp"
#include "vec3.hpp"
#include "vec4.hpp"

namespace rtm
{

class RNG
{
public:
	//Robert Jenkins' one a time hash funcion for size 1 https://en.wikipedia.org/wiki/Jenkins_hash_function
	uint32_t hash(uint32_t u)
	{
		u += u << 10;
		u ^= u >> 6;
		u += u << 3;
		u ^= u >> 11;
		u += u << 15;
		return u;
	}

	static uint32_t fast_hash(uint32_t u)
	{
		return 1664525 * u + 1013904223;
	}

private:
	uint32_t _state{ 0u };

	float _build_float(uint32_t u)
	{
		uint32_t bits = u & 0x007FFFFFu | 0x3F800000u;
		return *((float*)(&bits)) - 1.0f;
	}

public:
	RNG(uint32_t seed = 0)
	{
		_state = hash(seed + 1);
	}

	float randf()
	{
		uint32_t r = _state;
		_state = fast_hash(_state);
		return _build_float(r);
	}

	rtm::vec2 randv2() { return rtm::vec2(randf(), randf()); }
	rtm::vec3 randv3() { return rtm::vec3(randf(), randf(), randf()); }
	rtm::vec4 randv4() { return rtm::vec4(randf(), randf(), randf(), randf()); }

	uint32_t randi()
	{
		uint32_t r = _state;
		_state = fast_hash(_state);
		return r;
	}
};

}