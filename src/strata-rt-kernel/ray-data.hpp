#pragma once
#include "stdafx.hpp"

namespace STRaTARTKernel {

struct RestartTrail
{
	const static uint N = 5;

	uint64_t _data{0};

	static uint shft(uint level)
	{
		return level * 3;
	}

	uint find_parent_level(uint level) const
	{
		for(uint i = level - 1; i < ~0u; --i)
			if(get(i) < N)
				return i;
		return ~0u;
	}

	uint get(uint level) const
	{
		uint64_t current_offset = (_data >> shft(level)) & 0x7ull;
		return current_offset;
	}

	void set(uint level, uint value)
	{
		_data &= ~(0x7ull << shft(level));
		_data |= (uint64_t)value << shft(level);
	}

	void clear(uint start_level)
	{
		uint64_t mask = ((0x1ull << shft(start_level)) - 1);
		_data &= mask;
	}

	bool is_done()
	{
		return _data == ~0ull;
	}

	void mark_done()
	{
		_data = ~0ull;
	}
};

struct alignas(64) RayData
{
	rtm::Ray ray; //32
	rtm::Hit hit; //16

	uint64_t global_ray_id : 26;
	uint64_t treelet_id : 20;
	uint64_t level : 6;
	uint64_t pf_mask : 8;

	RestartTrail restart_trail{}; //8 //3 per level 21 levels
};

struct HitReturn
{
	rtm::Hit hit;
	uint32_t index;		// id for framebuffer
};

}