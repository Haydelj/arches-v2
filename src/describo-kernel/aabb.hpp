#pragma once
#include "stdafx.hpp"

class AABB
{
public:
	rtm::vec3 min{  FLT_MAX };
	rtm::vec3 max{ -FLT_MAX };

public:
	AABB() = default;

	void add(const AABB& other)
	{
		this->min = rtm::min(this->min, other.min);
		this->max = rtm::max(this->max, other.max);
	}

	float surface_area() const
	{
		float x = max.x - min.x;
		float y = max.y - min.y;
		float z = max.z - min.z;

		return (x * y + y * z + z * x) * 2.0f;
	}

	static float cost() { return 1.0f; };

	rtm::vec3 centroid() const { return (min + max) * 0.5f; }

	uint32_t longest_axis() const
	{
		uint32_t axis = 0;
		float max_length = 0.0f;
		for(uint32_t i = 0; i < 3; ++i)
		{
			float length = max[i] - min[i];
			if(length > max_length)
			{
				axis = i;
				max_length = length;
			}
		}
		return axis;
	}
};


class AABB16
{
public:
	uint16_t min[3];
	uint16_t max[3];

	AABB16() {}

	AABB16(const AABB& aabb)
	{
		for(uint i = 0; i < 3; ++i)
		{
			min[i] = f32_to_u16_rd(aabb.min[i] * 0.5f + 0.5f);
			max[i] = f32_to_u16_ru(aabb.max[i] * 0.5f + 0.5f);
		}
	}
};
