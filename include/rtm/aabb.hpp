#pragma once

#include "float.hpp"
#include "int.hpp"
#include "vec3.hpp"

namespace rtm
{

class AABB
{
public:
	vec3 min{  FLT_MAX };
	vec3 max{ -FLT_MAX };

public:
	AABB() = default;

	void add(const AABB& other)
	{
		this->min = rtm::min(this->min, other.min);
		this->max = rtm::max(this->max, other.max);
	}

	void add(const rtm::vec3& point)
	{
		this->min = rtm::min(this->min, point);
		this->max = rtm::max(this->max, point);
	}

	float surface_area() const
	{
		float x = max.x - min.x;
		float y = max.y - min.y;
		float z = max.z - min.z;

		return (x * y + y * z + z * x) * 2.0f;
	}

	static float cost() { return 1.0f; };

	vec3 centroid() const { return (min + max) * 0.5f; }

	uint longest_axis() const
	{
		uint axis = 0;
		float max_length = 0.0f;
		for(uint i = 0; i < 3; ++i)
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

}