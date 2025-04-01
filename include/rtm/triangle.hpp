#pragma once

#include "aabb.hpp"

namespace rtm
{

class alignas(64) Triangle
{
public:
	rtm::vec3 vrts[3];

public:
	Triangle() = default;

	Triangle(const rtm::vec3& v0, const rtm::vec3& v1, const rtm::vec3& v2)
	{
		this->vrts[0] = v0;
		this->vrts[1] = v1;
		this->vrts[2] = v2;
	}

	AABB aabb() const
	{
		AABB aabb;
		for(uint i = 0; i < 3; ++i)
			aabb.add(vrts[i]);
		return aabb;
	}

	static float cost() { return 1.0f; }

	rtm::vec3 normal()
	{
		return rtm::normalize(rtm::cross(vrts[0] - vrts[2], vrts[1] - vrts[2]));
	}
};

struct IntersectionTriangle
{
	Triangle tri;
	uint id;
};

inline uint decompress(const rtm::Triangle& in, uint id0, rtm::IntersectionTriangle* out)
{
	out[0].tri = in;
	out[0].id = id0;
	return 1;
}

class alignas(64) TriangleStrip
{
public:
	const static uint MAX_TRIS = 1;
	uint32_t id : 29;
	uint16_t num_tris : 4;
	uint16_t edge_mask : 12;
	rtm::vec3 vrts[2 + MAX_TRIS];

public:
	TriangleStrip() = default;

	AABB aabb() const
	{
		AABB aabb;
		for(uint i = 0; i < num_tris + 2; ++i)
			aabb.add(vrts[i]);
		return aabb;
	}

	float cost() { return 1.0f; }
};

inline uint decompress(const rtm::TriangleStrip& strip, uint strip_id, rtm::IntersectionTriangle* out)
{
	out[0].tri = Triangle(strip.vrts[0], strip.vrts[1], strip.vrts[2]);
	out[0].id = strip.id;
	for(uint i = 1; i < strip.num_tris; ++i)
	{
		out[i].id = strip.id + i;
		if((strip.edge_mask >> (i - 1)) & 0x1)
			out[i].tri = Triangle(out[i - 1].tri.vrts[0], out[i - 1].tri.vrts[2], strip.vrts[i + 2]);
		else
			out[i].tri = Triangle(out[i - 1].tri.vrts[2], out[i - 1].tri.vrts[1], strip.vrts[i + 2]);
	}
	return strip.num_tris;
}


}