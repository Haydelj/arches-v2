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

inline void decompress(const rtm::Triangle& tri, uint& id, uint& count, rtm::Triangle tris[1])
{
	count = 1;
	tris[0] = tri;
}

class alignas(64) TriangleStrip
{
public:
	const static uint MAX_TRIS = 2;
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

inline void decompress(const rtm::TriangleStrip& strip, uint& id, uint& num_tris, rtm::Triangle tris[TriangleStrip::MAX_TRIS])
{
	id = strip.id;
	num_tris = strip.num_tris;
	tris[0] = Triangle(strip.vrts[0], strip.vrts[1], strip.vrts[2]);
	for(uint i = 1; i < num_tris; ++i)
	{
		if((strip.edge_mask >> (i - 1)) & 0x1)
		{
			tris[i] = Triangle(tris[i - 1].vrts[0], tris[i - 1].vrts[2], strip.vrts[i + 2]);
		}
		else
		{
			tris[i] = Triangle(tris[i - 1].vrts[2], tris[i - 1].vrts[1], strip.vrts[i + 2]);
		}
	}
}


}