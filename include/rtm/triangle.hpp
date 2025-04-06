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
	uint32_t id : 28;
	uint32_t num_tris : 2;
	uint32_t edge_mask : 2;
	//uint32_t num_tris : 4;
	//uint8_t edge_mask : MAX_TRIS;

	rtm::vec3 vrts[2 + MAX_TRIS];

public:
	TriangleStrip()
	{
		sizeof(TriangleStrip);
	}

	AABB aabb() const
	{
		AABB aabb;
		for(uint i = 0; i < num_tris + 2; ++i)
			aabb.add(vrts[i]);
		return aabb;
	}

	float cost() { return 1.0f; }
};

class alignas(32) IndexStrip
{
public:
	const static uint MAX_TRIS = TriangleStrip::MAX_TRIS;
	uint32_t id : 28;
	uint32_t num_tris : 4;
	uint8_t edge_mask : MAX_TRIS;

	uint inds[2 + MAX_TRIS];

public:
	IndexStrip()
	{
		sizeof(IndexStrip);
	}
};

struct alignas(32) QTriangleStrip
{
	const static uint MAX_TRIS = TriangleStrip::MAX_TRIS;
	uint32_t id : 28;
	uint32_t num_tris : 4;
	uint16_t edge_mask : MAX_TRIS;

	uint8_t data[9 * (2 + MAX_TRIS)];
	QTriangleStrip(const rtm::TriangleStrip& other) : id(other.id), num_tris(other.num_tris), edge_mask(other.edge_mask)
	{
		assert(other.num_tris <= MAX_TRIS);
		sizeof(QTriangleStrip);
		for(uint i = 0; i < 3 * num_tris + 6; ++i)
		{
			uint32_t u24 = f32_to_u24(((float*)other.vrts)[i]);
			std::memcpy(data + i * 3, &u24, 3);
		}
	}
};

inline TriangleStrip derefrence(const rtm::IndexStrip& strip, const rtm::vec3* verts)
{
	TriangleStrip tri_strip;
	tri_strip.id = strip.id;
	tri_strip.num_tris = strip.num_tris;
	tri_strip.edge_mask = strip.edge_mask;
	for(uint i = 0; i < strip.num_tris + 2; ++i)
		tri_strip.vrts[i] = verts[strip.inds[i]];
	return tri_strip;
}

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

inline uint decompress(const rtm::QTriangleStrip& strip, uint strip_id, rtm::IntersectionTriangle* tris)
{
	rtm::TriangleStrip temp_strip;
	temp_strip.id = strip.id;
	temp_strip.num_tris = strip.num_tris;
	temp_strip.edge_mask = strip.edge_mask;
	for(uint i = 0; i < 3 * (strip.num_tris + 2); ++i)
	{
		uint32_t u24;
		std::memcpy(&u24, strip.data + i * 3, 3);
		((float*)temp_strip.vrts)[i] = u24_to_f32(u24);
	}
	return decompress(temp_strip, strip_id, tris);
}


}