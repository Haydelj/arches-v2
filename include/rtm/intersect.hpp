#pragma once

#include "ray.hpp"
#include "frustrum.hpp"
#include "aabb.hpp"
#include "triangle.hpp"
#include "vec2.hpp"

namespace rtm
{

inline float intersect(const rtm::AABB& aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
{
	rtm::vec3 t0 = (aabb.min - ray.o) * inv_d;
	rtm::vec3 t1 = (aabb.max - ray.o) * inv_d;

	rtm::vec3 tminv = rtm::min(t0, t1);
	rtm::vec3 tmaxv = rtm::max(t0, t1);

	float tmin = max(max(tminv.x, tminv.y), max(tminv.z, ray.t_min));
	float tmax = min(min(tmaxv.x, tmaxv.y), min(tmaxv.z, ray.t_max));

	if (tmin > tmax || tmax < ray.t_min) return ray.t_max;//no hit || behind
	return tmin;
}

inline float intersect(const rtm::AABB& aabb, const rtm::Frustrum& frustrum)
{
	rtm::vec3 d[4];
	d[0] = frustrum.d;
	d[1] = frustrum.d + frustrum.dx;
	d[2] = frustrum.d + frustrum.dy;
	d[3] = frustrum.d + frustrum.dx + frustrum.dy;

	uint8_t o_mask =
		((frustrum.o.x >= aabb.min.x && frustrum.o.x <= aabb.max.x) << 0) |
		((frustrum.o.y >= aabb.min.y && frustrum.o.y <= aabb.max.y) << 1) |
		((frustrum.o.z >= aabb.min.z && frustrum.o.z <= aabb.max.z) << 2) ;

	bool hit = false;
	uint8_t lt_masks[4], eq_masks[4], min_order[4];
	rtm::vec3 _tminv[4], _tmaxv[4];
	float min_t = frustrum.t_min;
	for(uint i = 0; i < 4; ++i)
	{
		rtm::vec3 inv_d = rtm::vec3(1.0f) / d[i];

		rtm::vec3 t0 = (aabb.min - frustrum.o) * inv_d;
		rtm::vec3 t1 = (aabb.max - frustrum.o) * inv_d;

		rtm::vec3 tminv = rtm::min(t0, t1);
		rtm::vec3 tmaxv = rtm::max(t0, t1);

		float tmin = max(max(tminv.x, tminv.y), max(tminv.z, frustrum.t_min));
		float tmax = min(min(tmaxv.x, tmaxv.y), min(tmaxv.z, frustrum.t_max));

		lt_masks[i] = ((t0.x < t1.x) << 0) | ((t0.y < t1.y) << 1) | ((t0.z < t1.z) << 2);
		eq_masks[i] = ((t0.x == t1.x) << 0) | ((t0.y == t1.y) << 1) | ((t0.z == t1.z) << 2);
		min_order[i] = ((tminv.y < tminv.z) << 0) | ((tminv.z < tminv.x) << 1) | ((tminv.x < tminv.y) << 2);

		if(tmin <= tmax && tmax >= frustrum.t_min)
		{
			min_t = min(min_t, tmin);
			hit = true;
		}
	}

	if(hit)
		return frustrum.t_min;

	//evaluate other hit conditions
	uint8_t c0 = 0b000;
	uint8_t c1 = 0b000;
	uint8_t v0 = 0b000;
	uint8_t w0 = 0b111;
	for(uint i = 0; i < 4; ++i)
	{
		uint8_t v1 = lt_masks[i];
		uint8_t w1 = eq_masks[i];

		c0 |= min_order[i] ^ min_order[0];
		c1 |= (v0 ^ v1) & ~(w0 | w1);
		v0 = (~w0 & v0) | (w0 & v1);
		w0 = w0 & w1;
	}

	c1 = c1 & o_mask;

	uint count;
	for(uint i = 0; i < 3; ++i)
		count += (c0 >> i) & 0x1;

	if(count >= 2)
		return frustrum.t_min;

	return frustrum.t_max;
}

inline bool intersect(const rtm::Triangle& tri, const rtm::Ray& ray, rtm::Hit& hit)
{
#if 0
	rtm::vec3 bc;
	bc[0] = rtm::dot(rtm::cross(tri.vrts[2] - tri.vrts[1], tri.vrts[1] - ray.o), ray.d);//
	bc[1] = rtm::dot(rtm::cross(tri.vrts[0] - tri.vrts[2], tri.vrts[2] - ray.o), ray.d);
	bc[2] = rtm::dot(rtm::cross(tri.vrts[1] - tri.vrts[0], tri.vrts[0] - ray.o), ray.d);
		
	rtm::vec3 gn = rtm::cross(tri.vrts[1] - tri.vrts[0], tri.vrts[2] - tri.vrts[0]);
	float gn_dot_d = rtm::dot(gn, ray.d);

	if(gn_dot_d > 0.0f) bc = -bc;
	if(bc[0] < 0.0f || bc[1] < 0.0f || bc[2] < 0.0f) return false;

	float t = rtm::dot(gn, tri.vrts[0] - ray.o) / gn_dot_d;
	if(t < ray.t_min || t > hit.t) return false;

	hit.bc = rtm::vec2(bc.x, bc.y) / (bc[0] + bc[1] + bc[2]);
	hit.t = t ;
	return true;
#else
    rtm::vec3 e0     = tri.vrts[1] - tri.vrts[2];//2,2
    rtm::vec3 e1     = tri.vrts[0] - tri.vrts[2];//2
  	//rtm::vec3 normal = rtm::normalize(rtm::cross(e1, e0));
    rtm::vec3 r1     = rtm::cross(ray.d, e0);//4, 6
    float denom      = rtm::dot(e1, r1);//6, 12
    float rcp_denom  = 1.0f / denom; //2, 14
    rtm::vec3 s       = ray.o - tri.vrts[2]; //2
    float b1          = rtm::dot(s, r1) * rcp_denom;//2, 16
    if (b1 < 0.0f || b1 > 1.0f) //1, 17
        return false;

    rtm::vec3 r2 = rtm::cross(s, e1); //4
    float b2  = rtm::dot(ray.d, r2) * rcp_denom;//2, 16
    if (b2 < 0.0f || (b2 + b1) > 1.0f)//1, 18
       	return false;

    float t = rtm::dot(e0, r2) * rcp_denom;//2, 16
	if(t < ray.t_min || t > hit.t) 
		return false;

	hit.bc = rtm::vec2(b1, b2); //20
	hit.t = t;
	return true;
#endif
}

}