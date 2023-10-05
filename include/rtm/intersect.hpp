#pragma once

#include "ray.hpp"
#include "aabb.hpp"
#include "triangle.hpp"
#include "vec2.hpp"

namespace rtm
{

inline float intersect_aabb(const rtm::AABB& aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
{
	rtm::vec3 t0 = (aabb.min - ray.o) * inv_d;
	rtm::vec3 t1 = (aabb.max - ray.o) * inv_d;

	rtm::vec3 tminv = rtm::min(t0, t1);
	rtm::vec3 tmaxv = rtm::max(t0, t1);

	float tmin = max(max(tminv.x, tminv.y), max(tminv.z, ray.t_min));
	float tmax = min(min(tmaxv.x, tmaxv.y), min(tmaxv.z, ray.t_max));

	if (tmin > tmax || tmax < ray.t_max) return ray.t_max;//no hit || behind
	return tmin;
}

inline bool intersect_tri(const rtm::Triangle& tri, const rtm::Ray& ray, rtm::Hit& hit)
{
#if 0
	rtm::vec3 bc;
	bc[0] = rtm::dot(rtm::cross(tri.vrts[2] - tri.vrts[1], tri.vrts[1] - ray.o), ray.d);
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
    rtm::vec3 e0     = tri.vrts[1] - tri.vrts[2];
    rtm::vec3 e1     = tri.vrts[0] - tri.vrts[2];
    rtm::vec3 normal = rtm::normalize(rtm::cross(e1, e0));
    rtm::vec3 r1     = rtm::cross(ray.d, e0);
    float denom      = rtm::dot(e1, r1);
    float rcp_denom  = 1.0f / denom;
    rtm::vec3 s       = ray.o - tri.vrts[2];
    float b1          = rtm::dot(s, r1) * rcp_denom;
    if (b1 < 0.0f || b1 > 1.0f)
        return false;

    rtm::vec3 r2 = rtm::cross(s, e1);
    float b2  = rtm::dot(ray.d, r2) * rcp_denom;
    if (b2 < 0.0f || (b2 + b1) > 1.0f)
       	return false;

    float t = rtm::dot(e0, r2) * rcp_denom;
	if(t < ray.t_min || t > hit.t) 
		return false;

	hit.bc = rtm::vec2(b1, b2);
	hit.t = t;
	return true;
#endif
}

}