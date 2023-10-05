#pragma once
#include "stdafx.hpp"

#ifdef __riscv
//#define HARDWARE_INTERSECT
#endif

struct MeshPointers
{
	rtm::BVH::Node* blas;
	rtm::Triangle*  tris;
};

inline float intersect(const rtm::AABB aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
{
#ifdef HARDWARE_INTERSECT
	register float f3 asm("f3") = ray.o.x;
	register float f4 asm("f4") = ray.o.y;
	register float f5 asm("f5") = ray.o.z;
	register float f6 asm("f6") = ray.t_min;
	register float f7 asm("f7") = inv_d.x;
	register float f8 asm("f8") = inv_d.y;
	register float f9 asm("f9") = inv_d.z;
	register float f10 asm("f10") = ray.t_max;
	register float f11 asm("f11") = aabb.min.x;
	register float f12 asm("f12") = aabb.min.y;
	register float f13 asm("f13") = aabb.min.z;
	register float f14 asm("f14") = aabb.max.x;
	register float f15 asm("f15") = aabb.max.y;
	register float f16 asm("f16") = aabb.max.z;

	float t;
	asm volatile
	(
		"boxisect %0\t\n"
		: "=f" (t)
		: "f" (f3), "f" (f4), "f" (f5), "f" (f6), "f" (f7), "f" (f8), "f" (f9), "f" (f10),  "f" (f11), "f" (f12), "f" (f13), "f" (f14),  "f" (f15), "f" (f16)
	);

	return t;
#else
	rtm::vec3 t0 = (aabb.min - ray.o) * inv_d;
	rtm::vec3 t1 = (aabb.max - ray.o) * inv_d;

	rtm::vec3 tminv = rtm::min(t0, t1);
	rtm::vec3 tmaxv = rtm::max(t0, t1);

	float tmin = rtm::max(rtm::max(tminv.x, tminv.y), rtm::max(tminv.z, ray.t_min));
	float tmax = rtm::min(rtm::min(tmaxv.x, tmaxv.y), rtm::min(tmaxv.z, ray.t_max));

	if (tmin > tmax) return ray.t_max;//no hit || behind
	return tmin;
	#endif
}

inline bool intersect(const rtm::Triangle tri, const rtm::Ray& ray, rtm::Hit& hit)
{
#ifdef HARDWARE_INTERSECT
	register float f0 asm("f0") = hit.t;
	register float f1 asm("f1") = hit.bc.x;
	register float f2 asm("f2") = hit.bc.y;
	register float f3 asm("f3") = ray.o.x;
	register float f4 asm("f4") = ray.o.y;
	register float f5 asm("f5") = ray.o.z;
	register float f6 asm("f6") = ray.t_min;
	register float f7 asm("f7") = ray.d.x;
	register float f8 asm("f8") = ray.d.y;
	register float f9 asm("f9") = ray.d.z;
	register float f10 asm("f10") = ray.t_max;
	register float f11 asm("f11") = tri.vrts[0].x;
	register float f12 asm("f12") = tri.vrts[0].y;
	register float f13 asm("f13") = tri.vrts[0].z;
	register float f14 asm("f14") = tri.vrts[1].x;
	register float f15 asm("f15") = tri.vrts[1].y;
	register float f16 asm("f16") = tri.vrts[1].z;
	register float f17 asm("f17") = tri.vrts[2].x;
	register float f18 asm("f18") = tri.vrts[2].y;
	register float f19 asm("f19") = tri.vrts[2].z;

	uint32_t is_hit;
	asm volatile
	(
		"triisect %0\t\n"
		: "=r" (is_hit), "+f" (f0), "+f" (f1), "+f" (f2)
		: "f" (f3), "f" (f4), "f" (f5), "f" (f6), "f" (f7), "f" (f8), "f" (f9), "f" (f10),  "f" (f11), "f" (f12), "f" (f13), "f" (f14),  "f" (f15), "f" (f16), "f" (f17), "f" (f18), "f" (f19)
	);

	hit.t = f0;
	hit.bc.x = f1;
	hit.bc.y = f2;

	return is_hit;
#elif 0
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
inline bool intersect(const MeshPointers& mesh, const rtm::Ray& ray, rtm::Hit& hit, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::BVH::NodeData data;
	};

	NodeStackEntry node_stack[32];
	uint32_t node_stack_size = 1u;
	node_stack[0].t = intersect(mesh.blas[0].aabb, ray, inv_d);
	node_stack[0].data = mesh.blas[0].data;
	
	bool found_hit = false;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) continue;

	POP_SKIP:
		if(!current_entry.data.is_leaf)
		{
			uint child_index = current_entry.data.fst_chld_ind;
			float t0 = intersect(mesh.blas[child_index + 0].aabb, ray, inv_d);
			float t1 = intersect(mesh.blas[child_index + 1].aabb, ray, inv_d);
			if(t0 < hit.t || t1 < hit.t)
			{
				if(t0 < t1)
				{
					current_entry = {t0, mesh.blas[child_index + 0].data};
					if(t1 < hit.t)  node_stack[node_stack_size++] = {t1, mesh.blas[child_index + 1].data};
				}
				else
				{
					current_entry = {t1, mesh.blas[child_index + 1].data};
					if(t0 < hit.t)  node_stack[node_stack_size++] = {t0, mesh.blas[child_index + 0].data};
				}
				goto POP_SKIP;
			}
		}
		else
		{
			for(uint32_t i = 0; i <= current_entry.data.lst_chld_ofst; ++i)
			{
				uint32_t id = current_entry.data.fst_chld_ind + i;
				if(intersect(mesh.tris[id], ray, hit))
				{
					hit.id = id;
					if(first_hit) return true;
					else          found_hit = true;
				}
			}
		}
	} while(node_stack_size);

	return found_hit;
}