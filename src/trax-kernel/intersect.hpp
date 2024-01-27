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

template<uint32_t FLAGS>
inline void _traceray(uint id, const rtm::Ray& ray, rtm::Hit& hit)
{
#ifdef __riscv
	register float src0 asm("f0") = ray.o.x;
	register float src1 asm("f1") = ray.o.y;
	register float src2 asm("f2") = ray.o.z;
	register float src3 asm("f3") = ray.t_min;
	register float src4 asm("f4") = ray.d.x;
	register float src5 asm("f5") = ray.d.y;
	register float src6 asm("f6") = ray.d.z;
	register float src7 asm("f7") = ray.t_max;

	register float dst0 asm("f28");
	register float dst1 asm("f29");
	register float dst2 asm("f30");
	register float dst3 asm("f31");

	asm volatile
	(
		"traceray %0, %4, %12\t\n"
		: "=f" (dst0), "=f" (dst1), "=f" (dst2), "=f" (dst3)
		: "f" (src0), "f" (src1), "f" (src2), "f" (src3), "f" (src4), "f" (src5), "f" (src6), "f" (src7), "I" (FLAGS) 
	);

	float _dst3 = dst3;

	hit.t = dst0;
	hit.bc.x = dst1;
	hit.bc.y = dst2;
	hit.id = *(uint*)&_dst3;
#else
#endif
}

inline float _intersect(const rtm::AABB& aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
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
	return rtm::intersect(aabb, ray, inv_d);
#endif
}

inline bool _intersect(const rtm::Triangle& tri, const rtm::Ray& ray, rtm::Hit& hit)
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
#else
	return rtm::intersect(tri, ray, hit);
#endif
}

inline bool intersect(const MeshPointers& mesh, const rtm::Ray& ray, rtm::Hit& hit, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::BVH::Node::Data data;
	};

	NodeStackEntry node_stack[32];
	uint32_t node_stack_size = 1u;
	node_stack[0].t = _intersect(mesh.blas[0].aabb, ray, inv_d);
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
			float t0 = _intersect(mesh.blas[child_index + 0].aabb, ray, inv_d);
			float t1 = _intersect(mesh.blas[child_index + 1].aabb, ray, inv_d);
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
				if(_intersect(mesh.tris[id], ray, hit))
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