#pragma once
#include "stdafx.hpp"

#include "treelet-bvh.hpp"

struct BucketRay
{
	rtm::Ray ray;
	uint32_t id{~0u};
};

struct WorkItem
{
	BucketRay bray;
	uint32_t  segment;
};

inline WorkItem _lwi()
{
#ifdef __riscv
	register float f0 asm("f0");
	register float f1 asm("f1");
	register float f2 asm("f2");
	register float f3 asm("f3");
	register float f4 asm("f4");
	register float f5 asm("f5");
	register float f6 asm("f6");
	register float f7 asm("f7");
	register float f8 asm("f8");
	register float f9 asm("f9");
	asm volatile("lwi f0, 0(x0)" : "=f" (f0),  "=f" (f1), "=f" (f2), "=f" (f3), "=f" (f4), "=f" (f5), "=f" (f6), "=f" (f7), "=f" (f8), "=f" (f9));

	WorkItem wi;
	wi.bray.ray.o.x = f0;
	wi.bray.ray.o.y = f1;
	wi.bray.ray.o.z = f2;
	wi.bray.ray.t_min = f3;
	wi.bray.ray.d.x = f4;
	wi.bray.ray.d.y = f5;
	wi.bray.ray.d.z = f6;
	wi.bray.ray.t_min = f7;

	float _f8 = f8;
	wi.bray.id = *(uint*)&_f8;

	float _f9 = f9;
	wi.segment = *(uint*)&_f9;

	return wi;
#else
	return WorkItem();
#endif
}

inline void _swi(const WorkItem& wi)
{
#ifdef __riscv
	register float f0 asm("f0") = wi.bray.ray.o.x;
	register float f1 asm("f1") = wi.bray.ray.o.y;
	register float f2 asm("f2") = wi.bray.ray.o.z;
	register float f3 asm("f3") = wi.bray.ray.t_min;
	register float f4 asm("f4") = wi.bray.ray.d.x;
	register float f5 asm("f5") = wi.bray.ray.d.y;
	register float f6 asm("f6") = wi.bray.ray.d.z;
	register float f7 asm("f7") = wi.bray.ray.t_max;
	register float f8 asm("f8") = *(float*)&wi.bray.id;
	register float f9 asm("f9") = *(float*)&wi.segment;
	asm volatile("swi f0, 0(x0)" : : "f" (f0),  "f" (f1), "f" (f2), "f" (f3), "f" (f4), "f" (f5), "f" (f6), "f" (f7), "f" (f8), "f" (f9));
#endif
}

inline void _cshit(const rtm::Hit& hit, rtm::Hit* dst)
{
#ifdef __riscv
	register float f17 asm("f17") = hit.t;
	register float f18 asm("f18") = hit.bc.x;
	register float f19 asm("f19") = hit.bc.y;
	register float f20 asm("f20") = *(float*)&hit.id;
	asm volatile("cshit f17, 0(x0)" : : "f" (f17),  "f" (f18), "f" (f19), "f" (f20));
#endif
}

inline float intersect(const rtm::AABB& aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
{
#ifdef __riscv
	register float f0 asm("f0") = ray.o.x;
	register float f1 asm("f1") = ray.o.y;
	register float f2 asm("f2") = ray.o.z;
	register float f3 asm("f3") = inv_d.x;
	register float f4 asm("f4") = inv_d.y;
	register float f5 asm("f5") = inv_d.z;

	register float f6 asm("f6") = aabb.min.x;
	register float f7 asm("f7") = aabb.min.y;
	register float f8 asm("f8") = aabb.min.z;
	register float f9 asm("f9") = aabb.max.x;
	register float f10 asm("f10") = aabb.max.y;
	register float f11 asm("f11") = aabb.max.z;

	float t;
	asm volatile
	(
		"boxisect %0"
		: 
		"=f" (t)
		: 
		"f" (f0),
		"f" (f1),
		"f" (f2),
		"f" (f3),
		"f" (f4),
		"f" (f5),
		"f" (f6),
		"f" (f7),
		"f" (f8),
		"f" (f9),
		"f" (f10),
		"f" (f11)
	);

	return t;
#else
	return rtm::intersect_aabb(aabb, ray, inv_d);
#endif
}

inline bool intersect(const rtm::Triangle& tri, const rtm::Ray& ray, rtm::Hit& hit)
{
#ifdef __riscv
	register float f0 asm("f0") = ray.o.x;
	register float f1 asm("f1") = ray.o.y;
	register float f2 asm("f2") = ray.o.z;
	register float f3 asm("f3") = ray.d.x;
	register float f4 asm("f4") = ray.d.y;
	register float f5 asm("f5") = ray.d.z;

	register float f6 asm("f6") = tri.vrts[0].x;
	register float f7 asm("f7") = tri.vrts[0].y;
	register float f8 asm("f8") = tri.vrts[0].z;
	register float f9 asm("f9") = tri.vrts[1].x;
	register float f10 asm("f10") = tri.vrts[1].y;
	register float f11 asm("f11") = tri.vrts[1].z;
	register float f12 asm("f12") = tri.vrts[2].x;
	register float f13 asm("f13") = tri.vrts[2].y;
	register float f14 asm("f14") = tri.vrts[2].z;

	register float f15 asm("f15") = hit.t;
	register float f16 asm("f16") = hit.bc.x;
	register float f17 asm("f17") = hit.bc.y;

	uint is_hit;
	asm volatile
	(
		"triisect %0\n\t"
		: 
		"=r" (is_hit),
		"+f" (f15),
		"+f" (f16),
		"+f" (f17)
		: 
		"f" (f0),
		"f" (f1),
		"f" (f2),
		"f" (f3),
		"f" (f4),
		"f" (f5),
		"f" (f6),
		"f" (f7),
		"f" (f8),
		"f" (f9),
		"f" (f10),
		"f" (f11),
		"f" (f12),
		"f" (f13),
		"f" (f14)
	);

	hit.t = f15;
	hit.bc.x = f16;
	hit.bc.y = f17;

	return is_hit;
#else
	return rtm::intersect_tri(tri, ray, hit);
#endif
}

bool inline intersect_treelet(const Treelet& treelet, const rtm::Ray& ray, rtm::Hit& hit, uint* treelet_stack, uint& treelet_stack_size)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float hit_t{T_MAX};
		union
		{
			uint32_t data;
			struct
			{
				uint32_t is_leaf : 1;
				uint32_t is_treelet_leaf : 1;
				uint32_t last_tri_offset : 3;
				uint32_t child_index : 27;
			};
		};
	};

	uint node_stack_index = 0u; NodeStackEntry node_stack[32];
	node_stack[0].hit_t = T_MIN;
	node_stack[0].is_leaf = 0;
	node_stack[0].is_treelet_leaf = 0;
	node_stack[0].child_index = 0;

	bool is_hit = false;
	while(node_stack_index != ~0u)
	{
		NodeStackEntry current_entry = node_stack[node_stack_index--];
		if(current_entry.hit_t >= hit.t) continue;

	TRAV:
		if(!current_entry.is_leaf)
		{
			if(current_entry.is_treelet_leaf)
			{
				treelet_stack[treelet_stack_size++] = current_entry.child_index;
			}
			else
			{
				TreeletNode nodes_local[2];
				nodes_local[0] = ((TreeletNode*)&treelet.data[current_entry.child_index * 4])[0];
				nodes_local[1] = ((TreeletNode*)&treelet.data[current_entry.child_index * 4])[1];

				float hit_ts[2];
				hit_ts[0] = intersect(nodes_local[0].aabb, ray, inv_d);
				hit_ts[1] = intersect(nodes_local[1].aabb, ray, inv_d);

				if(hit_ts[0] < hit_ts[1])
				{

					if(hit_ts[1] < hit.t) node_stack[++node_stack_index] = {hit_ts[1], nodes_local[1].data};
					if(hit_ts[0] < hit.t)
					{
						current_entry = {hit_ts[0], nodes_local[0].data};
						goto TRAV;
					}
				}
				else
				{
					if(hit_ts[0] < hit.t) node_stack[++node_stack_index] = {hit_ts[0], nodes_local[0].data};
					if(hit_ts[1] < hit.t)
					{
						current_entry = {hit_ts[1], nodes_local[1].data};
						goto TRAV;
					}
				}
			}
		}
		else
		{
			TreeletTriangle* tris = (TreeletTriangle*)(&treelet.data[current_entry.child_index * 4]);
			for(uint i = 0; i <= current_entry.last_tri_offset; ++i)
			{
				TreeletTriangle tri = tris[i];
				if(intersect(tri.tri, ray, hit))
				{
					hit.id = tri.id;
					is_hit |= true;
				}
			}
		}
	}

	return is_hit;
}

bool inline intersect(const Treelet* treelets, const rtm::Ray& ray, rtm::Hit& hit)
{
	uint treelet_stack_size = 1u;  uint treelet_stack[64];
	treelet_stack[0] = 0;

	bool is_hit = false;
	while(treelet_stack_size != 0u)
	{
		//TODO in dual streaming this comes from the current scene segment we are traversing
		uint treelet_index = treelet_stack[--treelet_stack_size];
		is_hit |= intersect_treelet(treelets[treelet_index], ray, hit, treelet_stack, treelet_stack_size);
	}

	return is_hit;
}

bool inline intersect_buckets(void* ray_staging_buffer, const Treelet* scene_buffer, rtm::Hit* hit_records)
{
	bool is_hit = false;
	while(1)
	{
		WorkItem work_item = _lwi();
		if(work_item.segment == ~0u) break;

		rtm::Hit hit; hit.t = T_MAX;
		uint treelet_stack[64]; uint treelet_stack_size = 0;
		if(intersect_treelet(scene_buffer[work_item.segment], work_item.bray.ray, hit, treelet_stack, treelet_stack_size))
		{
			//update hit record with hit using cshit
			_cshit(hit, hit_records + work_item.bray.id);
		}
		
		//drain treelet stack
		for(uint i = 0; i < treelet_stack_size; ++i)
		{
			work_item.segment = treelet_stack[i];
			_swi(work_item);
		}
	}

	return is_hit;
}

