#pragma once
#include "stdafx.hpp"

#include "work-item.hpp"
#include "treelet-bvh.hpp"
#include "custom-instr.hpp"

#ifdef __riscv
//#define BOX_PIPLINE
//#define TRI_PIPLINE
#endif

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
	wi.bray.ray.t_max = f7;

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
	asm volatile("swi f0, 256(x0)" : : "f" (f0),  "f" (f1), "f" (f2), "f" (f3), "f" (f4), "f" (f5), "f" (f6), "f" (f7), "f" (f8), "f" (f9));
#endif
}

inline void _cshit(const rtm::Hit& hit, rtm::Hit* dst)
{
#ifdef __riscv
	register float f15 asm("f15") = hit.t;
	register float f16 asm("f16") = hit.bc.x;
	register float f17 asm("f17") = hit.bc.y;
	register float f18 asm("f18") = *(float*)&hit.id;
	asm volatile("cshit f15, 0(%0)" : : "r" (dst), "f" (f15),  "f" (f16), "f" (f17), "f" (f18));
#endif
}

inline rtm::Hit _lhit(rtm::Hit* src)
{
#ifdef __riscv
	register float f15 asm("f15");
	register float f16 asm("f16");
	register float f17 asm("f17");
	register float f18 asm("f18");
	asm volatile("lhit f15, 0(%4)" : "=f" (f15), "=f" (f16), "=f" (f17), "=f" (f18) : "r" (src));

	rtm::Hit hit;
	hit.t = f15;
	hit.bc.x = f16;
	hit.bc.y = f17;

	float _f18 = f18;
	hit.id = *(uint*)&_f18;

	return hit;
#endif
}


inline float _intersect(const rtm::AABB& aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
{
#ifdef BOX_PIPLINE
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
	return rtm::intersect(aabb, ray, inv_d);
#endif
}

inline bool _intersect(const rtm::Triangle& tri, const rtm::Ray& ray, rtm::Hit& hit)
{
#ifdef TRI_PIPLINE
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
	return rtm::intersect(tri, ray, hit);
#endif
}

bool inline intersect_treelet(const Treelet& treelet, const rtm::Ray& ray, rtm::Hit& hit, uint* treelet_stack, uint& treelet_stack_size)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float hit_t;
		Treelet::Node::Data data;
	};
	NodeStackEntry node_stack[64]; uint node_stack_size = 1u;

	const Treelet::Node& root_node = treelet.nodes[0];
	node_stack[0].hit_t = _intersect(root_node.aabb, ray, inv_d);
	if(node_stack[0].hit_t >= hit.t) return false;
	node_stack[0].data = root_node.data;

	bool is_hit = false;
	while(node_stack_size)
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.hit_t >= hit.t) continue;

	TRAV:
		if(!current_entry.data.is_leaf)
		{
			const Treelet::Node& child0 = treelet.nodes[current_entry.data.child[0].index];
			const Treelet::Node& child1 = treelet.nodes[current_entry.data.child[1].index];

			float hit_ts[2];
			if(current_entry.data.child[0].is_treelet)
			{
				treelet_stack[treelet_stack_size++] = current_entry.data.child[0].index;
				hit_ts[0] = ray.t_max;
			}
			else hit_ts[0] = _intersect(child0.aabb, ray, inv_d);

			if(current_entry.data.child[1].is_treelet)
			{
				treelet_stack[treelet_stack_size++] = current_entry.data.child[1].index;
				hit_ts[1] = ray.t_max;
			}
			else hit_ts[1] = _intersect(child1.aabb, ray, inv_d);

			if(hit_ts[0] < hit_ts[1])
			{
				if(hit_ts[1] < hit.t) node_stack[node_stack_size++] = {hit_ts[1], child1.data};
				if(hit_ts[0] < hit.t)
				{
					current_entry = {hit_ts[0], child0.data};
					goto TRAV;
				}
			}
			else
			{
				if(hit_ts[0] < hit.t) node_stack[node_stack_size++] = {hit_ts[0], child0.data};
				if(hit_ts[1] < hit.t)
				{
					current_entry = {hit_ts[1], child1.data};
					goto TRAV;
				}
			}
		}
		else
		{
			Treelet::Triangle* tris = (Treelet::Triangle*)(&treelet.words[current_entry.data.tri0_word0]);
			for(uint i = 0; i <= current_entry.data.num_tri; ++i)
			{
				Treelet::Triangle tri = tris[i];
				if(_intersect(tri.tri, ray, hit))
				{
					hit.id = tri.id;
					is_hit |= true;
				}
			}
		}
	}

	return is_hit;
}

inline void intersect_buckets(const Treelet* treelets, rtm::Hit* hit_records)
{
	while(1)
	{
		WorkItem wi = _lwi();
		if(wi.segment == ~0u) break;

		rtm::Hit hit; hit.t = wi.bray.ray.t_max;
		uint treelet_stack[16]; uint treelet_stack_size = 0;
		if(intersect_treelet(treelets[wi.segment], wi.bray.ray, hit, treelet_stack, treelet_stack_size))
		{
			//update hit record with hit using cshit
			_cshit(hit, hit_records + wi.bray.id);
			wi.bray.ray.t_max = hit.t;
		}

		//drain treelet stack
		while(treelet_stack_size)
		{
			uint treelet_index = treelet_stack[--treelet_stack_size];
			wi.segment = treelet_index;
			_swi(wi);
		}
	}
}

inline bool intersect(const Treelet* treelets, const rtm::Ray& ray, rtm::Hit& hit)
{
	uint treelet_stack[64]; uint treelet_stack_size = 1u; 
	treelet_stack[0] = 0;

	bool is_hit = false;
	while(treelet_stack_size)
	{
		uint treelet_index = treelet_stack[--treelet_stack_size];
		is_hit |= intersect_treelet(treelets[treelet_index], ray, hit, treelet_stack, treelet_stack_size);
	}

	return is_hit;
}