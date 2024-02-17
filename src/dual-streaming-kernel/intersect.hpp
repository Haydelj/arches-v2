#pragma once
#include "include.hpp"
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
	asm volatile("lwi f0, 0(x0)" : "=f" (f0), "=f" (f1), "=f" (f2), "=f" (f3), "=f" (f4), "=f" (f5), "=f" (f6), "=f" (f7), "=f" (f8), "=f" (f9));

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
	asm volatile("swi f0, 256(x0)" : : "f" (f0), "f" (f1), "f" (f2), "f" (f3), "f" (f4), "f" (f5), "f" (f6), "f" (f7), "f" (f8), "f" (f9));
#endif
}

inline void _cshit(const rtm::Hit& hit, rtm::Hit* dst)
{
#ifdef __riscv
	register float f15 asm("f15") = hit.t;
	register float f16 asm("f16") = hit.bc.x;
	register float f17 asm("f17") = hit.bc.y;
	register float f18 asm("f18") = *(float*)&hit.id;
	asm volatile("cshit %1, 0(%0)\t\n" : : "r" (dst), "f" (f15), "f" (f16), "f" (f17), "f" (f18) : "memory");
#endif
}

inline rtm::Hit _lhit(rtm::Hit* src)
{
#ifdef __riscv
	register float dst0 asm("f28");
	register float dst1 asm("f29");
	register float dst2 asm("f30");
	register float dst3 asm("f31");
	asm volatile("lhit %0, 0(%4)" : "=f" (dst0), "=f" (dst1), "=f" (dst2), "=f" (dst3) : "r" (src) : "memory");

	rtm::Hit cloest_hit;
	cloest_hit.t = dst0;
	cloest_hit.bc.x = dst1;
	cloest_hit.bc.y = dst2;
	float _dst3 = dst3;
	cloest_hit.id = *(uint*)&_dst3;

	return cloest_hit;

#endif
}

inline void _lhit_delay(rtm::Hit* src)
{
#ifdef __riscv
	register float dst0 asm("f28");
	register float dst1 asm("f29");
	register float dst2 asm("f30");
	register float dst3 asm("f31");
	asm volatile("lhit %0, 0(%4)" : "=f" (dst0), "=f" (dst1), "=f" (dst2), "=f" (dst3) : "r" (src) : "memory");
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

bool inline intersect_treelet(const Treelet& treelet, const rtm::Ray& ray, rtm::Hit& hit, uint* treelet_stack, float* child_hit, uint& treelet_stack_size)
{
#ifdef __riscv
	register float f28 asm("f28") __attribute__((unused));
	register float f29 asm("f29") __attribute__((unused));
	register float f30 asm("f30") __attribute__((unused));
	register float f31 asm("f31") __attribute__((unused));

	//register float f28 asm("f28");
	//register float f29 asm("f29");
	//register float f30 asm("f30");
	//register float f31 asm("f31");
#endif
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float hit_t;
		Treelet::Node::Data data;
		uint index;
	};
	NodeStackEntry node_stack[64]; uint node_stack_size = 1u;
	
	node_stack[0].hit_t = 0;
	node_stack[0].index = 0;
	node_stack[0].data = treelet.nodes[0].data;

	bool is_hit = false;
	while (node_stack_size)
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if (current_entry.hit_t >= hit.t) continue;

	TRAV:
		if (!current_entry.data.is_leaf)
		{
			const uint& child0_index = current_entry.data.child[0].index;
			const uint& child1_index = current_entry.data.child[1].index;
			const rtm::AABB& child0_aabb = treelet.nodes[current_entry.index].child_aabb[0];
			const rtm::AABB& child1_aabb = treelet.nodes[current_entry.index].child_aabb[1];
			const Treelet::Node& child0 = treelet.nodes[child0_index];
			const Treelet::Node& child1 = treelet.nodes[child1_index];
			const bool& left_child_treelet = current_entry.data.child[0].is_treelet;
			const bool& right_child_treelet = current_entry.data.child[1].is_treelet;
			float hit_ts[2];
			hit_ts[0] = _intersect(child0_aabb, ray, inv_d);
			hit_ts[1] = _intersect(child1_aabb, ray, inv_d);
			if (left_child_treelet || right_child_treelet)
			{
				if (hit_ts[0] < hit_ts[1])
				{
					if (hit_ts[0] < hit.t && left_child_treelet) child_hit[treelet_stack_size] = hit_ts[0], treelet_stack[treelet_stack_size++] = child0_index;
					if (hit_ts[1] < hit.t && right_child_treelet) child_hit[treelet_stack_size] = hit_ts[1], treelet_stack[treelet_stack_size++] = child1_index;
				}
				else
				{
					if (hit_ts[1] < hit.t && right_child_treelet) child_hit[treelet_stack_size] = hit_ts[1], treelet_stack[treelet_stack_size++] = child1_index;
					if (hit_ts[0] < hit.t && left_child_treelet) child_hit[treelet_stack_size] = hit_ts[0], treelet_stack[treelet_stack_size++] = child0_index;
				}
				if (left_child_treelet) hit_ts[0] = ray.t_max;
				if (right_child_treelet) hit_ts[1] = ray.t_max;
			}
			if (hit_ts[0] < hit_ts[1])
			{
				if (hit_ts[1] < hit.t) node_stack[node_stack_size++] = { hit_ts[1], child1.data, child1_index};
				if (hit_ts[0] < hit.t)
				{
					current_entry = { hit_ts[0], child0.data, child0_index};
					goto TRAV;
				}
			}
			else
			{
				if (hit_ts[0] < hit.t) node_stack[node_stack_size++] = { hit_ts[0], child0.data, child0_index };
				if (hit_ts[1] < hit.t)
				{
					current_entry = { hit_ts[1], child1.data, child1_index};
					goto TRAV;
				}
			}
		}
		else
		{
			Treelet::Triangle* tris = (Treelet::Triangle*)(&treelet.words[current_entry.data.tri0_word0]);
			for (uint i = 0; i <= current_entry.data.num_tri; ++i)
			{
				Treelet::Triangle tri = tris[i];
				if (_intersect(tri.tri, ray, hit))
				{
					hit.id = tri.id;
					is_hit = true;
				}
			}
		}
	}

	return is_hit;
}

inline void intersect_buckets(const KernelArgs& args)
{
#ifdef __riscv
	register float f28 asm("f28") __attribute__((unused));
	register float f29 asm("f29") __attribute__((unused));
	register float f30 asm("f30") __attribute__((unused));
	register float f31 asm("f31") __attribute__((unused));
	//register float f28 asm("f28");
	//register float f29 asm("f29");
	//register float f30 asm("f30");
	//register float f31 asm("f31");
#endif
	bool early = args.use_early;
	bool lhit_delay = args.hit_delay;
	while (1)
	{
		WorkItem wi = _lwi();
		if (wi.segment == ~0u) break;

		rtm::Ray ray = wi.bray.ray;
		rtm::Hit hit; hit.t = wi.bray.ray.t_max;
		if (early) {
			if (lhit_delay) _lhit_delay(args.hit_records + wi.bray.id);
			else hit = _lhit(args.hit_records + wi.bray.id);
		}
		uint treelet_stack[16]; uint treelet_stack_size = 0;
		float child_hit[16];
		if (intersect_treelet(args.treelets[wi.segment], wi.bray.ray, hit, treelet_stack, child_hit, treelet_stack_size))
		{
			// get cloest hit here
			rtm::Hit cloest_hit;
			cloest_hit.t = T_MAX;
#ifdef __riscv
			cloest_hit.t = f28;
			cloest_hit.bc.x = f29;
			cloest_hit.bc.y = f30;
			float _f31 = f31;
			cloest_hit.id = *(uint*)&_f31;
#endif
			//cloest_hit.t = T_MAX;
			if (early && lhit_delay && hit.t > cloest_hit.t)
			{
				hit = cloest_hit;
				wi.bray.ray.t_max = hit.t;
			}
			// update hit record with hit using cshit
			else if (hit.id != ~0u)
			{
				_cshit(hit, args.hit_records + wi.bray.id);
				wi.bray.ray.t_max = hit.t;
			}
		}
		rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;
		//drain treelet stack

		uint weight = 15;
		// weight = 2 ^ order
		// cloest: 2 ^ 15
		// second-cloest: 2 ^ 14
		while (treelet_stack_size)
		{
			uint treelet_index = treelet_stack[--treelet_stack_size];
			uint weight = (1 << 15 - treelet_stack_size);
			// If we delay the LHIT, we need to check here
			if (early && lhit_delay)
			{
				float hit_t = child_hit[treelet_stack_size];
				if (hit_t < hit.t)
				{
					wi.segment = (treelet_index | (weight << 16));
					_swi(wi);
				}
			}
			else
			{
				wi.segment = (treelet_index | (weight << 16));
				_swi(wi);
			}
		}
	}
}

inline bool intersect(const Treelet* treelets, const rtm::Ray& ray, rtm::Hit& hit)
{
	uint treelet_stack[64]; uint treelet_stack_size = 1u;
	float child_hit[64];
	treelet_stack[0] = 0;
	child_hit[0] = 0;
	bool is_hit = false;
	while (treelet_stack_size)
	{
		uint treelet_index = treelet_stack[--treelet_stack_size];
		is_hit |= intersect_treelet(treelets[treelet_index], ray, hit, treelet_stack, child_hit, treelet_stack_size);
	}

	return is_hit;
}