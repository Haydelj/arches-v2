#pragma once
#include "stdafx.hpp"
#include "include.hpp"
#include "work-item.hpp"
#include "custom-instr.hpp"

#ifdef __riscv
inline WorkItem _lwi()
{
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
	wi._data = *(uint*)&_f9;

	return wi;
}
#endif

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
	register float f9 asm("f9") = *(float*)&wi._data;
	asm volatile("swi f0, 256(x0)" : : "f" (f0), "f" (f1), "f" (f2), "f" (f3), "f" (f4), "f" (f5), "f" (f6), "f" (f7), "f" (f8), "f" (f9));
#endif
}

#ifdef __riscv
inline void _cshit(const rtm::Hit& hit, rtm::Hit* dst)
{
	register float f15 asm("f15") = hit.t;
	register float f16 asm("f16") = hit.bc.x;
	register float f17 asm("f17") = hit.bc.y;
	register float f18 asm("f18") = *(float*)&hit.id;
	asm volatile("cshit %1, 0(%0)\t\n" : : "r" (dst), "f" (f15), "f" (f16), "f" (f17), "f" (f18) : "memory");
}
#endif

inline rtm::Hit _lhit(rtm::Hit* src)
{
#ifdef __riscv
	register float dst0 asm("f28");
	register float dst1 asm("f29");
	register float dst2 asm("f30");
	register float dst3 asm("f31");
	asm volatile("lhit %0, 0(%4)" : "=f" (dst0), "=f" (dst1), "=f" (dst2), "=f" (dst3) : "r" (src) : "memory");

	rtm::Hit hit;
	hit.t = dst0;
	hit.bc.x = dst1;
	hit.bc.y = dst2;
	float _dst3 = dst3;
	hit.id = *(uint*)&_dst3;

	return hit;
#else 
	return *src;
#endif


}

#ifdef __riscv
inline void _lhit_delay(rtm::Hit* src)
{
	register float dst0 asm("f28");
	register float dst1 asm("f29");
	register float dst2 asm("f30");
	register float dst3 asm("f31");
	asm volatile("lhit %0, 0(%4)" : "=f" (dst0), "=f" (dst1), "=f" (dst2), "=f" (dst3) : "r" (src) : "memory");
}
#endif

inline float _intersect(const rtm::AABB& aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
{
#if defined(__riscv) && defined(USE_HARDWARE_INTERSECTORS)
	register float src0 asm("f0") = ray.o.x;
	register float src1 asm("f1") = ray.o.y;
	register float src2 asm("f2") = ray.o.z;
	register float src3 asm("f3") = ray.t_min;
	register float src4 asm("f4") = inv_d.x;
	register float src5 asm("f5") = inv_d.y;
	register float src6 asm("f6") = inv_d.z;
	register float src7 asm("f7") = ray.t_max;

	register float src8 asm("f8")   = aabb.min.x;
	register float src9 asm("f9")   = aabb.min.y;
	register float src10 asm("f10") = aabb.min.z;
	register float src11 asm("f11") = aabb.max.x;
	register float src12 asm("f12") = aabb.max.y;
	register float src13 asm("f13") = aabb.max.z;

	float t;
	asm volatile ("boxisect %0" : "=f" (t) :"f" (src0), "f" (src1), "f" (src2), "f" (src3), "f" (src4), "f" (src5), "f" (src6), "f" (src7), "f" (src8), "f" (src9), "f" (src10), "f" (src11), "f" (src12), "f" (src13));

	return t;
#else
	return rtm::intersect(aabb, ray, inv_d);
#endif
}

inline bool _intersect(const rtm::Triangle& tri, const rtm::Ray& ray, rtm::Hit& hit)
{
#if defined(__riscv) && defined(USE_HARDWARE_INTERSECTORS)
	register float src0 asm("f0") = ray.o.x;
	register float src1 asm("f1") = ray.o.y;
	register float src2 asm("f2") = ray.o.z;
	register float src3 asm("f3") = ray.t_min;
	register float src4 asm("f4") = ray.d.x;
	register float src5 asm("f5") = ray.d.y;
	register float src6 asm("f6") = ray.d.z;
	register float src7 asm("f7") = ray.t_max;

	register float src8 asm("f8")   = tri.vrts[0].x;
	register float src9 asm("f9")   = tri.vrts[0].y;
	register float src10 asm("f10") = tri.vrts[0].z;
	register float src11 asm("f11") = tri.vrts[1].x;
	register float src12 asm("f12") = tri.vrts[1].y;
	register float src13 asm("f13") = tri.vrts[1].z;
	register float src14 asm("f14") = tri.vrts[2].x;
	register float src15 asm("f15") = tri.vrts[2].y;
	register float src16 asm("f16") = tri.vrts[2].z;

	register float dst0 asm("f17") = hit.t;
	register float dst1 asm("f18") = hit.bc.x;
	register float dst2 asm("f19") = hit.bc.y;
	register float dst3 asm("f20") = *(float*)&hit.id;

	asm volatile("triisect %0\n\t" : "+f" (dst0), "+f" (dst1), "+f" (dst2), "+f" (dst3) : "f" (src0), "f" (src1), "f" (src2), "f" (src3), "f" (src4), "f" (src5), "f" (src6), "f" (src7), "f" (src8), "f" (src9), "f" (src10), "f" (src11), "f" (src12), "f" (src13), "f" (src14), "f" (src15), "f" (src16));

	bool is_hit = dst0 < hit.t;
	float _dst3 = dst3;

	hit.t    = dst0;
	hit.bc.x = dst1;
	hit.bc.y = dst2;
	hit.id   = *(uint*)&_dst3;

	return is_hit;
#else
	return rtm::intersect(tri, ray, hit);
#endif
}

struct TreeletStackEntry
{
	float t;
	uint treelet_id;
};

inline bool intersect_treelet(const rtm::PackedTreelet& treelet, const rtm::Ray& ray, rtm::Hit& hit, TreeletStackEntry* treelet_queue, uint& treelet_queue_tail)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;
	uint treelet_stack_start = treelet_queue_tail;

	struct NodeStackEntry
	{
		float t;
		rtm::PackedTreelet::Node::Data data;
	};
	NodeStackEntry node_stack[32]; uint node_stack_size = 1u;

	node_stack[0].t = ray.t_min;
	node_stack[0].data.is_leaf = 0;
	node_stack[0].data.is_child_treelet = 0;
	node_stack[0].data.child_index = 0;

	bool is_hit = false;
	while(node_stack_size)
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) continue;

	TRAV:
		if(!current_entry.data.is_leaf)
		{
			if(!current_entry.data.is_child_treelet)
			{
				const rtm::PackedTreelet::Node& node = treelet.nodes[current_entry.data.child_index];
				float hit_ts[2] = {_intersect(node.aabb[0], ray, inv_d), _intersect(node.aabb[1], ray, inv_d)};
				if(hit_ts[0] < hit_ts[1])
				{
					if(hit_ts[1] < hit.t) node_stack[node_stack_size++] = {hit_ts[1], node.data[1]};
					if(hit_ts[0] < hit.t)
					{
						current_entry = {hit_ts[0], node.data[0]};
						goto TRAV;
					}
				}
				else
				{
					if(hit_ts[0] < hit.t) node_stack[node_stack_size++] = {hit_ts[0], node.data[0]};
					if(hit_ts[1] < hit.t)
					{
						current_entry = {hit_ts[1], node.data[1]};
						goto TRAV;
					}
				}
			}
			else treelet_queue[treelet_queue_tail++] = {current_entry.t, current_entry.data.child_index};
		}
		else
		{
			rtm::Treelet::Triangle* tris = (rtm::Treelet::Triangle*)(&treelet.bytes[current_entry.data.tri_offset]);
			for(uint i = 0; i <= current_entry.data.num_tri; ++i)
			{
				rtm::Treelet::Triangle tri = tris[i];
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

#ifdef __riscv
inline void intersect_buckets(const KernelArgs& args)
{
	bool early = args.use_early;
	bool lhit_delay = args.hit_delay;
	for(WorkItem wi = _lwi(); wi.segment_id != INVALID_SEGMENT_ID; wi = _lwi())
	{
		rtm::Ray ray = wi.bray.ray;
		rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
		if(early)
		{
			if(lhit_delay) _lhit_delay(args.hit_records + wi.bray.id);
			else     hit = _lhit(args.hit_records + wi.bray.id);
		}

		uint treelet_queue_tail = 0;
		TreeletStackEntry treelet_queue[32];

		bool hit_found = intersect_treelet(args.treelets[wi.segment_id], wi.bray.ray, hit, treelet_queue, treelet_queue_tail);
		if(lhit_delay)
		{
			//If we delayed lhit we have to explicitly read the registers it returned to
			register float f28 asm("f28");
			register float f29 asm("f29");
			register float f30 asm("f30");
			register float f31 asm("f31");
			float _f31 = f31;

			rtm::Hit lhit_ret;
			lhit_ret.t = f28;
			lhit_ret.bc.x = f29;
			lhit_ret.bc.y = f30;
			lhit_ret.id = *(uint*)&_f31;

			if(lhit_ret.t <= hit.t)
			{
				hit = lhit_ret; //if the lhit returned something closer use it
				hit_found = false;
			}
		}

		if(hit_found)
		{
			//if we found a new hit store it
			_cshit(hit, args.hit_records + wi.bray.id);
		}

		//Truncate the ray range if possible
		wi.bray.ray.t_max = rtm::min(wi.bray.ray.t_max, hit.t);

		uint order_hint = 0;
		uint treelet_queue_head = 0;
		while(treelet_queue_head < treelet_queue_tail)
		{
			//Always Check here to make sure we cull any unnecessary workitem stores
		 	const TreeletStackEntry& entry = treelet_queue[treelet_queue_head++];
			if(entry.t < hit.t)
			{
				wi.segment_id = entry.treelet_id;
				wi.order_hint = order_hint++; //hints to the scheduler what order we found these in
				_swi(wi);
			}
		}
	}
}
#endif

inline bool intersect(const rtm::PackedTreelet* treelets, const rtm::Ray& ray, rtm::Hit& hit)
{
	TreeletStackEntry treelet_stack[128]; 
	uint treelet_stack_size = 1u;
	treelet_stack[0].treelet_id = 0;
	treelet_stack[0].t = ray.t_min;

	bool hit_found = false;
	while(treelet_stack_size)
	{
		const TreeletStackEntry& entry = treelet_stack[--treelet_stack_size];
		if(entry.t >= hit.t) continue;

		uint treelet_queue_tail = 0;
		TreeletStackEntry treelet_queue[32];
		if(intersect_treelet(treelets[entry.treelet_id], ray, hit, treelet_queue, treelet_stack_size))
			hit_found = true;
	
		for(uint treelet_queue_head = 0; treelet_queue_head < treelet_queue_tail; ++treelet_queue_head)
			treelet_stack[treelet_stack_size++] = treelet_queue[treelet_queue_head];
	}

	return hit_found;
}