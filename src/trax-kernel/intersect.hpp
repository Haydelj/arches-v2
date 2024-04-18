#pragma once
#include "stdafx.hpp"
#include "include.hpp"

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
#if defined(__riscv) && defined(USE_HARDWARE_INTERSECTORS)
	register float src0 asm("f0") = ray.o.x;
	register float src1 asm("f1") = ray.o.y;
	register float src2 asm("f2") = ray.o.z;
	register float src3 asm("f3") = ray.t_min;
	register float src4 asm("f4") = inv_d.x;
	register float src5 asm("f5") = inv_d.y;
	register float src6 asm("f6") = inv_d.z;
	register float src7 asm("f7") = ray.t_max;

	register float src8 asm("f8") = aabb.min.x;
	register float src9 asm("f9") = aabb.min.y;
	register float src10 asm("f10") = aabb.min.z;
	register float src11 asm("f11") = aabb.max.x;
	register float src12 asm("f12") = aabb.max.y;
	register float src13 asm("f13") = aabb.max.z;

	float t;
	asm volatile ("boxisect %0" : "=f" (t) : "f" (src0), "f" (src1), "f" (src2), "f" (src3), "f" (src4), "f" (src5), "f" (src6), "f" (src7), "f" (src8), "f" (src9), "f" (src10), "f" (src11), "f" (src12), "f" (src13));

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

	register float src8 asm("f8") = tri.vrts[0].x;
	register float src9 asm("f9") = tri.vrts[0].y;
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

	hit.t = dst0;
	hit.bc.x = dst1;
	hit.bc.y = dst2;
	hit.id = *(uint*)&_dst3;

	return is_hit;
#else
	return rtm::intersect(tri, ray, hit);
#endif
}

inline bool intersect(const rtm::BVH2::Node* nodes, const rtm::Triangle* tris, const rtm::Ray& ray, rtm::Hit& hit, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::BVH2::Node::Data data;
	};

	NodeStackEntry node_stack[32];
	uint32_t node_stack_size = 1u;
	node_stack[0].t = _intersect(nodes[0].aabb, ray, inv_d);
	node_stack[0].data = nodes[0].data;
	
	bool found_hit = false;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) continue;

	POP_SKIP:
		if(!current_entry.data.is_leaf)
		{
			uint child_index = current_entry.data.child_index;
			float t0 = _intersect(nodes[child_index + 0].aabb, ray, inv_d);
			float t1 = _intersect(nodes[child_index + 1].aabb, ray, inv_d);
			if(t0 < hit.t || t1 < hit.t)
			{
				if(t0 < t1)
				{
					current_entry = {t0, nodes[child_index + 0].data};
					if(t1 < hit.t)  node_stack[node_stack_size++] = {t1, nodes[child_index + 1].data};
				}
				else
				{
					current_entry = {t1, nodes[child_index + 1].data};
					if(t0 < hit.t)  node_stack[node_stack_size++] = {t0, nodes[child_index + 0].data};
				}
				goto POP_SKIP;
			}
		}
		else
		{
			for(uint32_t i = 0; i <= current_entry.data.num_prims; ++i)
			{
				uint32_t id = current_entry.data.prim_index + i;
				if(_intersect(tris[id], ray, hit))
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

inline bool intersect(const rtm::PackedBVH2::NodePack* nodes, const rtm::Triangle* tris, const rtm::Ray& ray, rtm::Hit& hit, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::PackedBVH2::NodePack::Data data;
	};

	NodeStackEntry node_stack[32];
	uint32_t node_stack_size = 1u;
	node_stack[0].t = ray.t_min;
	node_stack[0].data.is_leaf = 0;
	node_stack[0].data.child_index = 0;

	bool found_hit = false;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) continue;

	POP_SKIP:
		if(!current_entry.data.is_leaf)
		{
			uint child_index = current_entry.data.child_index;
			float t0 = _intersect(nodes[child_index].aabb[0], ray, inv_d);
			float t1 = _intersect(nodes[child_index].aabb[1], ray, inv_d);
			if(t0 < hit.t || t1 < hit.t)
			{
				if(t0 < t1)
				{
					current_entry = {t0, nodes[child_index].data[0]};
					if(t1 < hit.t)  node_stack[node_stack_size++] = {t1, nodes[child_index].data[1]};
				}
				else
				{
					current_entry = {t1, nodes[child_index].data[1]};
					if(t0 < hit.t)  node_stack[node_stack_size++] = {t0, nodes[child_index].data[0]};
				}
				goto POP_SKIP;
			}
		}
		else
		{
			for(uint32_t i = 0; i <= current_entry.data.num_prims; ++i)
			{
				uint32_t id = current_entry.data.prim_index + i;
				if(_intersect(tris[id], ray, hit))
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

inline bool intersect_treelet(const rtm::PackedTreelet& treelet, const rtm::Ray& ray, rtm::Hit& hit, uint* treelet_stack, uint& treelet_stack_size)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;
	uint treelet_stack_start = treelet_stack_size;

	struct NodeStackEntry
	{
		float hit_t;
		rtm::PackedTreelet::Node::Data data;
	};
	NodeStackEntry node_stack[32]; uint node_stack_size = 1u;

	node_stack[0].hit_t = ray.t_min;
	node_stack[0].data.is_leaf = 0;
	node_stack[0].data.is_child_treelet = 0;
	node_stack[0].data.child_index = 0;

	bool is_hit = false;
	while(node_stack_size)
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.hit_t >= hit.t) continue;

	TRAV:
		if(!current_entry.data.is_leaf)
		{
			if(!current_entry.data.is_child_treelet)
			{
				const rtm::PackedTreelet::Node& node = treelet.nodes[current_entry.data.child_index];
				float hit_ts[2] = {rtm::intersect(node.aabb[0], ray, inv_d), rtm::intersect(node.aabb[1], ray, inv_d)};
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
			else treelet_stack[treelet_stack_size++] = current_entry.data.child_index;
		}
		else
		{
			rtm::PackedTreelet::Triangle* tris = (rtm::PackedTreelet::Triangle*)(&treelet.bytes[current_entry.data.tri_offset]);
			for(uint i = 0; i <= current_entry.data.num_tri; ++i)
			{
				rtm::PackedTreelet::Triangle tri = tris[i];
				if(rtm::intersect(tri.tri, ray, hit))
				{
					hit.id = tri.id;
					is_hit |= true;
				}
			}
		}
	}

	//treelets are pushed in nearest first order so we need to flip such that we pop nearest first
	if(treelet_stack_size > 0)
	{
		uint treelet_stack_end = treelet_stack_size - 1;
		while(treelet_stack_start < treelet_stack_end)
		{
			uint temp = treelet_stack[treelet_stack_start];
			treelet_stack[treelet_stack_start] = treelet_stack[treelet_stack_end];
			treelet_stack[treelet_stack_end] = temp;
			treelet_stack_start++;
			treelet_stack_end--;
		}
	}

	return is_hit;
}

bool inline intersect(const rtm::PackedTreelet* treelets, const rtm::Ray& ray, rtm::Hit& hit)
{
	uint treelet_stack[256]; uint treelet_stack_size = 1u;
	treelet_stack[0] = 0;

	bool is_hit = false;
	while(treelet_stack_size)
	{
		uint treelet_index = treelet_stack[--treelet_stack_size];
		is_hit |= intersect_treelet(treelets[treelet_index], ray, hit, treelet_stack, treelet_stack_size);
	}

	return is_hit;
}

#ifndef __riscv 
inline void pregen_rays(const TRaXKernelArgs& args, uint bounce, std::vector<rtm::Ray>& rays)
{
	printf("Generating bounce %d rays from %d path\n", bounce, args.framebuffer_size);
	uint num_rays = args.framebuffer_size;
	for(int index = 0; index < args.framebuffer_size; index++)
	{
		uint32_t x = index % args.framebuffer_width;
		uint32_t y = index / args.framebuffer_width;
		rtm::RNG rng(index);

		rtm::Ray ray = args.camera.generate_ray_through_pixel(x, y); // Assuming spp = 1
		for(uint i = 0; i < bounce; ++i)
		{
			rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
			intersect(args.nodes, args.tris, ray, hit);
			if(hit.id != ~0u)
			{
				rtm::vec3 normal = args.tris[hit.id].normal();
				ray.o += ray.d * hit.t;
				ray.d = cosine_sample_hemisphere(normal, rng); // generate secondray rays
			}
			else
			{
				num_rays--;
				ray.t_max = ray.t_min;
				break;
			}
		}
		rays[index] = ray;
	}
	printf("Generated %d rays\n", num_rays);
}
#endif
