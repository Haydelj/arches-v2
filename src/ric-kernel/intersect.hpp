#pragma once
#include "stdafx.hpp"
#include "include.hpp"
#include "custom-instr.hpp"

inline void _swi(const uint32_t ray_id)
{
#ifdef __riscv
	register float f0 asm("f0") = *(float*)&ray_id;
	asm volatile("swi f0, 256(x0)" : : "f" (f0));
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

struct TreeletStackEntry
{
	float t;
	uint treelet_id;
};

static rtm::WideTreeletBVH::Treelet::Node decompress(const rtm::WideTreeletBVH::Treelet::Node& node)
{
	return node;
}

static rtm::WideTreeletBVH::Treelet::Node decompress(const rtm::CompressedWideTreeletBVH::Treelet::Node& node)
{
	return node.decompress();
}

template<typename T>
inline bool intersect_treelet(const T& treelet, const rtm::Ray& ray, rtm::Hit& hit, TreeletStackEntry* treelet_queue, uint& treelet_queue_tail)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;
	uint treelet_stack_start = treelet_queue_tail;

	struct NodeStackEntry
	{
		float t;
		rtm::WideTreeletBVH::Treelet::Node::Data data;
	};
	NodeStackEntry node_stack[32]; uint node_stack_size = 1u;

	node_stack[0].t = ray.t_min;
	node_stack[0].data.is_int = 1;
	node_stack[0].data.is_child_treelet = 0;
	node_stack[0].data.child_index = 0;

	bool found_hit = false;
	while(node_stack_size)
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) continue;

		if(current_entry.data.is_int)
		{
			if(current_entry.data.is_child_treelet)
			{
				treelet_queue[treelet_queue_tail++] = {current_entry.t, current_entry.data.child_index};
				continue;
			}

			uint max_insert_depth = node_stack_size;
			const rtm::WideTreeletBVH::Treelet::Node node = decompress(treelet.nodes[current_entry.data.child_index]);
			for(int i = 0; i < rtm::WideTreeletBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float t = rtm::intersect(node.aabb[i], ray, inv_d);
				if(t < hit.t)
				{
					uint j = node_stack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(node_stack[j - 1].t > t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].data = node.data[i];
				}
			}
		}
		else
		{
		#if 1
			for(uint i = 0; i < current_entry.data.num_tri; ++i)
			{
				uint32_t offset = current_entry.data.triangle_index + i * (sizeof(rtm::WideTreeletBVH::Treelet::Triangle) / 4);
				const rtm::WideTreeletBVH::Treelet::Triangle& tri = *(rtm::WideTreeletBVH::Treelet::Triangle*)((uint32_t*)treelet.nodes + offset);
				if(rtm::intersect(tri.tri, ray, hit))
				{
					hit.id = tri.id;
					found_hit = true;
				}
			}
		#else
			if(current_entry.t < hit.t)
			{
				hit.id = current_entry.data.triangle_index;
				hit.t = current_entry.t;
				found_hit = true;
			}
		#endif
		}
	}

	return found_hit;
}

template <typename T>
inline bool intersect(const T* treelets, const rtm::Ray& ray, rtm::Hit& hit)
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
		if(intersect_treelet(treelets[entry.treelet_id], ray, hit, treelet_queue, treelet_queue_tail))
			hit_found = true;
	
		for(uint treelet_queue_head = 0; treelet_queue_head < treelet_queue_tail; ++treelet_queue_head)
			treelet_stack[treelet_stack_size++] = treelet_queue[treelet_queue_head];
	}

	return hit_found;
}

inline bool intersect(const rtm::BVH2::Node* nodes, const rtm::Triangle* tris, const rtm::Ray& ray, rtm::Hit& hit, uint& steps, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::BVH2::Node::Data data;
	};

	NodeStackEntry node_stack[32];
	uint32_t node_stack_size = 1u;
	node_stack[0].t = rtm::intersect(nodes[0].aabb, ray, inv_d);
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
			float t0 = rtm::intersect(nodes[child_index + 0].aabb, ray, inv_d);
			float t1 = rtm::intersect(nodes[child_index + 1].aabb, ray, inv_d);
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
				if(rtm::intersect(tris[id], ray, hit))
				{
					hit.id = id;
					if(first_hit) return true;
					else          found_hit = true;
				}
			}
		}
	}
	while(node_stack_size);

	return found_hit;
}

#ifndef __riscv 
inline void pregen_rays(uint framebuffer_width, uint framebuffer_height, const rtm::Camera camera, const rtm::BVH2& bvh, const rtm::Mesh& mesh, uint bounce, std::vector<rtm::Ray>& rays)
{
	const uint framebuffer_size = framebuffer_width * framebuffer_height;
	printf("Generating bounce %d rays from %d path\n", bounce, framebuffer_size);

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);

	uint num_rays = framebuffer_size;
	for(int index = 0; index < framebuffer_size; index++)
	{
		uint32_t x = index % framebuffer_width;
		uint32_t y = index / framebuffer_width;
		rtm::RNG rng(index);

		rtm::Ray ray = camera.generate_ray_through_pixel(x, y); // Assuming spp = 1
		for(uint i = 0; i < bounce; ++i)
		{
			uint steps = 0;
			rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
			intersect(bvh.nodes.data(), tris.data(), ray, hit, steps);
			if(hit.id != ~0u)
			{
				rtm::vec3 normal = tris[hit.id].normal();
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
