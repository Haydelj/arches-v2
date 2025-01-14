#pragma once
#include "stdafx.hpp"
#include "include.hpp"
#include "ray-data.hpp"

inline void _swi(const RayData& rb)
{
#ifdef __riscv
	register float f0 asm("f0") = rb.ray.o.x;
	register float f1 asm("f1") = rb.ray.o.y;
	register float f2 asm("f2") = rb.ray.o.z;
	register float f3 asm("f3") = rb.ray.t_min;
	register float f4 asm("f4") = rb.ray.d.x;
	register float f5 asm("f5") = rb.ray.d.y;
	register float f6 asm("f6") = rb.ray.d.z;
	register float f7 asm("f7") = rb.ray.t_max;
	register float f8 asm("f8") = rb.hit.t;
	register float f9 asm("f9") = rb.hit.bc.x;
	register float f10 asm("f10") = rb.hit.bc.y;
	register float f11 asm("f11") = *(float*)&rb.hit.id;
	uint32_t temp = (static_cast<uint32_t>(rb.node_id) << 12) | static_cast<uint32_t>(rb.treelet_id);
	register float f12 asm("f12") = *(float*)&temp;
	temp = (static_cast<uint32_t>(rb.traversal_state) << 28) | static_cast<uint32_t>(rb.global_ray_id);
	register float f13 asm("f13") = *(float*)&temp;
	temp = static_cast<uint32_t>(rb.visited_stack);
	register float f14 asm("f14") = *(float*)&temp;
	temp = static_cast<uint32_t>(rb.visited_stack >> 32);
	register float f15 asm("f15") = *(float*)&temp;
	asm volatile("swi f0, 256(x0)" : : "f" (f0), "f" (f1), "f" (f2), "f" (f3), "f" (f4), "f" (f5), "f" (f6), "f" (f7), "f" (f8), "f" (f9), "f" (f10), "f" (f11), "f" (f12), "f" (f13), "f" (f14), "f" (f15));
#endif
}

inline STRaTAHitReturn _lhit(rtm::Hit* src)
{
#ifdef __riscv
	register float dst0 asm("f27");
	register float dst1 asm("f28");
	register float dst2 asm("f29");
	register float dst3 asm("f30");
	register float dst4 asm("f31");
	asm volatile("lhit %0, 0(%5)" : "=f" (dst0), "=f" (dst1), "=f" (dst2), "=f" (dst3), "=f" (dst4) : "r" (src) : "memory");

	STRaTAHitReturn hit_return;
	hit_return.hit.t = dst0;
	hit_return.hit.bc.x = dst1;
	hit_return.hit.bc.y = dst2;
	float _dst3 = dst3;
	hit_return.hit.id = *(uint*)&_dst3;
	float _dst4 = dst4;
	hit_return.index = *(uint*)&_dst4;

	return hit_return;
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

static rtm::WideBVH::Node decompress(const rtm::WideBVH::Node& node)
{
	return node;
}

static rtm::WideBVH::Node decompress(const rtm::CompressedWideBVH::Node& node)
{
	return node.decompress();
}

inline bool intersect(const rtm::WideBVH::Node* nodes, const rtm::Triangle* tris, const rtm::Ray& ray, rtm::Hit& hit, uint& steps, bool first_hit = false)
{

	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::WideBVH::Node::Data data;
	};

	NodeStackEntry node_stack[32 * (rtm::WideBVH::WIDTH - 1)];
	uint32_t node_stack_size = 1u;

	//Decompress and insert nodes
	node_stack[0].t = ray.t_min;
	node_stack[0].data.is_int = 1;
	node_stack[0].data.child_index = 0;

	bool found_hit = false;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) continue;

		if(current_entry.data.is_int)
		{
			uint max_insert_depth = node_stack_size;
			const rtm::WideBVH::Node node = decompress(nodes[current_entry.data.child_index]);
			for(int i = 0; i < rtm::WideBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float t = _intersect(node.aabb[i], ray, inv_d);
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
			for(uint i = 0; i < current_entry.data.num_prims; ++i)
			{
				uint32_t prim_id = current_entry.data.prim_index + i;
				if(_intersect(tris[prim_id], ray, hit))
				{
					hit.id = prim_id;
					if(first_hit) return true;
					else found_hit = true;
				}
			}
		#else
			if(current_entry.t < hit.t)
			{
				hit.id = current_entry.data.prim_index;
				hit.t = current_entry.t;
				found_hit = true;
			}
		#endif
		}

		steps++;
	}
	while(node_stack_size);

	return found_hit;
}

static rtm::WideTreeletBVH::Treelet::Node decompress(const rtm::WideTreeletBVH::Treelet::Node& node)
{
	return node;
}

inline bool intersect(const rtm::WideTreeletBVH::Treelet* treelets, const rtm::Ray& ray, rtm::Hit& hit, uint& steps, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		uint treelet_id;
		rtm::WideTreeletBVH::Treelet::Node::Data data;
	};

	NodeStackEntry node_stack[32 * (rtm::WideTreeletBVH::WIDTH - 1)];
	uint32_t node_stack_size = 1u;

	//Decompress and insert nodes
	node_stack[0].t = ray.t_min;
	node_stack[0].treelet_id = 0;
	node_stack[0].data.is_int = 1;
	node_stack[0].data.child_index = 0;

	bool found_hit = false;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) continue;

		steps++;
		if(current_entry.data.is_int)
		{
			if(current_entry.data.is_child_treelet)
			{
				current_entry.treelet_id = current_entry.data.child_index;
				current_entry.data.child_index = 0;
			}

			uint max_insert_depth = node_stack_size;
			const rtm::WideTreeletBVH::Treelet::Node node = decompress(treelets[current_entry.treelet_id].nodes[current_entry.data.child_index]);
			for(int i = 0; i < rtm::WideTreeletBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float t = _intersect(node.aabb[i], ray, inv_d);
				if(t < hit.t)
				{
					uint j = node_stack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(node_stack[j - 1].t > t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].treelet_id = current_entry.treelet_id;
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
				const rtm::WideTreeletBVH::Treelet::Triangle& tri = *(rtm::WideTreeletBVH::Treelet::Triangle*)((uint32_t*)treelets[current_entry.treelet_id].nodes + offset);
				if(_intersect(tri.tri, ray, hit))
				{
					hit.id = tri.id;
					if(first_hit) return true;
					else found_hit = true;
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
	while(node_stack_size);

	return found_hit;
}

static rtm::WideTreeletBVH::Treelet::Node decompress(const rtm::CompressedWideTreeletBVH::Treelet::Node& node)
{
	return node.decompress();
}

inline bool intersect(const rtm::CompressedWideTreeletBVH::Treelet* treelets, const rtm::Ray& ray, rtm::Hit& hit, uint& steps, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		uint treelet_id;
		rtm::WideTreeletBVH::Treelet::Node::Data data;
	};

	NodeStackEntry node_stack[32 * (rtm::WideTreeletBVH::WIDTH - 1)];
	uint32_t node_stack_size = 1u;

	//Decompress and insert nodes
	node_stack[0].t = ray.t_min;
	node_stack[0].treelet_id = 0;
	node_stack[0].data.is_int = 1;
	node_stack[0].data.child_index = 0;

	bool found_hit = false;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) continue;

		steps++;
		if(current_entry.data.is_int)
		{
			if(current_entry.data.is_child_treelet)
			{
				current_entry.treelet_id = current_entry.data.child_index;
				current_entry.data.child_index = 0;
			}

			uint max_insert_depth = node_stack_size;
			const rtm::WideTreeletBVH::Treelet::Node node = decompress(treelets[current_entry.treelet_id].nodes[current_entry.data.child_index]);
			for(int i = 0; i < rtm::WideTreeletBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float t = _intersect(node.aabb[i], ray, inv_d);
				if(t < hit.t)
				{
					uint j = node_stack_size++;
					for(; j > max_insert_depth; --j)
					{
						if(node_stack[j - 1].t > t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].treelet_id = current_entry.treelet_id;
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
				const rtm::WideTreeletBVH::Treelet::Triangle& tri = *(rtm::WideTreeletBVH::Treelet::Triangle*)((uint32_t*)treelets[current_entry.treelet_id].nodes + offset);
				if(_intersect(tri.tri, ray, hit))
				{
					hit.id = tri.id;
					if(first_hit) return true;
					else found_hit = true;
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
	while(node_stack_size);

	return found_hit;
}

#ifndef __riscv 
inline void pregen_rays(uint framebuffer_width, uint framebuffer_height, const rtm::Camera camera, const rtm::BVH2& bvh, const rtm::Mesh& mesh, uint bounce, std::vector<rtm::Ray>& rays)
{
	uint num_rays = framebuffer_width * framebuffer_height;
	printf("Generating bounce %d rays from %d path\n", bounce, num_rays);

	std::vector<rtm::Triangle> tris;
	mesh.get_triangles(tris);

	for(int index = 0; index < num_rays; index++)
	{
		uint32_t x = index % framebuffer_width;
		uint32_t y = index / framebuffer_width;
		rtm::RNG rng(index);

		rtm::Ray ray = camera.generate_ray_through_pixel(x, y); // Assuming spp = 1
		for(uint i = 0; i < bounce; ++i)
		{
			rtm::Hit hit(ray.t_max, rtm::vec2(0.0f), ~0u);
			intersect(bvh.nodes.data(), tris.data(), ray, hit);
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
