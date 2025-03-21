#pragma once
#include "stdafx.hpp"
#include "include.hpp"

inline void _swi(const STRaTARTKernel::RayData& rb)
{
#ifdef __riscv

	const float* f = (float*)&rb;
	register float f0 asm("f0") = f[0];
	register float f1 asm("f1") = f[1];
	register float f2 asm("f2") = f[2];
	register float f3 asm("f3") = f[3];
	register float f4 asm("f4") = f[4];
	register float f5 asm("f5") = f[5];
	register float f6 asm("f6") = f[6];
	register float f7 asm("f7") = f[7];
	register float f8 asm("f8") = f[8];
	register float f9 asm("f9") = f[9];
	register float f10 asm("f10") = f[10];
	register float f11 asm("f11") = f[11];
	register float f12 asm("f12") = f[12];
	register float f13 asm("f13") = f[13];
	register float f14 asm("f14") = f[14];
	register float f15 asm("f15") = f[15];
	asm volatile("swi f0, 256(x0)" : : "f" (f0), "f" (f1), "f" (f2), "f" (f3), "f" (f4), "f" (f5), "f" (f6), "f" (f7), "f" (f8), "f" (f9), "f" (f10), "f" (f11), "f" (f12), "f" (f13), "f" (f14), "f" (f15));
#endif
}

inline STRaTARTKernel::HitReturn _lhit(uint priority)
{
#ifdef __riscv
	register float dst0 asm("f27");
	register float dst1 asm("f28");
	register float dst2 asm("f29");
	register float dst3 asm("f30");
	register float dst4 asm("f31");
	asm volatile("lhit %0, 0(%5)" : "=f" (dst0), "=f" (dst1), "=f" (dst2), "=f" (dst3), "=f" (dst4) : "r" (priority) : "memory");

	STRaTARTKernel::HitReturn hit_return;
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

inline bool intersect(const rtm::CompressedWideTreeletBVH::Treelet* treelets, const rtm::Ray& ray, rtm::Hit& hit, uint& steps, bool first_hit = false)
{
#define ENABLE_PRINTS (false)

	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		bool is_last;
		rtm::WideTreeletBVH::Treelet::Node::Data data;
	};

	NodeStackEntry root_node;
	root_node.t = ray.t_min;
	root_node.is_last = 0;
	root_node.data.is_int = 1;
	root_node.data.is_child_treelet = 0;
	root_node.data.child_index = 0;

	uint treelet_id = 0;
	NodeStackEntry node_stack[32 * (rtm::WideTreeletBVH::WIDTH - 1)];
	uint32_t node_stack_size = 1u;
	node_stack[0] = root_node;

	rtm::RestartTrail restart_trail;
	uint level = 0;

	bool update_restart_trail = false;
	bool found_hit = false;
	while(true)
	{
		NodeStackEntry current_entry;
		if(update_restart_trail)
		{
			uint parent_level = restart_trail.find_parent_level(level);
			if(parent_level == ~0u)
			{
				//if(ENABLE_PRINTS)
				//	printf("RAY_COMPLETE\n");
				break;
			}

			restart_trail.set(parent_level, restart_trail.get(parent_level) + 1);
			restart_trail.clear(parent_level + 1);

			if(node_stack_size == 0)
			{
				//restart
				//if(ENABLE_PRINTS)
				//	printf("RESTART\n");
				treelet_id = 0;
				current_entry = root_node;
				level = 0;
			}
			else
			{
				current_entry = node_stack[--node_stack_size];
				if(current_entry.is_last)
					restart_trail.set(parent_level, rtm::RestartTrail::N);
				level = parent_level + 1;
			}
		}
		else
		{
			current_entry = node_stack[--node_stack_size];
		}

		if(current_entry.t >= hit.t)
		{
			update_restart_trail = true;
			//if(ENABLE_PRINTS)
			//	printf("POP_CULL\n");
			continue;
		}

		steps++;
		if(current_entry.data.is_int)
		{
			if(current_entry.data.is_child_treelet)
			{
				node_stack_size = 0;
				treelet_id = current_entry.data.child_index;

				current_entry.data.is_int = 1;
				current_entry.data.is_child_treelet = 0;
				current_entry.data.child_index = 0;

				//if(ENABLE_PRINTS)
				//	printf("TREELET_JUMP: %d\n", treelet_id);
			}
			
			//if(ENABLE_PRINTS)
			//	printf("NODE_ISSUE: %d:%d\n", treelet_id, current_entry.data.child_index);

			uint k = restart_trail.get(level);

			uint nodes_pushed = 0;
			const rtm::WideTreeletBVH::Treelet::Node node = treelets[treelet_id].nodes[current_entry.data.child_index].decompress();
			for(uint i = 0; i < rtm::WideTreeletBVH::WIDTH; ++i)
			{
				if(!node.is_valid(i)) continue;

				float t = _intersect(node.aabb[i], ray, inv_d);
				if(t < hit.t)
				{
					uint j = node_stack_size + nodes_pushed++;
					for(; j > node_stack_size; --j)
					{
						if(node_stack[j - 1].t > t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].is_last = false;
					node_stack[j].data = node.data[i];
				}
			}

			if(k == rtm::RestartTrail::N) nodes_pushed = 1;
			else                     nodes_pushed -= rtm::min(nodes_pushed, k);

			if(nodes_pushed == 0)
			{
				update_restart_trail = true;
			}
			else
			{
				update_restart_trail = false;
				if(nodes_pushed == 1) restart_trail.set(level, rtm::RestartTrail::N);
				else                  node_stack[node_stack_size].is_last = true;
				node_stack_size += nodes_pushed;
				level++;
			}
		}
		else
		{
			//if(ENABLE_PRINTS)
			//	printf("TRI: %d:%d\n", current_entry.data.triangle_index, current_entry.data.num_tri);
		#if 1
			for(uint i = 0; i < current_entry.data.num_tri; ++i)
			{
				uint32_t offset = current_entry.data.triangle_index + i * (sizeof(rtm::WideTreeletBVH::Treelet::Triangle) / 4);
				const rtm::WideTreeletBVH::Treelet::Triangle& tri = *(rtm::WideTreeletBVH::Treelet::Triangle*)((uint32_t*)treelets[treelet_id].nodes + offset);
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
			update_restart_trail = true;
		}
	}

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
