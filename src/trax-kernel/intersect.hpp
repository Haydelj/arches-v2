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
	assert(false);
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

constexpr uint PACKET_SIZE = 16;
inline uint64_t intersect(const rtm::PackedBVH2::Node* nodes, const rtm::Triangle* tris, const rtm::Frustum& ray_packet, rtm::Hit hit_buffer[PACKET_SIZE])
{

	struct NodeStackEntry
	{
		uint64_t mask;
		rtm::BVH2::Node::Data data;
	};

	NodeStackEntry node_stack[32];
	uint32_t node_stack_size = 1u;
	node_stack[0].mask = ~0;
	node_stack[0].data.is_leaf = 0;
	node_stack[0].data.child_index = 0;
	
	uint64_t hit_mask = 0;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];

	POP_SKIP:
		if(!current_entry.data.is_leaf)
		{
			uint child_index = current_entry.data.child_index;

			uint64_t mask0 = 0;
			uint64_t mask1 = 0;
			float t0 = ray_packet.t_max;
			float t1 = ray_packet.t_max;
			for(uint i = 0; i < PACKET_SIZE; ++i)
			{
				if((current_entry.mask >> i) & 0x1ull)
				{
					rtm::Ray ray{ray_packet.o, ray_packet.t_min, ray_packet.d, ray_packet.t_max};
					ray.d += ray_packet.dx * (i % 4) + ray_packet.dy * (i / 4);
					rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

					float _t0 = _intersect(nodes[child_index].aabb[0], ray, inv_d);
					if(_t0 < hit_buffer[i].t) 
					{
						t0 = rtm::min(t0, _t0);
						mask0 |= 0x1ull << i;
					}

					float _t1 = _intersect(nodes[child_index].aabb[1], ray, inv_d);
					if(_t1 < hit_buffer[i].t) 
					{
						t1 = rtm::min(t1, _t1);
						mask1 |= 0x1ull << i;
					}
				}
			}

			if(mask0 || mask1)
			{
				if(t0 < t1)
				{
					current_entry = {mask0, nodes[child_index].data[0]};
					if(mask1) node_stack[node_stack_size++] = {mask1, nodes[child_index].data[1]};
				}
				else
				{
					current_entry = {mask1, nodes[child_index].data[1]};
					if(mask0) node_stack[node_stack_size++] = {mask0, nodes[child_index].data[0]};
				}
				goto POP_SKIP;
			}
		}
		else
		{
			for(uint32_t i = 0; i <= current_entry.data.num_prims; ++i)
			{
				uint32_t id = current_entry.data.prim_index + i;
				for(uint i = 0; i < PACKET_SIZE; ++i)
				{
					if((current_entry.mask >> i) & 0x1ull)
					{
						rtm::Ray ray{ray_packet.o, ray_packet.t_min, ray_packet.d, ray_packet.t_max};
						ray.d += ray_packet.dx * (i % 4) + ray_packet.dy * (i / 4);
						if(_intersect(tris[id], ray, hit_buffer[i]))
						{
							hit_buffer[i].id = id;
							hit_mask |= 0x1ull << i;
						}
					}
				}
			}
		}
	} 
	while(node_stack_size);

	return hit_mask;
}

inline bool intersect(const rtm::PackedBVH2::Node* nodes, const rtm::Triangle* tris, const rtm::Ray& ray, rtm::Hit& hit, uint& steps, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;
	steps = 0;

	struct NodeStackEntry
	{
		float t;
		rtm::BVH2::Node::Data data;
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
			if (current_entry.t < hit.t)
			{
				hit.id = current_entry.data.prim_index;
				hit.t = current_entry.t;
			}

			//for(uint32_t i = 0; i <= current_entry.data.num_prims; ++i)
			//{
			//	steps++;
			//	uint32_t id = current_entry.data.prim_index + i;
			//	if(_intersect(tris[id], ray, hit))
			//	{
			//		hit.id = id;
			//		if(first_hit) return true;
			//		else          found_hit = true;
			//	}
			//}
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


inline bool intersect(const rtm::CompressedWideBVH::Node* nodes, const rtm::Triangle* tris, const rtm::Ray& ray, rtm::Hit& hit, uint& steps ,bool first_hit = false)
{
	steps = 0;
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::BVH2::Node::Data data;
	};

	NodeStackEntry node_stack[32 * rtm::N_ARY_SZ];
	uint32_t node_stack_size = 1u;

	//Decompress and insert nodes
	node_stack[0].t = ray.t_min;
	node_stack[0].data.is_leaf = false;
	node_stack[0].data.child_index = 0;
	bool found_hit = false;

	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if (current_entry.t >= hit.t) continue;
		
		if (!current_entry.data.is_leaf)
		{
			int childCount;
			rtm::BVH2::Node dnodes[rtm::N_ARY_SZ];
			nodes[current_entry.data.child_index].decompress(dnodes, childCount);

			uint max_insert_depth = node_stack_size;
			for (int i = 0; i < childCount; i++)
			{
				float t = _intersect(dnodes[i].aabb, ray, inv_d);
				if (t < hit.t)
				{
#if 1
					uint j = node_stack_size++;
					for (; j > max_insert_depth; --j)
					{
						if (node_stack[j - 1].t > t) break;
						node_stack[j] = node_stack[j - 1];
					}
					node_stack[j].t = t;
					node_stack[j].data = dnodes[i].data;
#else

					node_stack[node_stack_size].t = t;
					node_stack[node_stack_size].data = dnodes[i].data;
					node_stack_size++;
#endif
				}
			}
		}
		else
		{
			//if (current_entry.t < hit.t)
			//{
			//	hit.id = current_entry.data.prim_index;
			//	hit.t = current_entry.t;
			//}

			for (uint32_t i = 0; i <= current_entry.data.num_prims; i++)
			{
				uint32_t triID = current_entry.data.prim_index + i;
				steps++;
				if (_intersect(tris[triID], ray, hit))
				{
					hit.id = triID;
					if (first_hit)	return true;
					else			found_hit = true;
				}
			}
		}
	} while (node_stack_size);
	return found_hit;
}

#ifndef __riscv
inline bool intersect(const rtm::WideBVH::Node* bvh8,
	const rtm::Triangle* tris, const rtm::Ray& ray, rtm::Hit& hit, uint& steps, bool first_hit = false)
{
	rtm::vec3 inv_d = rtm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		rtm::BVH2::Node::Data data;
		int node_index;
		int child_count;
	};

	NodeStackEntry node_stack[32 * rtm::N_ARY_SZ];
	uint32_t node_stack_size = 1u;

	//Decompress and insert nodes
	node_stack[0].t = ray.t_min;
	node_stack[0].data.is_leaf = false;
	node_stack[0].node_index = 0;
	node_stack[0].child_count = rtm::N_ARY_SZ;

	bool found_hit = false;


	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if (current_entry.t >= hit.t) continue;		//if node out of ray interval, skip and continue 

		if (!current_entry.data.is_leaf)
		{
			rtm::WideBVH::Node current_node8 = bvh8[current_entry.node_index];

			for (int i = 0; i < current_entry.child_count; i++)
			{
				rtm::BVH2::Node childNode = current_node8.nodeArray[i];
				float t = _intersect(childNode.aabb, ray, inv_d);		//intersects children
				if (t < hit.t)											//If valid interval distance then push onto traversal stack
				{

					node_stack[node_stack_size].t = t;
					node_stack[node_stack_size].node_index = current_node8.base_index_child + childNode.data.child_index; //hack to store child nodes index in global node array

					if (!childNode.data.is_leaf)
					{
						node_stack[node_stack_size].child_count = bvh8[node_stack[node_stack_size].node_index].childCount;
					}
					node_stack[node_stack_size++].data = childNode.data;
				}
			}
		}
		else
		{
			if (current_entry.t < hit.t)
			{
				hit.id = current_entry.data.prim_index;
				hit.t = current_entry.t;
			}
			/*
			for (int i = 0; i <= current_entry.data.num_prims; i++)
			{
				uint32_t triID = current_entry.data.child_index + i;
				if (_intersect(tris[triID], ray, hit))
				{

					hit.id = triID;

					if (first_hit)	return true;
					else			found_hit = true;

				}
			}*/
		}
	} while (node_stack_size);

	return found_hit;
}

#endif

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
