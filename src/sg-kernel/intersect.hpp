#pragma once
#include "stdafx.hpp"
#include "include.hpp"

template<uint32_t FLAGS>
inline SGKernel::HitPacket _traceray(const rtm::Ray& ray, float opacity)
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
	register float src8 asm("f8") = opacity;

	register int dst0 asm("x10");
	register int dst1 asm("x11");
	register int dst2 asm("x12");
	register int dst3 asm("x13");
	register int dst4 asm("x14");
	register int dst5 asm("x15");
	register int dst6 asm("x16");
	register int dst7 asm("x17");
	register int dst8 asm("x18");
	register int dst9 asm("x19");
	register int dst10 asm("x20");
	register int dst11 asm("x21");
	register int dst12 asm("x22");
	register int dst13 asm("x23");
	register int dst14 asm("x24");
	register int dst15 asm("x25");

	asm volatile
	(
		"traceray f10, %16, %25\t\n"
		: "=r" (dst0), "=r" (dst1), "=r" (dst2), "=r" (dst3), "=r" (dst4), "=r" (dst5), "=r" (dst6), "=r" (dst7), "=r" (dst8), "=r" (dst9), "=r" (dst10), "=r" (dst11), "=r" (dst12), "=r"(dst13), "=r"(dst14), "=r"(dst15)
		: "f" (src0), "f" (src1), "f" (src2), "f" (src3), "f" (src4), "f" (src5), "f" (src6), "f" (src7), "f" (src8), "I" (FLAGS)
	);

	SGKernel::HitPacket packet;
	packet.last_t = u_to_f(dst0);
	packet.size = dst1;
	packet.hits[0].u32 = dst2;
	packet.hits[1].u32 = dst3;
	packet.hits[2].u32 = dst4;
	packet.hits[3].u32 = dst5;
	packet.hits[4].u32 = dst6;
	packet.hits[5].u32 = dst7;
	packet.hits[6].u32 = dst8;
	packet.hits[7].u32 = dst9;
	packet.hits[8].u32 = dst10;
	packet.hits[9].u32 = dst11;
	packet.hits[10].u32 = dst12;
	packet.hits[11].u32 = dst13;
	packet.hits[12].u32 = dst14;
	packet.hits[13].u32 = dst15;
	return packet;
#else
	assert(false);
	return 0;
#endif
}

inline float _intersect(const rtm::AABB& aabb, const rtm::Ray& ray, const rtm::vec3& inv_d)
{
#if defined(__riscv) && defined(DS_USE_HARDWARE_INTERSECTORS)
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
#if defined(__riscv) && defined(DS_USE_HARDWARE_INTERSECTORS)
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
	hit.id = f_to_u(_dst3);
	return is_hit;
#else
	return 0;
#endif
}

#ifndef __riscv
inline SGKernel::HitPacket intersect(const rtm::CompressedWideBVH::Node* nodes, const rtm::SphericalGaussian* sgs, const rtm::Ray& ray, float opacity0, uint& steps)
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

	float ts[SGKernel::HitPacket::MAX_HITS];
	SGKernel::HitPacket packet;
	packet.size = 0;
	packet.last_t = ray.t_min;

	float opacity = opacity0;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if((opacity < 0.0001f || packet.size == SGKernel::HitPacket::MAX_HITS) && current_entry.t >= ts[packet.size - 1]) continue;

		if(current_entry.data.is_int)
		{
			uint max_insert_depth = node_stack_size;
			const rtm::WideBVH::Node node = nodes[current_entry.data.child_index].decompress();


			for(int i = 0; i < rtm::WideBVH::WIDTH; i++)
			{
				if(!node.is_valid(i)) continue;

				float t = _intersect(node.aabb[i], ray, inv_d);
				if(t < ray.t_max)
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
			for(uint i = 0; i < current_entry.data.num_prims; ++i)
			{
				float a, t;
				uint32_t prim_id = current_entry.data.prim_index + i;
				if(rtm::intersect(sgs[prim_id], ray, a, t))
				{
					uint j = packet.size++;
					for(; j > 0; --j)
					{
						if(ts[j - 1] < t) break;
						if(j < SGKernel::HitPacket::MAX_HITS)
						{
							packet.hits[j] = packet.hits[j - 1];
							ts[j] = ts[j - 1];
						}
					}
					if(j < SGKernel::HitPacket::MAX_HITS)
					{
						packet.hits[j].id = prim_id;
						packet.hits[j].a = (uint8_t)(a * 255.0f + 0.5);
						ts[j] = t;
					}

					packet.size = rtm::min(SGKernel::HitPacket::MAX_HITS, packet.size);

					opacity = opacity0;
					for(j = 0; j < packet.size; ++j)
					{
						if(opacity < 0.0001f)
						{
							packet.size = j;
							break;
						}
						opacity *= (1.0f - (packet.hits[j].a * (1.0f / 255.0f)));
					}
				}
			}
		}
		steps++;
	}
	while(node_stack_size);

	if(packet.size > 0) packet.last_t = ts[packet.size - 1];
	return packet;
}

/*
inline void pregen_rays(uint framebuffer_width, uint framebuffer_height, const rtm::Camera camera, const rtm::BVH2& bvh, const rtm::Mesh& mesh, uint bounce, std::vector<rtm::Ray>& rays, bool serializeRays = false)
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



	if (serializeRays)
	{
		std::string resultPathName = std::filesystem::current_path().generic_string() + "/pregenRayData.bin";
		std::ofstream file_stream(resultPathName.c_str() , std::ios::binary);
		
		if (file_stream.good())
		{
			file_stream.write((char*)rays.data(), rays.size() * sizeof(rtm::Ray));
			if (file_stream.good())
			{
				printf("Serialized ray data\n");
			}
		}
		else
		{
			fprintf(stderr, "pregen_rays: ofstream failed to open output file\n");
		}

		file_stream.close();
	}
}
*/
#endif
