#pragma once

#include "wide-bvh-strata.hpp"

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion,  collapsing can be further followed by compression 
//https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs

namespace rtm {

class CompressedWideBVHSTRaTA
{
public:
	const static uint WIDTH = WideBVHSTRaTA::WIDTH;

	struct alignas(64) Node
	{
		struct CompressedData
		{
			uint8_t is_int    : 1;
			uint8_t num_prims : 2;
			uint8_t offset    : 5;
		};

		//Quantized aabb
		struct CompressedAABB
		{
			struct
			{
				uint8_t x;
				uint8_t y;
				uint8_t z;
			}
			min;

			struct
			{
				uint8_t x;
				uint8_t y;
				uint8_t z;
			}
			max;

			//AABB decompress(const vec3& p, const uint8_t e[3]) {}
		};

		uint64_t base_child_index    : 28; //base offset in node array
		uint64_t base_triangle_index : 28; //base offset in primitive array
		uint64_t e0 : 8;
		rtm::vec3 p;
		uint8_t e1;
		uint8_t e2;
		CompressedData cdata[WIDTH];
		CompressedAABB caabb[WIDTH];

		Node() : p(0.0f, 0.0f, 0.0f), e0 {0}, e1{ 0 }, e2{ 0 }, base_child_index(0), base_triangle_index(0)
		{
			for(int i = 0; i < WIDTH; i++)
			{
				cdata[i].is_int = 0u;
				cdata[i].num_prims = 0u;
				cdata[i].offset = 0u;
				caabb[i].min.x = 0u;
				caabb[i].min.y = 0u;
				caabb[i].min.z = 0u;
				caabb[i].max.x = 0u;
				caabb[i].max.y = 0u;
				caabb[i].max.z = 0u;
			}
		};
		~Node() = default;

		WideBVHSTRaTA::Node decompress() const
		{
			WideBVHSTRaTA::Node node;

			uint32_t e0u32 = uint32_t(e0) << 23;
			uint32_t e1u32 = uint32_t(e1) << 23;
			uint32_t e2u32 = uint32_t(e2) << 23;

			float e0f = *reinterpret_cast<float*>(&e0u32);
			float e1f = *reinterpret_cast<float*>(&e1u32);
			float e2f = *reinterpret_cast<float*>(&e2u32);

			for(int i = 0; i < WIDTH; i++)
			{
				node.aabb[i].min.x = p.x + e0f * float(caabb[i].min.x);
				node.aabb[i].min.y = p.y + e1f * float(caabb[i].min.y);
				node.aabb[i].min.z = p.z + e2f * float(caabb[i].min.z);
				node.aabb[i].max.x = p.x + e0f * float(caabb[i].max.x);
				node.aabb[i].max.y = p.y + e1f * float(caabb[i].max.y);
				node.aabb[i].max.z = p.z + e2f * float(caabb[i].max.z);

				node.data[i].is_int = cdata[i].is_int;
				if(cdata[i].is_int)
				{
					node.data[i].child_index = (uint32_t)base_child_index + cdata[i].offset;
				}
				else
				{
					node.data[i].num_prims = cdata[i].num_prims;
					node.data[i].prim_index = (uint32_t)base_triangle_index + cdata[i].offset;
				}
			}

			return node;
		}
	};

#ifndef __riscv
	std::vector<Node> nodes;
	std::vector<uint> indices; //flat triangle array indices

public:
	/// <summary>
	/// Creates compressed wide bvh with branching factor equal to WIDE_BVH_SIZE from a wide bvh
	///  Note: bvh2 should have max 1 prim per leaf node in order for this to work
	/// </summary>
	/// <param name="_wbvh"></param>
	CompressedWideBVHSTRaTA(const rtm::WideBVHSTRaTA& wbvh)
	{
		printf("Building Compressed Wide BVH for STRaTA\n");

		assert(wbvh.nodes.size() != 0);
		indices.resize(wbvh.indices.size());
		std::copy(wbvh.indices.begin(), wbvh.indices.end(), indices.begin());

		nodes.clear();
		for(const WideBVHSTRaTA::Node& wnode : wbvh.nodes)
		{
			AABB wnode_aabb;
			uint base_index_child = ~0u;
			uint base_triangle_index = ~0u;
			for(uint i = 0; i < WIDTH; ++i)
			{
				if(!wnode.is_valid(i)) continue;

				wnode_aabb.add(wnode.aabb[i]);
				if(wnode.data[i].is_int)
				{
					base_index_child = min(base_index_child, wnode.data[i].child_index);
				}
				else
				{
					base_triangle_index = min(base_triangle_index, wnode.data[i].prim_index);
				}
			}

			nodes.emplace_back();
			CompressedWideBVHSTRaTA::Node& cwnode = nodes.back();

			cwnode.p = wnode_aabb.min;
			constexpr int Nq = 8;
			constexpr float denom = 1.0f / float((1 << Nq) - 1);

			const AABB& aabb = wnode_aabb;
			vec3 e(
				exp2f(ceilf(log2f((aabb.max.x - aabb.min.x) * denom))),
				exp2f(ceilf(log2f((aabb.max.y - aabb.min.y) * denom))),
				exp2f(ceilf(log2f((aabb.max.z - aabb.min.z) * denom)))
			);

			vec3 one_over_e = vec3(1.0f / e.x, 1.0f / e.y, 1.0f / e.z);

			uint32_t u_ex = {};
			uint32_t u_ey = {};
			uint32_t u_ez = {};

			memcpy(&u_ex, &e.x, sizeof(float));
			memcpy(&u_ey, &e.y, sizeof(float));
			memcpy(&u_ez, &e.z, sizeof(float));

			assert((u_ex & 0b10000000011111111111111111111111) == 0);
			assert((u_ey & 0b10000000011111111111111111111111) == 0);
			assert((u_ez & 0b10000000011111111111111111111111) == 0);

			//Store 8 bit exponent
			cwnode.e0 = u_ex >> 23;
			cwnode.e1 = u_ey >> 23;
			cwnode.e2 = u_ez >> 23;

			cwnode.base_child_index = base_index_child;
			cwnode.base_triangle_index = base_triangle_index;

			uint32_t num_triangles = 0;
			uint32_t num_children = 0;

			//Loop over all the uncompressed child nodes
			for(int i = 0; i < WIDTH; i++)
			{
				if(!wnode.is_valid(i))
				{
					cwnode.cdata[i].is_int = 0;
					cwnode.cdata[i].num_prims = 0;
					continue;
				}

				cwnode.caabb[i].min.x = uint8_t(floorf((wnode.aabb[i].min.x - cwnode.p.x) * one_over_e.x));
				cwnode.caabb[i].min.y = uint8_t(floorf((wnode.aabb[i].min.y - cwnode.p.y) * one_over_e.y));
				cwnode.caabb[i].min.z = uint8_t(floorf((wnode.aabb[i].min.z - cwnode.p.z) * one_over_e.z));
					   
				cwnode.caabb[i].max.x = uint8_t(ceilf((wnode.aabb[i].max.x - cwnode.p.x) * one_over_e.x));
				cwnode.caabb[i].max.y = uint8_t(ceilf((wnode.aabb[i].max.y - cwnode.p.y) * one_over_e.y));
				cwnode.caabb[i].max.z = uint8_t(ceilf((wnode.aabb[i].max.z - cwnode.p.z) * one_over_e.z));

				cwnode.cdata[i].is_int = wnode.data[i].is_int;
				cwnode.cdata[i].num_prims = wnode.data[i].num_prims;
				
				if(wnode.data[i].is_int)
				{
					cwnode.cdata[i].offset = num_children++;
				}
				else
				{
					cwnode.cdata[i].offset = wnode.data[i].prim_index - base_triangle_index;
				}
			}
		}

		printf("Built Compressed Wide BVH for STRaTA\n");
	}

	~CompressedWideBVHSTRaTA() = default;
#endif
};

}