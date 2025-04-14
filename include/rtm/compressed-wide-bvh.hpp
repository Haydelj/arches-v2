#pragma once

#include "wide-bvh.hpp"
#include "uvec3.hpp"

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion,  collapsing can be further followed by compression 
//https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs

namespace rtm {

//NVIDIA Compressed Wide Bounding Volume Hierarchy
class NVCWBVH
{
public:
	const static uint WIDTH = WBVH::WIDTH;

	struct alignas(32) Node
	{
		struct MetaData
		{
			uint8_t is_int : 1;
			uint8_t num_prims : 2;
			uint8_t offset : 5;
		};

		const static uint NQ = 8;
		uint64_t base_child_index : 27;
		uint64_t base_prim_index : 29;
		uint64_t e0 : 8;
		float p0;
		float p1;
		float p2;
		uint8_t e1;
		uint8_t e2;
		MetaData mdata[WIDTH];
		QAABB8 qaabb[WIDTH];

		void e(uvec3 v) { e0 = v[0], e1 = v[1], e2 = v[2]; }
		vec3 e() const { return vec3(float32_bf(0, e0, 0).f32, float32_bf(0, e1, 0).f32, float32_bf(0, e2, 0).f32); }
		void p(vec3 v) { p0 = v[0], p1 = v[1], p2 = v[2]; }
		vec3 p() const { return vec3(p0, p1, p2); }

		uint is_int(uint i) const { return mdata[i].is_int; }
		uint num_prims(uint i) const { return mdata[i].num_prims; }
		uint offset(uint i) const { return mdata[i].offset; }

	#ifndef __riscv
		Node(const WBVH::Node& wnode)
		{
			constexpr float denom = 1.0f / ((1 << NQ) - 1);

			AABB aabb;
			base_child_index = ~0u, base_prim_index = ~0u;
			for(uint i = 0; i < WIDTH; ++i)
			{
				if(!wnode.is_valid(i)) continue;
				aabb.add(wnode.aabb[i]);
				if(wnode.data[i].is_int) base_child_index = min(base_child_index, wnode.data[i].child_index);
				else                     base_prim_index = min(base_prim_index, wnode.data[i].prim_index);
			}

			p(aabb.min);

			float32_bf e0((aabb.max.x - p().x) * denom);
			float32_bf e1((aabb.max.y - p().y) * denom);
			float32_bf e2((aabb.max.z - p().z) * denom);
			if(e0.mantisa != 0) e0.mantisa = 0, e0.exp++;
			if(e1.mantisa != 0) e1.mantisa = 0, e1.exp++;
			if(e2.mantisa != 0) e2.mantisa = 0, e2.exp++;
			e(uvec3(e0.exp, e1.exp, e2.exp));

			uint32_t num_children = 0, num_prims = 0;
			vec3 one_over_e(1.0f / e().x, 1.0f / e().y, 1.0f / e().z);
			for(uint i = 0; i < WIDTH; i++)
			{
				if(!wnode.is_valid(i))
				{
					mdata[i].is_int = 0;
					mdata[i].num_prims = 0;
					mdata[i].offset = 0;
					continue;
				}

				mdata[i].is_int = wnode.data[i].is_int;
				mdata[i].num_prims = wnode.data[i].num_prims;
				if(wnode.data[i].is_int) mdata[i].offset = num_children++;
				else                     mdata[i].offset = wnode.data[i].prim_index - base_prim_index;

				qaabb[i].min[0] = floorf((wnode.aabb[i].min.x - p().x) * one_over_e.x);
				qaabb[i].min[1] = floorf((wnode.aabb[i].min.y - p().y) * one_over_e.y);
				qaabb[i].min[2] = floorf((wnode.aabb[i].min.z - p().z) * one_over_e.z);
				qaabb[i].max[0] = ceilf((wnode.aabb[i].max.x - p().x) * one_over_e.x);
				qaabb[i].max[1] = ceilf((wnode.aabb[i].max.y - p().y) * one_over_e.y);
				qaabb[i].max[2] = ceilf((wnode.aabb[i].max.z - p().z) * one_over_e.z);
			}
		}
	#endif
	};

#ifndef __riscv
	std::vector<Node> nodes;

	NVCWBVH(const rtm::WBVH& wbvh)
	{
		sizeof(Node);
		printf("NVCWBVH%d: Building\n", WIDTH);
		assert(wbvh.nodes.size() != 0);

		nodes.clear();
		for(uint wnode_id = 0; wnode_id < wbvh.nodes.size(); ++wnode_id)
		{
			const WBVH::Node& wnode = wbvh.nodes[wnode_id];
			NVCWBVH::Node cwnode(wnode);
			nodes.push_back(cwnode);
		}

		printf("NVCWBVH%d: Size: %.1f MiB\n", WIDTH, (float)sizeof(Node) * nodes.size() / (1 << 20));
	}
#endif
};

inline WBVH::Node decompress(const NVCWBVH::Node& cwnode)
{
	const rtm::vec3 p = cwnode.p();
	const rtm::vec3 e = cwnode.e();

	WBVH::Node wnode;
	for(int i = 0; i < WBVH::WIDTH; i++)
	{
		QAABB8 qaabb = cwnode.qaabb[i];
		wnode.aabb[i].min = vec3(qaabb.min[0], qaabb.min[1], qaabb.min[2]) * e + p;
		wnode.aabb[i].max = vec3(qaabb.max[0], qaabb.max[1], qaabb.max[2]) * e + p;
		wnode.data[i].is_int = cwnode.is_int(i);
		if(cwnode.is_int(i))
		{
			wnode.data[i].child_index = cwnode.base_child_index + cwnode.offset(i);
		}
		else
		{
			wnode.data[i].num_prims = cwnode.num_prims(i);
			wnode.data[i].prim_index = cwnode.base_prim_index + cwnode.offset(i);
		}
	}

	return wnode;
}

}