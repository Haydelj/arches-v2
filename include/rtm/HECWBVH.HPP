#pragma once

#include "wide-bvh.hpp"
#include "bits.hpp"
#include "uvec3.hpp"

namespace rtm {

//High Efficency Cowmpressed Wide Bounding Volume Hierarchy
class HECWBVH
{
public:
	const static uint WIDTH = WBVH::WIDTH;

	struct alignas(32) Node
	{
		const static uint NQ = 7;
	
		uint32_t base_index : 24;
		uint32_t max_exp    : 8;
		//uint64_t base_child_index : 27;
		//uint64_t base_prim_index : 29;
		//uint64_t max_exp : 8;
		uint16_t e0 : 5;
		uint16_t e1 : 5;
		uint16_t e2 : 5;
		uint16_t es : 1;
		int16_t p0;
		int16_t p1;
		int16_t p2;

		const static uint BIT_PER_BOX = NQ * 6;
		BitArray<3 * WIDTH + BIT_PER_BOX * WIDTH> bit_array;

		void set_e(uvec3 e)
		{
			uint min_exp = max_exp - 31;
			e0 = max_exp - max(e[0], min_exp);
			e1 = max_exp - max(e[1], min_exp);
			e2 = max_exp - max(e[2], min_exp);
		}
		void set_p(vec3 p, int round = -1)
		{
			p0 = f32_to_i16(p[0], max_exp, round);
			p1 = f32_to_i16(p[1], max_exp, round);
			p2 = f32_to_i16(p[2], max_exp, round);
		}
		void set_qaabb(uint i, QAABB16 qaabb)
		{
			for(uint j = 0; j < 3; ++j)
			{
				bit_array.write(3 * WIDTH + BIT_PER_BOX * i + NQ * (j + 0), NQ, qaabb.min[j]);
				bit_array.write(3 * WIDTH + BIT_PER_BOX * i + NQ * (j + 3), NQ, qaabb.max[j]);
			}
		}
		void set_int(uint i, uint v) { bit_array.write(i, 1, v); }
		void set_num_prims(uint i, uint v) { bit_array.write(WIDTH + i * 2, 2, v); }

		vec3 get_e() const { return vec3(float32_bf(0, max_exp - e0, 0).f32, float32_bf(0, max_exp - e1, 0).f32, float32_bf(0, max_exp - e2, 0).f32); }
		vec3 get_p() const { return vec3(i16_to_f32(p0, max_exp), i16_to_f32(p1, max_exp), i16_to_f32(p2, max_exp)); }
		QAABB16 get_qaabb(uint i) const
		{
			QAABB16 qaabb;
			for(uint j = 0; j < 3; ++j)
			{
				qaabb.min[j] = bit_array.read(3 * WIDTH + BIT_PER_BOX * i + NQ * (j + 0), NQ);
				qaabb.max[j] = bit_array.read(3 * WIDTH + BIT_PER_BOX * i + NQ * (j + 3), NQ);
			}
			return qaabb;
		}
		uint get_int(uint i) const { return bit_array.read(i, 1); }
		uint get_num_prims(uint i) const { return bit_array.read(WIDTH + i * 2, 2); }
		uint offset(uint i) const
		{
			uint offset = 0;
			for(uint j = 0; j < i; ++j)
				if(get_int(j)) offset += 1;
				else           offset += WBVH::LEAF_RATIO * get_num_prims(j);
			return offset;
		}

		Node() = default;

	#ifndef __riscv
		Node(const WBVH::Node& wnode, uint8_t max_exp) : max_exp(max_exp)
		{
			sizeof(Node);
			constexpr float denom = 1.0f / ((1 << NQ) - 1);

			AABB aabb;
			uint base_prim_index = ~0u;
			base_index = ~0u;
			for(uint i = 0; i < WIDTH; ++i)
			{
				if(!wnode.is_valid(i)) continue;
				aabb.add(wnode.aabb[i]);
				if(wnode.data[i].is_int) base_index = min(base_index, wnode.data[i].child_index);
				else                     base_prim_index = min(base_prim_index, wnode.data[i].prim_index);
			}

			set_p(aabb.min, -1);
			rtm::vec3 p_min = get_p();

			float32_bf e0((aabb.max.x - p_min.x) * denom);
			float32_bf e1((aabb.max.y - p_min.y) * denom);
			float32_bf e2((aabb.max.z - p_min.z) * denom);
			if(e0.mantisa != 0) e0.mantisa = 0, e0.exp++;
			if(e1.mantisa != 0) e1.mantisa = 0, e1.exp++;
			if(e2.mantisa != 0) e2.mantisa = 0, e2.exp++;
			set_e(uvec3(e0.exp, e1.exp, e2.exp));

			uint32_t num_children = 0, num_prims = 0;
			vec3 one_over_e(1.0f / get_e().x, 1.0f / get_e().y, 1.0f / get_e().z);
			for(uint i = 0; i < WIDTH; i++)
			{
				if(!wnode.is_valid(i))
				{
					set_int(i, 0);
					set_num_prims(i, 0);
					continue;
				}

				set_int(i, wnode.data[i].is_int);
				set_num_prims(i, wnode.data[i].num_prims);

				QAABB16 qaabb16;
				qaabb16.min[0] = floorf((wnode.aabb[i].min.x - get_p().x) * one_over_e.x);
				qaabb16.min[1] = floorf((wnode.aabb[i].min.y - get_p().y) * one_over_e.y);
				qaabb16.min[2] = floorf((wnode.aabb[i].min.z - get_p().z) * one_over_e.z);
				qaabb16.max[0] = ceilf((wnode.aabb[i].max.x - get_p().x) * one_over_e.x);
				qaabb16.max[1] = ceilf((wnode.aabb[i].max.y - get_p().y) * one_over_e.y);
				qaabb16.max[2] = ceilf((wnode.aabb[i].max.z - get_p().z) * one_over_e.z);
				set_qaabb(i, qaabb16);
				QAABB16 _qaabb16 = get_qaabb(i);
				for(uint j = 0; j < 3; ++j)
				{
					assert(_qaabb16.min[j] == qaabb16.min[j]);
					assert(_qaabb16.max[j] == qaabb16.max[j]);
				}
			}
		}
	#endif
	};

#ifndef __riscv
	std::vector<Node> nodes;
	HECWBVH(const rtm::WBVH& wbvh, const uint8_t* prims, uint prim_size)
	{
		printf("HE%dCWBVH%d: Building\n", Node::NQ, WIDTH);
		if(prim_size / sizeof(Node) != WBVH::LEAF_RATIO)
		{
			printf("HE%dCWBVH%d: Warning incorrect leaf ratio!!!\n", Node::NQ, WIDTH);
			__debugbreak();
		}

		uint8_t max_exp = 0;
		for(uint i = 0; i < rtm::WBVH::WIDTH; ++i)
		{
			if(!wbvh.nodes[0].is_valid(i)) continue;
			const AABB& aabb = wbvh.nodes[0].aabb[i];
			for(uint j = 0; j < 6; ++j)
			{
				float32_bf f(((float*)&aabb)[j]);
				max_exp = max(f.exp, max_exp);
			}
		}
		max_exp++;

		uint num_prims = 0, num_nodes = 0, dummy_blocks = 0;
		if(prims)
		{
			uint leaf_ratio = prim_size / sizeof(Node);

			nodes.clear();
			std::map<uint, uint> node_assignments;
			std::map<uint, uint> prim_assignments;
			node_assignments[0] = 0; nodes.emplace_back();

			for(uint wnode_id = 0; wnode_id < wbvh.nodes.size(); ++wnode_id)
			{
				WBVH::Node wnode = wbvh.nodes[wnode_id];

				//sort children
				{
					uint internal_nodes = 0;
					uint remap_inds[WIDTH];
					for(uint i = 0; i < WIDTH; ++i)
					{
						remap_inds[i] = i;
						if(wnode.data[i].is_int)
							internal_nodes++;
					}

					uint first_prim_offset = 0;
					while(((nodes.size() + first_prim_offset) % leaf_ratio) != 0) 
						first_prim_offset++;

					while(first_prim_offset > internal_nodes)
					{
						nodes.emplace_back();
						first_prim_offset--;
						dummy_blocks++;
					}

					std::sort(remap_inds, remap_inds + WIDTH, [&](uint a, uint b) -> bool
					{
						if(!wnode.is_valid(a)) return false;
						return wnode.data[a].is_int > wnode.data[b].is_int;
					});

					std::sort(remap_inds + first_prim_offset, remap_inds + WIDTH, [&](uint a, uint b) -> bool
					{
						if(!wnode.is_valid(a)) return false;
						return wnode.data[a].is_int < wnode.data[b].is_int;
					});

					WBVH::Node temp_node = wnode;
					for(uint i = 0; i < WIDTH; ++i)
					{
						wnode.aabb[i] = temp_node.aabb[remap_inds[i]];
						wnode.data[i] = temp_node.data[remap_inds[i]];
					}
				}

				uint cwnode_id = node_assignments[wnode_id];
				Node cwnode(wnode, max_exp);
				cwnode.base_index = nodes.size();
				nodes[cwnode_id] = cwnode;

				for(uint i = 0; i < WIDTH; ++i)
				{
					if(!wnode.is_valid(i)) continue;
					if(wnode.data[i].is_int)
					{
						node_assignments[wnode.data[i].child_index] = nodes.size();
						nodes.emplace_back();
					}
					else
					{
						for(uint j = 0; j < wnode.data[i].num_prims; ++j)
						{
							if((nodes.size() % leaf_ratio) != 0) printf("HE%dCWBVH%d: Warning unaligned leaf node!!!\n", Node::NQ, WIDTH);
							prim_assignments[wnode.data[i].prim_index + j] = nodes.size();
							for(uint k = 0; k < leaf_ratio; ++k)
								nodes.emplace_back();
		
						}
					}
				}
				
			}

			for(auto& a : prim_assignments)
			{
				std::memcpy(nodes.data() + a.second, prims + a.first * prim_size, prim_size);
			}

			num_nodes = node_assignments.size();
			num_prims = prim_assignments.size();
		}
		else
		{
			nodes.clear();
			for(uint wnode_id = 0; wnode_id < wbvh.nodes.size(); ++wnode_id)
			{
				const WBVH::Node& wnode = wbvh.nodes[wnode_id];
				HECWBVH::Node cwnode(wnode, max_exp);
				nodes.push_back(cwnode);
			}

			num_nodes = nodes.size();
		}

		printf("HE%dCWBVH%d: Built\n", Node::NQ, WIDTH);
		printf("HE%dCWBVH%d: Size: %.1f MiB\n", Node::NQ, WIDTH, (float)sizeof(Node) * num_nodes / (1 << 20));
		printf("HE%dCWBVH%d: Dummy Blocks: %.1f KiB\n", Node::NQ, WIDTH, (float)sizeof(Node) * dummy_blocks / (1 << 10));
		printf("HE%dCWBVH%d: Exp: %.0f\n", Node::NQ, WIDTH, float32_bf(0, max_exp, 0).f32);
	}
#endif
};




inline WBVH::Node decompress(const HECWBVH::Node& cwnode)
{
	const rtm::vec3 p = cwnode.get_p();
	const rtm::vec3 e = cwnode.get_e();

	WBVH::Node wnode;
	for(int i = 0; i < WBVH::WIDTH; i++)
	{
		QAABB16 qaabb = cwnode.get_qaabb(i);
		wnode.aabb[i].min = vec3(qaabb.min[0], qaabb.min[1], qaabb.min[2]) * e + p;
		wnode.aabb[i].max = vec3(qaabb.max[0], qaabb.max[1], qaabb.max[2]) * e + p;
		wnode.data[i].is_int = cwnode.get_int(i);
		if(cwnode.get_int(i))
		{
			wnode.data[i].num_prims = 1;
			wnode.data[i].child_index = cwnode.base_index + cwnode.offset(i);
		}
		else
		{
			wnode.data[i].num_prims = cwnode.get_num_prims(i);
			wnode.data[i].prim_index = cwnode.base_index + cwnode.offset(i);
		}
	}

	return wnode;
}

}