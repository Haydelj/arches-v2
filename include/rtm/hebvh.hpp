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

	class alignas(32) Node
	{
		const static uint NQ = 7;
		const static uint E_BIAS = 127 - 24;
		const static uint BIT_PER_BOX = NQ * 6;
		uint64_t base_child_index : 27;
		uint64_t base_tri_index : 29;
		uint64_t max_exp : 8;
		uint16_t e0 : 5;
		uint16_t e1 : 5;
		uint16_t e2 : 5;
		uint16_t es : 1;
		uint16_t p0;
		uint16_t p1;
		uint16_t p2;
		BitArray<2 * WIDTH + BIT_PER_BOX * WIDTH> bit_array;

		void set_e(uvec3 e) { e0 = max(e[0], E_BIAS) - E_BIAS, e1 = max(e[1], E_BIAS) - E_BIAS, e2 = max(e[2], E_BIAS) - E_BIAS; }
		vec3 get_e() const { return vec3(float32_bf(0, e0 + E_BIAS, 0).f32, float32_bf(0, e1 + E_BIAS, 0).f32, float32_bf(0, e2 + E_BIAS, 0).f32); }
		void set_p(vec3 p)
		{
			p0 = u24_to_u16(f32_to_u24(p[0]));
			p1 = u24_to_u16(f32_to_u24(p[1]));
			p2 = u24_to_u16(f32_to_u24(p[2]));
		}
		vec3 get_p() const { return vec3(u24_to_f32(u16_to_u24(p0)), u24_to_f32(u16_to_u24(p1)), u24_to_f32(u16_to_u24(p2))); }
		void set_qaabb(uint i, QAABB16 qaabb)
		{
			for(uint j = 0; j < 3; ++j)
			{
				bit_array.write(2 * WIDTH + BIT_PER_BOX * i + NQ * (j + 0), NQ, qaabb.min[j]);
				bit_array.write(2 * WIDTH + BIT_PER_BOX * i + NQ * (j + 3), NQ, qaabb.max[j]);
			}
		}
		QAABB16 get_qaabb(uint i) const
		{
			QAABB16 qaabb;
			for(uint j = 0; j < 3; ++j)
			{
				qaabb.min[j] = bit_array.read(2 * WIDTH + BIT_PER_BOX * i + NQ * (j + 0), NQ);
				qaabb.max[j] = bit_array.read(2 * WIDTH + BIT_PER_BOX * i + NQ * (j + 3), NQ);
			}
			return qaabb;
		}
		void set_valid(uint i, uint v) { bit_array.write(i * 2 + 0, 1, v); }
		uint get_valid(uint i) const { return bit_array.read(i * 2 + 0, 1); }
		void set_int(uint i, uint v) { bit_array.write(i * 2 + 1, 1, v); }
		uint get_int(uint i) const { return bit_array.read(i * 2 + 1, 1); }
		uint offset(uint i) const
		{
			return i;
		}

		Node() = default;

	#ifndef __riscv
		Node(const WBVH::Node& wnode, uint max_exp)
		{
			sizeof(Node);
			constexpr float denom = 1.0f / ((1 << NQ) - 1);

			AABB aabb;
			uint base_child_index = ~0u, base_prim_index = ~0u;
			for(uint i = 0; i < WIDTH; ++i)
			{
				if(!wnode.is_valid(i)) continue;
				aabb.add(wnode.aabb[i]);
				if(wnode.data[i].is_int) base_child_index = min(base_child_index, wnode.data[i].child_index);
				else                     base_prim_index = min(base_prim_index, wnode.data[i].prim_index);
			}
			set_p(aabb.min);

			float32_bf e0((aabb.max.x - get_p().x) * denom);
			float32_bf e1((aabb.max.y - get_p().y) * denom);
			float32_bf e2((aabb.max.z - get_p().z) * denom);
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
					set_valid(i, 0);
					continue;
				}

				set_int(i, wnode.data[i].is_int);
				set_valid(i, 1);

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
	HECWBVH(const rtm::WBVH& wbvh, uint8_t max_exp = 127)
	{
		sizeof(Node);
		sizeof(QTriangleStrip);
		printf("Building HE%dCWBVH%d\n", Node::NQ, WIDTH);
		assert(wbvh.nodes.size() != 0);

		nodes.clear();
		for(uint wnode_id = 0; wnode_id < wbvh.nodes.size(); ++wnode_id)
		{
			const WBVH::Node& wnode = wbvh.nodes[wnode_id];
			HECWBVH::Node cwnode(wnode, max_exp);
			nodes.push_back(cwnode);
		}

		nodes.clear();
		//std::map<uint, uint> node_assignments;
		//std::map<uint, uint> prim_assignments;
		//node_assignments[0] = 0; nodes.emplace_back();
		//for(uint wnode_id = 0; wnode_id < wbvh.nodes.size(); ++wnode_id)
		//{
		//	const WBVH::Node& wnode = wbvh.nodes[wnode_id];
		//	Node cwnode(wnode, max_exp); cwnode.base_index = nodes.size();
		//	uint cwnode_id = node_assignments[wnode_id];
		//	for(uint i = 0; i < WIDTH; ++i)
		//	{
		//		if(wnode.is_valid(i))
		//		{
		//			if(wnode.data[i].is_int)
		//			{
		//				node_assignments[wnode.data[i].child_index] = nodes.size();
		//				nodes.emplace_back();
		//			}
		//			else
		//			{
		//				prim_assignments[wnode.data[i].prim_index] = nodes.size();
		//				for(uint j = 0; j < sizeof(QTriangleStrip) / sizeof(Node); ++j) nodes.emplace_back();
		//			}
		//		}
		//	}
		//	nodes[cwnode_id] = cwnode;
		//}

		//for(auto& a : prim_assignments)
		//{
		//	uint id = a.first;
		//	QTriangleStrip packet(strips[id]);
		//	QTriangleStrip& prim = *(QTriangleStrip*)(nodes.data() + a.second);
		//	prim = packet;
		//}

		printf("Built HE%dCWBVH%d\n", Node::NQ, WIDTH);
		printf("Nodes+Strips: %d\n", (uint)nodes.size());
		printf("Size: %.2f MiB\n", (float)sizeof(Node) * nodes.size() / (1 << 20));
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
			wnode.data[i].child_index = cwnode.base_child_index + cwnode.offset(i);
		}
		else
		{
			wnode.data[i].num_prims = cwnode.get_valid(i);
			wnode.data[i].prim_index = cwnode.base_tri_index + cwnode.offset(i);
		}
	}

	return wnode;
}

}