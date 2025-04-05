#pragma once

#include "wide-bvh.hpp"

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion,  collapsing can be further followed by compression 
//https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs

namespace rtm {

//Quantized aabb
struct QAABB8
{
	uint8_t min[3];
	uint8_t max[3];
};

//Quantized aabb
struct QAABB16
{
	uint16_t min[3];
	uint16_t max[3];
};

//NVIDIA Compressed Wide Bounding Volume Hierarchy
class NVCWBVH
{
public:
	const static uint WIDTH = WBVH::WIDTH;
	const static uint NODE_SIZE = 64;

	struct alignas(NODE_SIZE) Node
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
		printf("Building NVCWBVH%d\n", WIDTH);
		assert(wbvh.nodes.size() != 0);

		nodes.clear();
		for(uint wnode_id = 0; wnode_id < wbvh.nodes.size(); ++wnode_id)
		{
			const WBVH::Node& wnode = wbvh.nodes[wnode_id];
			NVCWBVH::Node cwnode(wnode);
			nodes.push_back(cwnode);
		}

		printf("Built NVCWBVH%d\n", WIDTH);
		printf("Bytes per node: %d\n", sizeof(Node));
		printf("Nodes: %d\n", nodes.size());
		printf("Size: %dMB\n", nodes.size() * sizeof(Node) / (1 << 20));
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




template<uint BITS>
struct BitArray
{
	uint8_t data[(BITS + 7) / 8];
	uint16_t read(uint bit_offset, uint bits) const
	{
		uint byte_index = bit_offset >> 3;
		uint byte_offset = bit_offset & 0x7;
		uint cpy_size = rtm::min(sizeof(data) - byte_index, 3);
		uint32_t bit_mask = (1 << bits) - 1;

		uint32_t read_data = 0;
		std::memcpy(&read_data, data + byte_index, cpy_size);
		return (read_data >> byte_offset) & bit_mask;
	}
	void write(uint bit_offset, uint bits, uint16_t value)
	{
		uint byte_index = bit_offset >> 3;
		uint byte_offset = bit_offset & 0x7;
		uint cpy_size = rtm::min(sizeof(data) - byte_index, 3);
		uint32_t bit_mask = (1 << bits) - 1;

		uint32_t background_data = 0;
		std::memcpy(&background_data, data + byte_index, cpy_size);

		background_data &= ~(bit_mask << byte_offset);
		background_data |= value << byte_offset;
		std::memcpy(data + byte_index, &background_data, cpy_size);
	}
};

//High Efficency Cowmpressed Wide Bounding Volume Hierarchy
class HECWBVH
{
public:
	const static uint WIDTH = WBVH::WIDTH;
	const static uint NODE_SIZE = 64;

	struct alignas(NODE_SIZE) Node
	{
		const static uint NQ = 7;
		const static uint E_BIAS = 127 - 24;
		const static uint BIT_PER_BOX = NQ * 6;
		uint32_t base_index : 29;
		uint16_t e0 : 5;
		uint16_t e1 : 5;
		uint16_t e2 : 5;
		uint16_t es : 1;
		uint16_t p0;
		uint16_t p1;
		uint16_t p2;
		uint16_t imask : WIDTH;
		uint16_t count : 4;
		BitArray<BIT_PER_BOX * WIDTH> bit_array;

		void set_e(uvec3 e) { e0 = max(e[0], E_BIAS) - E_BIAS, e1 = max(e[1], E_BIAS) - E_BIAS, e2 = max(e[2], E_BIAS) - E_BIAS; }
		vec3 get_e() const { return vec3(float32_bf(0, e0 + E_BIAS, 0).f32, float32_bf(0, e1 + E_BIAS, 0).f32, float32_bf(0, e2 + E_BIAS, 0).f32); }
		void set_p(vec3 p) { p0 = u24_to_u16(f32_to_u24(p[0])), p1 = u24_to_u16(f32_to_u24(p[1])), p2 = u24_to_u16(f32_to_u24(p[2])); }
		vec3 get_p() const { return vec3(u24_to_f32(u16_to_u24(p0)), u24_to_f32(u16_to_u24(p1)), u24_to_f32(u16_to_u24(p2))); }
		void set_qaabb(uint i, QAABB16 qaabb)
		{
			for(uint j = 0; j < 3; ++j)
			{
				bit_array.write(BIT_PER_BOX * i + NQ * (j + 0), NQ, qaabb.min[j]);
				bit_array.write(BIT_PER_BOX * i + NQ * (j + 3), NQ, qaabb.max[j]);
			}
		}
		QAABB16 get_qaabb(uint i) const
		{
			QAABB16 qaabb;
			for(uint j = 0; j < 3; ++j)
			{
				qaabb.min[j] = bit_array.read(BIT_PER_BOX * i + NQ * (j + 0), NQ);
				qaabb.max[j] = bit_array.read(BIT_PER_BOX * i + NQ * (j + 3), NQ);
			}
			return qaabb;
		}
		uint is_int(uint i) const { return (imask >> i) & 0x1; }
		uint is_valid(uint i) const { return i < count; }
		uint offset(uint i) const
		{
			uint o = 0;
			for(uint j = 0; j < i; ++j)
				if((imask >> j) & 0x1) o++;
				else o += sizeof(Strip) / sizeof(Node);
			return o;
		}

		Node() = default;

	#ifndef __riscv
		Node(const WBVH::Node& wnode)
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

			count = 0;
			imask = 0;
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
				if(!wnode.is_valid(i)) continue;

				imask |= (wnode.data[i].is_int) << i;
				count++;

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

	struct alignas(NODE_SIZE) Strip
	{
		const static uint MAX_TRIS = TriangleStrip::MAX_TRIS;
		uint32_t id : 29;
		uint8_t num_tris : 3;
		uint8_t edge_mask : 5;
		uint8_t data[9 * (2 + MAX_TRIS)];
		Strip(const rtm::TriangleStrip& other) : id(other.id), num_tris(other.num_tris), edge_mask(other.edge_mask)
		{
			sizeof(Strip);
			for(uint i = 0; i < 3 * num_tris + 6; ++i)
			{
				uint32_t u24 = f32_to_u24(((float*)other.vrts)[i]);
				std::memcpy(data + i * 3, &u24, 3);
			}
		}
	};

#ifndef __riscv
	std::vector<Node> nodes;
	HECWBVH(const rtm::WBVH& wbvh, const std::vector<TriangleStrip>& strips)
	{
		//static_assert(sizeof(Node) == NODE_SIZE);
		//static_assert(sizeof(Strip) == NODE_SIZE);
		printf("Building HE%dCWBVH%d\n", Node::NQ, WIDTH);
		assert(wbvh.nodes.size() != 0);
		nodes.clear();

		std::map<uint, uint> node_assignments;
		std::map<uint, uint> prim_assignments;
		node_assignments[0] = 0; nodes.emplace_back();
		for(uint wnode_id = 0; wnode_id < wbvh.nodes.size(); ++wnode_id)
		{
			const WBVH::Node& wnode = wbvh.nodes[wnode_id];
			Node cwnode(wnode); cwnode.base_index = nodes.size();
			uint cwnode_id = node_assignments[wnode_id];
			for(uint i = 0; i < WIDTH; ++i)
			{
				if(wnode.is_valid(i))
				{
					if(wnode.data[i].is_int)
					{
						node_assignments[wnode.data[i].child_index] = nodes.size();
						nodes.emplace_back();
					}
					else
					{
						prim_assignments[wnode.data[i].prim_index] = nodes.size();
						for(uint j = 0; j < sizeof(Strip) / sizeof(Node); ++j) nodes.emplace_back();
					}
				}
			}
			nodes[cwnode_id] = cwnode;
		}

		for(auto& a : prim_assignments)
		{
			uint id = a.first;
			Strip packet(strips[id]);
			Strip& prim = *(Strip*)(nodes.data() + a.second);
			prim = packet;
		}

		printf("Built HE%dCWBVH%d\n", Node::NQ, WIDTH);
		printf("Bytes per Node/Strip: %d\n", sizeof(Node));
		printf("Nodes+Strips: %d\n", nodes.size());
		printf("Size: %dMB\n", nodes.size() * sizeof(Node) / (1 << 20));
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
		wnode.data[i].is_int = cwnode.is_int(i);
		if(cwnode.is_int(i))
		{
			wnode.data[i].child_index = cwnode.base_index + cwnode.offset(i);
		}
		else
		{
			wnode.data[i].num_prims = cwnode.is_valid(i);
			wnode.data[i].prim_index = cwnode.base_index + cwnode.offset(i);
		}
	}

	return wnode;
}

inline uint decompress(const HECWBVH::Strip& strip, uint strip_id, rtm::IntersectionTriangle* tris)
{
	rtm::TriangleStrip temp_strip;
	temp_strip.id = strip.id;
	temp_strip.num_tris = strip.num_tris;
	temp_strip.edge_mask = strip.edge_mask;
	for(uint i = 0; i < 3 * (strip.num_tris + 2); ++i)
	{
		uint32_t u24;
		std::memcpy(&u24, strip.data + i * 3, 3);
		((float*)temp_strip.vrts)[i] = u24_to_f32(u24);
	}
	return decompress(temp_strip, strip_id, tris);
}

}