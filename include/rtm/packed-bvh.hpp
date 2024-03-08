#pragma once 

#include "bvh.hpp"

#ifndef __riscv
#include <vector>
#endif

namespace rtm {

class alignas(64) PackedBVH2
{
public:
	struct alignas(64) Node
	{
		union Data
		{
			struct
			{
				uint32_t is_leaf : 1;
				uint32_t num_tri : 3;
				uint32_t tri_index : 28;
			};
			struct
			{
				uint32_t             : 1;
				uint32_t child_index : 31;
			};
		};

		rtm::AABB aabb[2];
		Data      data[2];
	};

#ifndef __riscv
	std::vector<Node> nodes;

	PackedBVH2(const BVH& bvh)
	{
		size_t num_packed_nodes = 0;
		struct NodeSet
		{
			BVH::Node::Data data;
			uint32_t        index;

			NodeSet() = default;
			NodeSet(BVH::Node::Data data, uint32_t index) : data(data), index(index) {};
		};

		std::vector<NodeSet> stack; stack.reserve(96);
		stack.emplace_back(bvh.nodes[0].data, 0);

		nodes.clear();
		nodes.emplace_back();

		while(!stack.empty())
		{
			NodeSet current_set = stack.back();
			stack.pop_back();

			for(uint i = 0; i <= current_set.data.lst_chld_ofst; ++i)
			{
				Node& current_pack = nodes[current_set.index];

				uint index = current_set.data.fst_chld_ind + i;
				BVH::Node node = bvh.nodes[index];

				current_pack.aabb[i] = node.aabb;
				current_pack.data[i].is_leaf = node.data.is_leaf;

				if(node.data.is_leaf)
				{
					current_pack.data[i].tri_index = node.data.fst_chld_ind;
					current_pack.data[i].num_tri = node.data.lst_chld_ofst;
				}
				else
				{
					stack.emplace_back(node.data, nodes.size());
					current_pack.data[i].child_index = nodes.size();
					nodes.emplace_back();
				}
			}
		}
	}
#endif
};

}