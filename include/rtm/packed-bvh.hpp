#pragma once 

#include "bvh.hpp"

#ifndef __riscv
#include <vector>
#endif

namespace rtm {

class alignas(64) PackedBVH2
{
public:
	struct Node
	{
		rtm::AABB            aabb[2];
		rtm::BVH::Node::Data data[2];
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

			for(uint i = 0; i < 2; ++i)
			{
				Node& current_packed_node = nodes[current_set.index];
			}

			for(uint i = 0; i <= current_set.data.lst_chld_ofst; ++i)
			{
				Node& current_pack = nodes[current_set.index];

				uint index = current_set.data.fst_chld_ind + i;
				BVH::Node node = bvh.nodes[index];

				current_pack.aabb[i] = node.aabb;
				current_pack.data[i] = node.data;

				if(!node.data.is_leaf)
				{
					stack.emplace_back(node.data, nodes.size());
					current_pack.data[i].fst_chld_ind = nodes.size();
					nodes.emplace_back();
				}
			}
		}
	}
#endif
};

}