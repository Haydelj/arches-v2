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
		rtm::AABB             aabb[2];
		rtm::BVH2::Node::Data data[2];
	};

#ifndef __riscv
	std::vector<Node> nodes;

	PackedBVH2() {};

	PackedBVH2(const BVH2& bvh, std::vector<rtm::BVH2::BuildObject> build_objects)
	{
		printf("Building Packed BVH2\n");

		size_t num_packed_nodes = 0;
		struct NodeSet
		{
			BVH2::Node::Data data;
			uint32_t         index;

			NodeSet() = default;
			NodeSet(BVH2::Node::Data data, uint32_t index) : data(data), index(index) {};
		};

		std::vector<NodeSet> stack; stack.reserve(32);
		stack.emplace_back(bvh.nodes[0].data, 0);

		nodes.resize(bvh.nodes.size() / 2);
		uint32_t nodes_allocated = 1;
		while(!stack.empty())
		{
			NodeSet current_set = stack.back();
			stack.pop_back();

			for(uint i = 0; i < 2; ++i)
			{
				const BVH2::Node& node = bvh.nodes[current_set.data.child_index + i];

				Node& current_node = nodes[current_set.index];
				current_node.aabb[i] = node.aabb;
				current_node.data[i].is_leaf = node.data.is_leaf;
				if(node.data.is_leaf)
				{
					current_node.data[i].prim_index = node.data.prim_index;
					current_node.data[i].num_prims = node.data.num_prims;
				}
				else
				{
					stack.emplace_back(node.data, nodes_allocated);
					current_node.data[i].child_index = nodes_allocated;
					nodes_allocated++;
				}
			}
		}

		printf("Built Packed BVH2\n");
	}
#endif
};

}