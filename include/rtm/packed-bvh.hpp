#pragma once 

#include "bvh.hpp"

#ifndef __riscv
#include <vector>
#endif

namespace rtm
{

class alignas(64) PackedBVH2
{
public:
	struct Pack
	{
		BVH::Node nodes[2];
	};

#ifndef __riscv
	std::vector<Pack> packs;

	PackedBVH2(std::vector<BVH::BuildObject>& build_objects)
	{
		BVH bvh;
		bvh.build(build_objects, 1u);

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

		packs.clear();
		packs.emplace_back();

		while(!stack.empty())
		{
			NodeSet current_set = stack.back();
			stack.pop_back();

			for(uint i = 0; i < 2; ++i)
			{
				Pack& current_packed_node = packs[current_set.index];
			}

			for(uint i = 0; i <= current_set.data.lst_chld_ofst; ++i)
			{
				Pack& current_pack = packs[current_set.index];

				uint index = current_set.data.fst_chld_ind + i;
				BVH::Node node = bvh.nodes[index];

				current_pack.nodes[i] = node;

				if(!node.data.is_leaf)
				{
					stack.emplace_back(node.data, packs.size());
					current_pack.nodes[i].data.fst_chld_ind = packs.size();
					packs.emplace_back();
				}
			}
		}
	}
#endif
};

}