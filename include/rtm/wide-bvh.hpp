#pragma once

#include "int.hpp"
#include "aabb.hpp"
#include "bvh.hpp"
#include "triangle-blocks.hpp"

#ifndef __riscv
#include <vector>
#include <algorithm>
#include <cassert>
#endif

namespace rtm {

constexpr int INVALID_NODE = -1;

/// <summary>
/// class to build n-ary wide-bvh using bvh2, takes in branching factor and max prims per leaf as template params
/// Phase 1 : Cost computation in bottom up fashion
/// Phase 2 : Collapse BVH2 to BVH8
/// Phase 3 : Compress BVH8
/// </summary>

class WBVH
{
public:
	const static uint WIDTH = 12;
	const static uint MAX_PRIMS = 3;
	const static uint LEAF_RATIO = 1;

	const static uint MAX_FOREST_SIZE = WIDTH - 1;

	struct alignas(32) Node
	{
		union Data
		{
			struct
			{
				uint32_t is_int     : 1;
				uint32_t num_prims  : 2;
				uint32_t prim_index : 29;
			};
			struct
			{
				uint32_t             : 3;
				uint32_t child_index : 29;
			};
		};

		Data data[WIDTH];
		AABB aabb[WIDTH];
		bool is_valid(uint i) const { return data[i].is_int || data[i].num_prims > 0; }
		uint num_aabb() const 
		{ 
			uint count = 0; 
			for(uint i = 0; i < WIDTH; ++i)
				if(is_valid(i)) count++;
			return count;
		}
	};


#ifndef __riscv
	std::vector<Node> nodes;
	std::vector<rtm::BVH2::BuildObject> new_build_objects;
	uint num_leafs{0};

	const Mesh* mesh; //used for the strip encoder
	bool quantize;
	std::vector<rtm::QTB> qt_blocks;
	std::vector<rtm::FTB> ft_blocks;

public:
	WBVH(const rtm::BVH2& bvh2, std::vector<rtm::BVH2::BuildObject>& build_objects, const Mesh* mesh = nullptr, bool quantize = false) : mesh(mesh), quantize(quantize)
	{
		printf("WBVH%d: Building\n", WIDTH);

		std::vector<Decision> decisions; // array to store cost and meta data for the collapse algorithm
		decisions.resize(bvh2.nodes.size() * MAX_FOREST_SIZE);
		nodes.emplace_back();

		calculate_cost(bvh2, build_objects, 0, bvh2.nodes[0].aabb.surface_area(), decisions);
		collapse(bvh2, build_objects, 0, 0, decisions);
		build_objects = new_build_objects;

		uint num_prims = build_objects.size();
		printf("WBVH%d: Size: %.1f MiB\n", WIDTH, (float)sizeof(Node) * nodes.size() / (1 << 20));
		printf("WBVH%d: Prims/Leaf: %.2f\n", WIDTH, (float)num_prims / num_leafs);

		if(mesh)
		{
			if(quantize)
			{
				printf("QTB: Size: %.1f MiB\n", (float)sizeof(QTB) * qt_blocks.size() / (1 << 20));
				printf("QTB: Prims/Block: %.2f\n", (float)build_objects.size() / qt_blocks.size());
			}
			else
			{
				printf("FTB: Size: %.1f MiB\n", (float)sizeof(FTB) * ft_blocks.size() / (1 << 20));
				printf("FTB: Prims/Block: %.2f\n", (float)build_objects.size() / ft_blocks.size());
			}
		}
	}

	static rtm::WBVH::Node decompress(const rtm::WBVH::Node& node)
	{
		return node;
	}


	~WBVH() = default;

private:
	/// <summary>
	/// Struct defines what a decision nodes holds, we use these decisions to build the cost tree for wide bvh
	/// </summary>
	struct Decision
	{
		enum struct Type : char
		{
			LEAF,
			INTERNAL,
			DISTRIBUTE
		}
		type;

		char distribute_left;
		char distribute_right;
		float cost;
	};

	//Subroutines to build decision tree
	uint calculate_cost(const rtm::BVH2& bvh2, const std::vector<rtm::BVH2::BuildObject>& build_objects, int node_index, float root_surface_area, std::vector<Decision>& decisions)
	{
		///starts with root node for SBVH 
		const BVH2::Node& node = bvh2.nodes[node_index];
		const float leaf_node_multiplier = 1.0 * LEAF_RATIO;
		
		uint num_tris = 0u;
		if(node.data.is_leaf)
		{
			num_tris = 1;

			//SAH cost for leaf
			const BVH2::BuildObject& build_object = build_objects[node.data.prim_index];
			float cost_leaf = node.aabb.surface_area() * leaf_node_multiplier;
			//float cost_leaf = node.aabb.surface_area() * num_tris;

			//initialize the forest to default value if leaf node
			for(int i = 0; i < MAX_FOREST_SIZE; i++)
			{
				decisions[node_index * MAX_FOREST_SIZE + i].type = Decision::Type::LEAF;
				decisions[node_index * MAX_FOREST_SIZE + i].cost = cost_leaf;
			}
		}
		else
		{
			//post order recursive traverse to calculate total primitives in this subtree
			num_tris = calculate_cost(bvh2, build_objects, node.data.child_index + 0, node.aabb.surface_area(), decisions) +
			           calculate_cost(bvh2, build_objects, node.data.child_index + 1, node.aabb.surface_area(), decisions);

			//Case for choosing a single node (i == 1 from paper)
			//use min(Cprim, Cinternal)
			{
				float cost_leaf = INFINITY;

				if(mesh)
				{
					if(num_tris <= QTB::MAX_PRIMS)
					{
						uint prims[QTB::MAX_PRIMS];
						uint num_prims = collect_primitives(bvh2, node_index, prims);
						if(quantize)
						{
							rtm::QTB block;
							if(rtm::compress(prims, num_prims, 0, *mesh, block))
								cost_leaf = node.aabb.surface_area() * leaf_node_multiplier;
						}
						else
						{
							rtm::FTB block;
							if(rtm::compress(prims, num_prims, 0, *mesh, block))
								cost_leaf = node.aabb.surface_area() * leaf_node_multiplier;
								//cost_leaf = node.aabb.surface_area() * num_tris;
						}
					}
				}
				else
				{
					if(num_tris <= MAX_PRIMS)
						cost_leaf = node.aabb.surface_area() * num_tris * leaf_node_multiplier;
				}

				float cost_distribute = INFINITY;
				char  distribute_left = INVALID_NODE;
				char  distribute_right = INVALID_NODE;

				//Pick min from permutation of costs from left and right subtree
				for(int k = 0; k < MAX_FOREST_SIZE; k++)
				{
					float cost =
						decisions[(node.data.child_index) * MAX_FOREST_SIZE + k].cost +
						decisions[(node.data.child_index + 1) * MAX_FOREST_SIZE + (MAX_FOREST_SIZE - 1) - k].cost;

					if(cost < cost_distribute)
					{
						cost_distribute = cost;
						distribute_left = k;
						distribute_right = (MAX_FOREST_SIZE - 1) - k;
					}
				}

				float cost_internal = cost_distribute + node.aabb.surface_area();

				//Pick the min cost
				if(cost_leaf < cost_internal)
				{
					decisions[node_index * MAX_FOREST_SIZE].type = Decision::Type::LEAF;
					decisions[node_index * MAX_FOREST_SIZE].cost = cost_leaf;
				}
				else
				{
					decisions[node_index * MAX_FOREST_SIZE].type = Decision::Type::INTERNAL;
					decisions[node_index * MAX_FOREST_SIZE].cost = cost_internal;
				}

				decisions[node_index * MAX_FOREST_SIZE].distribute_left = distribute_left;
				decisions[node_index * MAX_FOREST_SIZE].distribute_right = distribute_right;

			}

			//Create wide bvh root node for a subtree
			//Case for 1 > i <= 7 ( from paper)
			{
				for(int i = 1; i < MAX_FOREST_SIZE; i++)
				{
					//propagate cheapest option
					float cost_distribute = decisions[node_index * MAX_FOREST_SIZE + (i - 1)].cost;
					char distribute_left = INVALID_NODE;
					char distribute_right = INVALID_NODE;

					for(int k = 0; k < i; k++)
					{
						float cost = decisions[(node.data.child_index) * MAX_FOREST_SIZE + k].cost +
							decisions[(node.data.child_index + 1) * MAX_FOREST_SIZE + i - k - 1].cost;

						if(cost < cost_distribute)
						{
							cost_distribute = cost;
							distribute_left = k;
							distribute_right = i - k - 1;
						}
					}

					decisions[node_index * MAX_FOREST_SIZE + i].cost = cost_distribute;

					if(distribute_left != INVALID_NODE)
					{
						decisions[node_index * MAX_FOREST_SIZE + i].type = Decision::Type::DISTRIBUTE;
						decisions[node_index * MAX_FOREST_SIZE + i].distribute_left = distribute_left;
						decisions[node_index * MAX_FOREST_SIZE + i].distribute_right = distribute_right;
					}
					else
					{
						decisions[node_index * MAX_FOREST_SIZE + i] = decisions[node_index * MAX_FOREST_SIZE + i - 1];
					}
				}
			}
		}
		
		return num_tris;
	}

	//Recursive count of triangles in a subtree
	uint collect_primitives(const rtm::BVH2& bvh2, uint root_index, uint* prims)
	{
		uint num_prims = 0;

		uint stack[32];
		uint stack_size = 0;
		stack[stack_size++] = root_index;
		while(stack_size > 0)
		{
			uint nid = stack[--stack_size];
			const BVH2::Node& n = bvh2.nodes[nid];
			if(n.data.is_leaf)
			{
				prims[num_prims++] = n.data.prim_index;
			}
			else
			{
				stack[stack_size++] = n.data.child_index + 0;
				stack[stack_size++] = n.data.child_index + 1;
			}
		}

		return num_prims;
	}

	//Get potential child nodes based on decision tree
	void get_children(const rtm::BVH2& bvh2, std::vector<rtm::BVH2::BuildObject>& build_objects, int node_index, int* children, int& child_count, int i, const std::vector<Decision>& decisions)
	{
		const BVH2::Node& node = bvh2.nodes[node_index];
		const BVH2::BuildObject& build_object = build_objects[node.data.prim_index];

		if(node.data.is_leaf)
		{
			children[child_count++] = node_index; //Return self index if leaf node
			return;
		}

		char distribute_left = decisions[node_index * MAX_FOREST_SIZE + i].distribute_left;
		char distribute_right = decisions[node_index * MAX_FOREST_SIZE + i].distribute_right;

		assert(distribute_left >= 0 && distribute_left < MAX_FOREST_SIZE);
		assert(distribute_right >= 0 && distribute_right < MAX_FOREST_SIZE);

		//Recurse on left child if it needs to distribute
		if(decisions[node.data.child_index * MAX_FOREST_SIZE + distribute_left].type == Decision::Type::DISTRIBUTE)
		{
			get_children(bvh2, build_objects, node.data.child_index, children, child_count, distribute_left, decisions);
		}
		else
		{
			children[child_count++] = node.data.child_index;
		}

		//Recurse on right child if it needs to distribute
		if(decisions[(node.data.child_index + 1) * MAX_FOREST_SIZE + distribute_right].type == Decision::Type::DISTRIBUTE)
		{
			get_children(bvh2, build_objects, node.data.child_index + 1, children, child_count, distribute_right, decisions);
		}
		else
		{
			children[child_count++] = node.data.child_index + 1;
		}
	}

	//generate a n-ary-sz branch factor wide bvh from a bvh2
	void collapse(const rtm::BVH2& bvh2, std::vector<rtm::BVH2::BuildObject>& build_objects, int wnode_index, int node_index, const std::vector<Decision>& decisions)
	{
		Node& wnode = nodes[wnode_index];

		int children[WIDTH];
		for(int i = 0; i < WIDTH; i++) { children[i] = INVALID_NODE; }

		//Get child nodes for this node based on the decision array costs
		int child_count = 0;
		get_children(bvh2, build_objects, node_index, children, child_count, 0, decisions);
		assert(child_count <= WIDTH);

		int index = 0;
		int num_internal = 0;
		for(int i = 0; i < WIDTH; i++)
		{
			int child_index = children[i];
			if(child_index == INVALID_NODE)
			{
				wnode.data[index].is_int = 0;
				wnode.data[index].num_prims = 0;
				continue;
			}

			switch(decisions[child_index * MAX_FOREST_SIZE].type)
			{
			case Decision::Type::LEAF:
			{
				num_leafs++;
				wnode.data[index].is_int = 0;

				uint prims[QTB::MAX_PRIMS];
				uint num_prims = collect_primitives(bvh2, child_index, prims);
				if(mesh)
				{
					wnode.data[index].num_prims = 1;
					if(quantize)
					{
						wnode.data[index].prim_index = qt_blocks.size();
						rtm::QTB block;
						rtm::compress(prims, num_prims, new_build_objects.size(), *mesh, block);
						qt_blocks.push_back(block);
					}
					else
					{
						wnode.data[index].prim_index = ft_blocks.size();
						rtm::FTB block;
						rtm::compress(prims, num_prims, new_build_objects.size(), *mesh, block);
						ft_blocks.push_back(block);
					}
				}
				else
				{
					wnode.data[index].num_prims = num_prims;
					wnode.data[index].prim_index = new_build_objects.size();
				}
				for(uint i = 0; i < num_prims; ++i)
					new_build_objects.push_back(build_objects[prims[i]]);
			}
			break;

			case Decision::Type::INTERNAL:
				wnode.data[index].is_int = 1;
				wnode.data[index].num_prims = 1;
				wnode.data[index].child_index = nodes.size() + num_internal++;
				break;

			default:
				assert(false);
				break;
			}
			wnode.aabb[index] = bvh2.nodes[child_index].aabb;
			index++;
		}

		for(uint i = 0; i < num_internal; ++i)
			nodes.emplace_back();

		index = 0;
		for(int i = 0; i < WIDTH; i++)
		{
			int child_index = children[i];
			if(child_index == INVALID_NODE) continue;

			if(nodes[wnode_index].data[index].is_int)
				collapse(bvh2, build_objects, nodes[wnode_index].data[index].child_index, child_index, decisions);

			index++;
		}
	}
#endif
};







inline WBVH::Node decompress(const WBVH::Node& wnode)
{
	return wnode;
}

struct RestartTrail
{
	const static uint N = WBVH::WIDTH;

	uint8_t _data[32];

	uint find_parent_level(uint level) const
	{
		for(uint i = level - 1; i < ~0u; --i)
			if(get(i) < N)
				return i;
		return ~0u;
	}

	uint get(uint level) const
	{
		return _data[level];
	}

	void set(uint level, uint value)
	{
		_data[level] = value;
		//_data &= ~(0xfull << shft(level));
		//_data |= (uint64_t)value << shft(level);
	}

	void clear(uint start_level)
	{
		for(uint i = start_level; i < 32; ++i)
			_data[i] = 0;
		//uint64_t mask = ((0x1ull << shft(start_level)) - 1);
		//_data &= mask;
	}

	bool is_done()
	{
		return _data[0] == 255;
	}

	void mark_done()
	{
		_data[0] = 255;
	}
};


}