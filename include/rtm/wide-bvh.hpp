#pragma once

#include "int.hpp"
#include "aabb.hpp"
#include "bvh.hpp"

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
	const static uint WIDTH = 2;
	const static uint MAX_FOREST_SIZE = WIDTH - 1;

	struct alignas(64) Node
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
	std::vector<uint> indices;
	std::vector<TriangleStrip> triangle_strips;
	std::vector<IndexStrip> index_strips;

public:
	WBVH(const rtm::BVH2& bvh2, const Mesh& mesh, std::vector<rtm::BVH2::BuildObject>& build_objects)
	{
		printf("Building Wide BVH\n");

		decisions.resize(bvh2.nodes.size() * MAX_FOREST_SIZE);
		nodes.emplace_back();
		std::vector<uint> prims;
		rtm::MeshGraph mesh_graph(mesh);
		calculate_cost(bvh2, mesh_graph, decisions, 0, bvh2.nodes[0].aabb.surface_area(), prims);
		collapse(bvh2, mesh, mesh_graph, 0, 0);

		std::vector<rtm::BVH2::BuildObject> temp_build_objects(build_objects);
		for(uint i = 0; i < build_objects.size(); ++i)
			build_objects[i] = temp_build_objects[indices[i]];

		printf("Built Wide BVH\n");
		printf("Strips: %d\n", triangle_strips.size());
		printf("Tris/Strip: %f\n", (float)build_objects.size() / triangle_strips.size());
		printf("Strips size: %d MB\n", triangle_strips.size() * sizeof(TriangleStrip) / (1 << 20));
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

	std::vector<Decision> decisions; // array to store cost and meta data for the collapse algorithm

	//Subroutines to build decision tree
	static void calculate_cost(const BVH2& bvh2, const MeshGraph& mesh_graph, std::vector<Decision>& decisions, int node_index, float root_surface_area, std::vector<uint>& prims)
	{
		///starts with root node for SBVH 
		const BVH2::Node& node = bvh2.nodes[node_index];
		//int num_primitives = 0;
		prims.clear();

		if(node.data.is_leaf)
		{
			//num_primitives = node.data.num_prims + 1;
			prims.push_back(node.data.prim_index);

			//SAH cost for leaf
			float cost_leaf = node.aabb.surface_area();//float(num_primitives);

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
			//num_primitives =
				//recurse left child
			calculate_cost(bvh2, mesh_graph, decisions, node.data.child_index, node.aabb.surface_area(), prims);
				//+
				//recurse right child
			std::vector<uint> right;
			calculate_cost(bvh2, mesh_graph, decisions, node.data.child_index + 1, node.aabb.surface_area(), right);
			prims.insert(prims.end(), right.begin(), right.end());

			//Case for choosing a single node (i == 1 from paper)
			//use min(Cprim, Cinternal)
			{
				float cost_leaf = INFINITY;

				//if(num_primitives <= MAX_PRIMS)
				if(prims.size() <= rtm::TriangleStrip::MAX_TRIS)
				{
					bool can_compress = mesh_graph.can_stripify(prims);
					if(can_compress)
						cost_leaf = node.aabb.surface_area() * 1.0;// float(num_primitives) * 1.0f;
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

				float cost_internal = cost_distribute + node.aabb.surface_area() * 1.0f;

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
		//return num_primitives;
	}

	//Recursive count of triangles in a subtree
	static void collect_primitives(const BVH2& bvh2, uint node_index, std::vector<uint>& prims)
	{
		const BVH2::Node bvh2node = bvh2.nodes[node_index];

		if(bvh2node.data.is_leaf)
		{
			int count = bvh2node.data.num_prims + 1;
			assert(count == 1);

			for(uint32_t i = 0; i < count; i++)
			{
				prims.push_back(bvh2node.data.prim_index + i);
			}
		}
		else
		{
			collect_primitives(bvh2, bvh2node.data.child_index, prims);
			collect_primitives(bvh2, bvh2node.data.child_index + 1, prims);
		}
	}

	//Get potential child nodes based on decision tree
	static void get_children(const BVH2& bvh2, std::vector<Decision>& decisions, int node_index, int* children, int& child_count, int i, const int MAX_FOREST_SIZE)
	{
		const BVH2::Node& bvh2node = bvh2.nodes[node_index];

		if(bvh2node.data.is_leaf)
		{
			children[child_count++] = node_index; //Return self index if leaf node
			return;
		}

		char distribute_left = decisions[node_index * MAX_FOREST_SIZE + i].distribute_left;
		char distribute_right = decisions[node_index * MAX_FOREST_SIZE + i].distribute_right;

		assert(distribute_left >= 0 && distribute_left < MAX_FOREST_SIZE);
		assert(distribute_right >= 0 && distribute_right < MAX_FOREST_SIZE);

		//Recurse on left child if it needs to distribute
		if(decisions[bvh2node.data.child_index * MAX_FOREST_SIZE + distribute_left].type == Decision::Type::DISTRIBUTE)
		{
			get_children(bvh2, decisions, bvh2node.data.child_index, children, child_count, distribute_left, MAX_FOREST_SIZE);
		}
		else
		{
			children[child_count++] = bvh2node.data.child_index;
		}

		//Recurse on right child if it needs to distribute
		if(decisions[(bvh2node.data.child_index + 1) * MAX_FOREST_SIZE + distribute_right].type == Decision::Type::DISTRIBUTE)
		{
			get_children(bvh2, decisions, bvh2node.data.child_index + 1, children, child_count, distribute_right, MAX_FOREST_SIZE);
		}
		else
		{
			children[child_count++] = bvh2node.data.child_index + 1;
		}
	}

	//generate a n-ary-sz branch factor wide bvh from a bvh2
	void collapse(const BVH2& bvh2, const Mesh& mesh, const MeshGraph& mesh_graph, int node_index_wbvh, int node_index_bvh2)
	{
		Node& wbvh_node = nodes[node_index_wbvh];
		const BVH2::Node& bvh2Node = bvh2.nodes[node_index_bvh2];

		int children[WIDTH];
		for(int i = 0; i < WIDTH; i++) { children[i] = INVALID_NODE; }

		//Get child nodes for this node based on the decision array costs
		int child_count = 0;
		get_children(bvh2, decisions, node_index_bvh2, children, child_count, 0, MAX_FOREST_SIZE);
		assert(child_count <= WIDTH);

		int index = 0;
		int num_internal = 0;
		for(int i = 0; i < WIDTH; i++)
		{
			int child_index = children[i];
			if(child_index == INVALID_NODE)
			{
				wbvh_node.data[index].is_int = 0;
				wbvh_node.data[index].num_prims = 0;
				continue;
			}

			switch(decisions[child_index * MAX_FOREST_SIZE].type)
			{
				//Caution: This wide bvh leaf node might have more than 1 leaf node
			case Decision::Type::LEAF:
			{
				wbvh_node.data[index].is_int = 0;
				wbvh_node.data[index].prim_index = triangle_strips.size();
				wbvh_node.data[index].num_prims = 1;

				std::vector<uint> prims;
				collect_primitives(bvh2, child_index, prims);
				assert(prims.size() <= TriangleStrip::MAX_TRIS);

				IndexStrip index_strip = mesh_graph.make_strip(mesh, prims, indices);
				TriangleStrip tri_strip = derefrence(index_strip, mesh.vertices.data());
				index_strips.push_back(index_strip);
				triangle_strips.push_back(tri_strip);
				break;
			}

			case Decision::Type::INTERNAL:
				wbvh_node.data[index].is_int = 1;
				wbvh_node.data[index].num_prims = 1;
				wbvh_node.data[index].child_index = nodes.size() + num_internal++;
				break;

			default:
				assert(false);
				break;
			}

			wbvh_node.aabb[index] = bvh2.nodes[child_index].aabb;
			index++;
		}

		for(uint i = 0; i < num_internal; ++i)
			nodes.emplace_back();

		index = 0;
		for(int i = 0; i < WIDTH; i++)
		{
			int child_index = children[i];
			if(child_index == INVALID_NODE) continue;

			if(nodes[node_index_wbvh].data[index].is_int)
				collapse(bvh2, mesh, mesh_graph, nodes[node_index_wbvh].data[index].child_index, child_index);

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