#pragma once

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion,  collapsing can be further followed by compression 
//https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs

#include "int.hpp"
#include "rtm.hpp"
#include "aabb.hpp"
#include "bvh.hpp"

#ifndef __riscv
#include <vector>
#include <algorithm>
#include <cassert>
#endif


namespace rtm {

constexpr int INVALID_NODE = -1;
constexpr int N_ARY_SZ = 6;
constexpr int MAX_FOREST_SIZE = N_ARY_SZ - 1;
constexpr int LEAF_PRIM_COUNT = 3;




/// <summary>
/// class to build n-ary wide-bvh using bvh2, takes in branching factor and max prims per leaf as template params
/// Phase 1 : Cost computation in bottom up fashion
/// Phase 2 : Collapse BVH2 to BVH8
/// Phase 3 : Compress BVH8
/// </summary>
class WideBVH
{
public:
	WideBVH(rtm::BVH2 _bvh2) : bvh2(_bvh2)
	{

		decisions.resize(bvh2.nodes.size() * MAX_FOREST_SIZE);
		nodes.emplace_back();
		calculate_cost(bvh2, decisions, 0, bvh2.nodes[0].aabb.surface_area(), MAX_FOREST_SIZE, LEAF_PRIM_COUNT);
		collapse(bvh2, 0, 0);
	}

	~WideBVH() = default;

	/// <summary>
	/// Wide bvh node layout, stores n_ary_sz child node array and information to indes nodes or triangle 
	/// </summary>
	struct Node
	{
		BVH2::Node nodeArray[N_ARY_SZ];
		uint32_t base_index_child;
		uint32_t base_tri_index;
		uint32_t childCount;
		AABB aabb;
	};

#ifndef __riscv

	std::vector<uint> indices; // index buffer to triangle primitives
	std::vector<Node> nodes; // Linearized nodes buffer for wide bvh

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


	const rtm::BVH2& bvh2;
	std::vector<Decision> decisions; // array to store cost and meta data for the collapse algorithm

	//Subroutines to build decision tree
	static int calculate_cost(const BVH2& bvh2, std::vector<Decision>& decisions, int node_index, float root_surface_area, const int MAX_FOREST_SIZE, const int LEAF_PRIM_COUNT)
	{
		///starts with root node for SBVH 
		const BVH2::Node& node = bvh2.nodes[node_index];
		int num_primitives = 0;

		if(node.data.is_leaf)
		{
			num_primitives = node.data.num_prims + 1;
			assert(num_primitives == 1); //for wide bvh collapse the bvh2 should be constrained to 1 primitive per leaf node

			//SAH cost for leaf
			float cost_leaf = node.aabb.surface_area() * float(num_primitives);

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
			num_primitives =
				//recurse left child
				calculate_cost(bvh2, decisions, node.data.child_index, node.aabb.surface_area(), MAX_FOREST_SIZE, LEAF_PRIM_COUNT)
				+
				//recurse right child
				calculate_cost(bvh2, decisions, node.data.child_index + 1, node.aabb.surface_area(), MAX_FOREST_SIZE, LEAF_PRIM_COUNT);

			//Case for choosing a single node (i == 1 from paper)
			//use min(Cprim, Cinternal)
			{
				float cost_leaf = INFINITY;

				if(num_primitives <= LEAF_PRIM_COUNT)
				{
					cost_leaf = node.aabb.surface_area() * float(num_primitives);
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
		return num_primitives;
	}

	//Recursive count of triangles in a subtree
	static uint count_primitives(const BVH2& bvh2, std::vector<uint>& indices, uint node_index)
	{
		const BVH2::Node bvh2node = bvh2.nodes[node_index];

		if(bvh2node.data.is_leaf)
		{
			int count = bvh2node.data.num_prims + 1;
			assert(count == 1);

			for(uint32_t i = 0; i < count; i++)
			{
				indices.push_back(bvh2node.data.prim_index + i);
			}

			return count;
		}

		return
			count_primitives(bvh2, indices, bvh2node.data.child_index) +
			count_primitives(bvh2, indices, bvh2node.data.child_index + 1);
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
	void collapse(const BVH2& bvh2, int node_index_wbvh, int node_index_bvh2)
	{
		Node& bvh8Node = nodes[node_index_wbvh];
		const BVH2::Node& bvh2Node = bvh2.nodes[node_index_bvh2];

		bvh8Node.aabb = bvh2Node.aabb;

		bvh8Node.base_index_child = nodes.size();
		bvh8Node.base_tri_index = indices.size();

		int child_count = 0;
		int children[N_ARY_SZ];
		for(int i = 0; i < N_ARY_SZ; i++) { children[i] = INVALID_NODE; }

		//Get child nodes for this node based on the decision array costs
		get_children(bvh2, decisions, node_index_bvh2, children, child_count, 0, MAX_FOREST_SIZE);
		assert(child_count <= N_ARY_SZ);

		bvh8Node.childCount = child_count;

		uint32_t num_internal_nodes = 0;
		int index = 0;

		for(int i = 0; i < N_ARY_SZ; i++)
		{
			int num_triangles = 0;
			int first_child_index = 0;
			int child_index = children[i];
			if(child_index == INVALID_NODE) continue;
			switch(decisions[child_index * MAX_FOREST_SIZE].type)
			{
				//Caution: This wide bvh leaf node might have more than 1 leaf node
			case Decision::Type::LEAF:
				first_child_index = indices.size();
				num_triangles = count_primitives(bvh2, indices, child_index);
				bvh8Node.nodeArray[index].data.is_leaf = 1;							//make it leaf node
				bvh8Node.nodeArray[index].data.prim_index = first_child_index;
				bvh8Node.nodeArray[index].data.num_prims = num_triangles - 1;
				bvh8Node.nodeArray[index].aabb = bvh2.nodes[child_index].aabb;
				index++;
				break;
			case Decision::Type::INTERNAL:
				bvh8Node.nodeArray[index].aabb = bvh2.nodes[child_index].aabb;
				bvh8Node.nodeArray[index].data.is_leaf = 0;
				bvh8Node.nodeArray[index].data.child_index = num_internal_nodes; //save node entry index in umcompressedNode array
				num_internal_nodes++;
				index++;
				break;
			default:
				assert(false);
				break;
			}
		}


		for(int i = 0; i < num_internal_nodes; i++)
		{
			nodes.emplace_back();
		}

		assert((nodes[node_index_wbvh].base_index_child + num_internal_nodes) == nodes.size());

		int offset = 0;
		index = 0;
		for(int i = 0; i < N_ARY_SZ; i++)
		{
			int child_index = children[i];
			if(child_index == INVALID_NODE) continue;
			if(nodes[node_index_wbvh].nodeArray[index++].data.is_leaf == 0)
			{
				collapse(bvh2, nodes[node_index_wbvh].base_index_child + offset++, child_index);
			}
		}
	}

#endif
};




/// <summary>
/// 
/// </summary>
/// <typeparam name="N_ARY_SZ"></typeparam>
/// <typeparam name="LEAF_PRIM_COUNT"></typeparam>
class CompressedWideBVH
{
public:

	struct alignas(64) Node
	{
		/*
		vec3 p;								//anchor point 
		uint32_t base_index_child;			//base offset in streamlined child node array
		uint32_t base_index_triangle;		//base offset in streamlined primitive array
		uint8_t  meta[N_ARY_SZ];			//count and indexing offset
		uint8_t  q_min_x[N_ARY_SZ];			//quantized bounds 
		uint8_t  q_min_y[N_ARY_SZ];
		uint8_t  q_min_z[N_ARY_SZ];
		uint8_t  q_max_x[N_ARY_SZ];
		uint8_t  q_max_y[N_ARY_SZ];
		uint8_t  q_max_z[N_ARY_SZ];
		uint8_t  e[3];						//exponent power of 2 for local grid scale
		uint8_t  imask;						//internal node flag
		*/


		uint64_t base_index_child : 28;	//base offset in streamlined child node array
		uint64_t base_index_triangle : 28;	//base offset in streamlined primitive array
		uint64_t e0 : 8;
		rtm::vec3 p;
		uint8_t e1;
		uint8_t e2;
		uint8_t  meta[N_ARY_SZ];    //MSB bit stores imask now, next two MSB bits store binary encoded primitive count, rest layout remains the same 
		uint8_t  q_min_x[N_ARY_SZ]; //quantized bounds
		uint8_t  q_min_y[N_ARY_SZ];
		uint8_t  q_min_z[N_ARY_SZ];
		uint8_t  q_max_x[N_ARY_SZ];
		uint8_t  q_max_y[N_ARY_SZ];
		uint8_t  q_max_z[N_ARY_SZ];


		Node() : p(0.0f, 0.0f, 0.0f), e0 {0}, e1{ 0 }, e2{ 0 }, base_index_child(0), base_index_triangle(0)
		{
			for(int i = 0; i < N_ARY_SZ; i++)
			{
				meta[i] = 0u;
				q_min_x[i] = 0u;
				q_min_y[i] = 0u;
				q_min_z[i] = 0u;
				q_max_x[i] = 0u;
				q_max_y[i] = 0u;
				q_max_z[i] = 0u;
			}
		};
		~Node() = default;
		/// <summary>
		/// Decompresses the node and extracts max 8 child nodes 
		/// </summary>
		/// <param name="dnodes"></param>
		/// <param name="childCount"></param>
		void decompress(BVH2::Node* dnodes, int& childCount) const
		{
			childCount = 0;
			int num_internal_nodes = 0;
			int index = 0;

			uint32_t e00, e11, e22;
			float e0_f, e1_f, e2_f;

			e00 = uint32_t(e0) << 23;
			e11 = uint32_t(e1) << 23;
			e22 = uint32_t(e2) << 23;

			e0_f = *reinterpret_cast<float*>(&e00);
			e1_f = *reinterpret_cast<float*>(&e11);
			e2_f = *reinterpret_cast<float*>(&e22);

			for(int i = 0; i < N_ARY_SZ; i++)
			{
				//check for non-empty child slot
				if(meta[i])
				{
					dnodes[index].aabb.min.x = p.x + e0_f * float(q_min_x[i]);
					dnodes[index].aabb.min.y = p.y + e1_f * float(q_min_y[i]);
					dnodes[index].aabb.min.z = p.z + e2_f * float(q_min_z[i]);
					dnodes[index].aabb.max.x = p.x + e0_f * float(q_max_x[i]);
					dnodes[index].aabb.max.y = p.y + e1_f * float(q_max_y[i]);
					dnodes[index].aabb.max.z = p.z + e2_f * float(q_max_z[i]);

					if(meta[i] & (1 << 7)) //if internal node
					{
						dnodes[index].data.is_leaf = false;
						dnodes[index].data.child_index = uint32_t(base_index_child) + num_internal_nodes++;
					}
					else //is leaf
					{
						dnodes[index].data.is_leaf = true;
						dnodes[index].data.prim_index = uint32_t(base_index_triangle) + (meta[i] & 0b00011111); // & 0b00011111

						//check the second last and third last MSB bits for binary encoded triangle count
						uint num_primitives = uint((meta[i] & 0b01100000) >> 5);

						/*
						uint32_t num_set_bits = 0;
						for(int j = 0; j < LEAF_PRIM_COUNT; j++)
						{

							if(meta[i] & (1u << (j + 5))) //if triangle present
							{
								num_set_bits++;
							}
						}*/

						dnodes[index].data.num_prims = num_primitives - 1;
					}
					index++;
				}
			}
			childCount = index; //store the activated index positions which represent the count of children
		}
	};

#ifndef __riscv
	std::vector<Node> nodes; //node array
	std::vector<uint> indices; //flat triangle array indices

public:
	/// <summary>
	///  Creates a compressed wide bvh from bvh2 with branching factor equals N_ARY_SZ
	///  Note: bvh2 should have max 1 prim per leaf node in order for this to work
	/// </summary>
	/// <param name="_bvh2"></param>
	CompressedWideBVH(const rtm::BVH2& bvh2) : CompressedWideBVH(WideBVH(bvh2)) {}

	/// <summary>
	/// Creates compressed wide bvh with branching factor equal to N_ARY_SZ from a wide bvh
	///  Note: bvh2 should have max 1 prim per leaf node in order for this to work
	/// </summary>
	/// <param name="_wbvh"></param>
	CompressedWideBVH(const rtm::WideBVH& wbvh)
	{
		assert(wbvh.nodes.size() != 0);
		indices.resize(wbvh.indices.size());
		std::copy(wbvh.indices.begin(), wbvh.indices.end(), indices.begin());

		nodes.clear();
		for(const WideBVH::Node& wnode : wbvh.nodes)
		{
			nodes.emplace_back();
			CompressedWideBVH::Node& cwnode = nodes.back();

			cwnode.p = wnode.aabb.min;
			constexpr int Nq = 8;
			constexpr float denom = 1.0f / float((1 << Nq) - 1);

			const AABB& aabb = wnode.aabb;
			vec3 e(
				exp2f(ceilf(log2f((aabb.max.x - aabb.min.x) * denom))),
				exp2f(ceilf(log2f((aabb.max.y - aabb.min.y) * denom))),
				exp2f(ceilf(log2f((aabb.max.z - aabb.min.z) * denom)))
			);

			vec3 one_over_e = vec3(1.0f / e.x, 1.0f / e.y, 1.0f / e.z);

			uint32_t u_ex = {};
			uint32_t u_ey = {};
			uint32_t u_ez = {};

			memcpy(&u_ex, &e.x, sizeof(float));
			memcpy(&u_ey, &e.y, sizeof(float));
			memcpy(&u_ez, &e.z, sizeof(float));

			assert((u_ex & 0b10000000011111111111111111111111) == 0);
			assert((u_ey & 0b10000000011111111111111111111111) == 0);
			assert((u_ez & 0b10000000011111111111111111111111) == 0);

			//Store 8 bit exponent
			cwnode.e0 = u_ex >> 23;
			cwnode.e1 = u_ey >> 23;
			cwnode.e2 = u_ez >> 23;

			cwnode.base_index_child = wnode.base_index_child;
			cwnode.base_index_triangle = wnode.base_tri_index;

			uint32_t num_triangles = 0;
			//Loop over all the uncompressed child nodes
			for(int i = 0; i < wnode.childCount; i++)
			{
				const BVH2::Node& child_node = wnode.nodeArray[i];

				cwnode.q_min_x[i] = uint8_t(floorf((child_node.aabb.min.x - cwnode.p.x) * one_over_e.x));
				cwnode.q_min_y[i] = uint8_t(floorf((child_node.aabb.min.y - cwnode.p.y) * one_over_e.y));
				cwnode.q_min_z[i] = uint8_t(floorf((child_node.aabb.min.z - cwnode.p.z) * one_over_e.z));

				cwnode.q_max_x[i] = uint8_t(ceilf((child_node.aabb.max.x - cwnode.p.x) * one_over_e.x));
				cwnode.q_max_y[i] = uint8_t(ceilf((child_node.aabb.max.y - cwnode.p.y) * one_over_e.y));
				cwnode.q_max_z[i] = uint8_t(ceilf((child_node.aabb.max.z - cwnode.p.z) * one_over_e.z));

				if(child_node.data.is_leaf)
				{
					uint32_t triangle_count = child_node.data.num_prims + 1;

					uint8_t tcount = static_cast<uint8_t>(triangle_count);

					cwnode.meta[i] |= (tcount << 5);

					//for(uint32_t j = 0; j < triangle_count; j++)
						//cwnode.meta[i] |= (1 << (j + 5));

					cwnode.meta[i] |= num_triangles; //base index relative to triangle
					num_triangles += triangle_count;
				}
				else
				{
					cwnode.meta[i] = (i + 24) | 0b00100000; // 32
					cwnode.meta[i] |= (1 << 7);
				}
			}
		}
	}

	~CompressedWideBVH() = default;
#endif
};

}