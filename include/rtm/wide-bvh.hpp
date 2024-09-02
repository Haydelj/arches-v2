#pragma once

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion.
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


namespace rtm
{

#define n_ary_sz 8 // max tree width
#define max_forst_sz (n_ary_sz-1) //max forest size
#define p_max 3 // max allowed leaf node size for wide BVH
#define INVALID _FINITE

	/// <summary>
	/// class to build n-ary wide-bvh using bvh2
	/// Phase 1 : Cost computation in bottom up fashion
	/// Phase 2 : Collapse BVH2 to BVH8
	/// </summary>
	class WideBVH
	{
	public:
		WideBVH::WideBVH() = default;
		WideBVH::~WideBVH() = default;

	private:

		struct Decision
		{
			enum struct Type : char {
				LEAF,
				INTERNAL,
				DISTRIBUTE
			} type;

			char distribute_left;
			char distribute_right;
			float cost;
		};


	public:

		struct DecompressedNodeData
		{
			bool is_leaf;
			uint32_t nodeIndex;
			rtm::AABB aabb;
			int triIndices[p_max];

			DecompressedNodeData() : is_leaf(false), nodeIndex(0), aabb(), triIndices{ INVALID,INVALID,INVALID } {}
			~DecompressedNodeData() {}

			DecompressedNodeData& operator=(const DecompressedNodeData& rhs) {
				is_leaf = rhs.is_leaf;
				nodeIndex = rhs.nodeIndex;
				aabb = rhs.aabb;
				for (int i = 0; i < p_max; i++)
				{
					triIndices[i] = rhs.triIndices[i];
				}
				return *this;
			}
		};

		struct WideBVHNode
		{
			rtm::vec3 p;						//anchor point 
			uint8_t  e[3];						//exponent power of 2 for local grid scale
			uint8_t  imask;						//internal node flag
			uint32_t base_index_child;			//base offset in streamlined child node array
			uint32_t base_index_triangle;		//base offset in streamlined primitive array
			uint8_t  meta[n_ary_sz];			//count and indexing offset
			uint8_t  q_min_x[n_ary_sz];			//quantized bb 
			uint8_t  q_min_y[n_ary_sz];
			uint8_t  q_min_z[n_ary_sz];
			uint8_t  q_max_x[n_ary_sz];
			uint8_t  q_max_y[n_ary_sz];
			uint8_t  q_max_z[n_ary_sz];

			WideBVHNode() : p(0.0f, 0.0f, 0.0f), e{ 0,0,0 }, imask(0), base_index_child(0), base_index_triangle(0)
			{
				for (int i = 0; i < n_ary_sz; i++)
				{
					meta[i] = 0;
					q_min_x[i] = 0;
					q_min_y[i] = 0;
					q_min_z[i] = 0;
					q_max_x[i] = 0;
					q_max_y[i] = 0;
					q_max_z[i] = 0;
				}
			};

			~WideBVHNode() = default;

			void decompress(rtm::BVH::Node* dnodes, int& childCount) const
			{
				childCount = 0;
				
				for (int i = 0; i < n_ary_sz; i++)
				{
					//check for non-empty child slot
					if (meta[i])
					{

						childCount++; //found child

						if (imask & (1 << i)) //if internal node
						{
							
							dnodes[i].data.is_leaf = false;
							dnodes[i].data.fst_chld_ind = base_index_child + i;

							uint32_t e0, e1, e2;
							float e0_f, e1_f, e2_f;

							e0 = uint32_t(e[0]) << 23;
							e1 = uint32_t(e[1]) << 23;
							e2 = uint32_t(e[2]) << 23;
							
							memcpy(&e0_f, &e0, sizeof(uint32_t));
							memcpy(&e1_f, &e1, sizeof(uint32_t));
							memcpy(&e2_f, &e2, sizeof(uint32_t));

							dnodes[i].aabb.min.x = p.x + e0_f * float(q_min_x[i]);
							dnodes[i].aabb.min.y = p.y + e1_f * float(q_min_y[i]);
							dnodes[i].aabb.min.z = p.z + e2_f * float(q_min_z[i]);
							dnodes[i].aabb.max.x = p.x + e0_f * float(q_max_x[i]);
							dnodes[i].aabb.max.y = p.y + e1_f * float(q_max_y[i]);
							dnodes[i].aabb.max.z = p.z + e2_f * float(q_max_z[i]);
						}
						else //is leaf
						{

							dnodes[i].data.is_leaf = true;
							dnodes[i].data.fst_chld_ind = base_index_triangle + (meta[i] & 0b00011111);

							int num_set_bits = 0;
							for (int j = 0; j < p_max; j++)
							{
								if (meta[i] & (1 << (j + 5))) //if triangle present
								{
									num_set_bits++;
								}
							}
							dnodes[i].data.lst_chld_ofst = num_set_bits - 1;
						}
					}
				}
			}
		};

		struct WideBVHNodeUncompressed
		{
			rtm::BVH::Node nodeArray[n_ary_sz];
			uint32_t base_index_child;
			uint32_t childCount;
		};

		static void swap_dnodes(DecompressedNodeData& d1, DecompressedNodeData& d2)
		{
			DecompressedNodeData temp;

			temp = d2;
			d2 = d1;
			d1 = temp;
		}

		void build(const rtm::BVH& bvh2)
		{
			std::cout << "WideBVH building ... " << std::endl;

			//each node may have max forest size number of cost permutations
			decisions.resize(bvh2.nodes.size() * max_forst_sz);		
			nodes.emplace_back(); //default init root node

			calculate_cost(0, bvh2.nodes[0].aabb.surface_area(), bvh2);	//fill in cost table using dynamic programming (bottom up) 
			collapse(bvh2, 0, 0);										//collapse SBVH into WideBVH using the cost table
			std::cout << "WideBVH build complete "<< std::endl;
		}

		void buildUncompressed(const rtm::BVH& bvh)
		{
			decisions.resize(bvh.nodes.size() * max_forst_sz);		
			uncompressedNodes.emplace_back();
			calculate_cost(0,bvh.nodes[0].aabb.surface_area(),bvh);
			collapseUncompressed(bvh,0,0);

		}

		WideBVHNode* getNodes()
		{
			return nodes.data();
		}

		WideBVHNodeUncompressed* getUncompressedNodes()
		{
			return uncompressedNodes.data();
		}

		int* getIndices() { return indices.data(); }

		std::vector<int> indices;		 // index buffer to triangle primitives

	private:
		
			std::vector<Decision> decisions; // array to store cost and meta data for the collapse algorithm
			std::vector<WideBVHNode> nodes;  // Linearized nodes buffer for wide bvh
			std::vector<WideBVHNodeUncompressed> uncompressedNodes;

			int calculate_cost(int node_index, float root_surface_area, const rtm::BVH& bvh2)
			{
				///starts with root node for SBVH 
				const rtm::BVH::Node& node = bvh2.nodes[node_index];
				int num_primitives = 0;

				if (node.data.is_leaf)
				{
					num_primitives = node.data.lst_chld_ofst + 1; 
					assert(num_primitives == 1); //for wide bvh collapse the bvh2 should be constrained to 1 primitive per leaf node

					//SAH cost for leaf
					float cost_leaf = node.aabb.surface_area()* float(num_primitives);

					//initialize the forest to default value if leaf node
					for (int i = 0; i < max_forst_sz; i++)
					{
						decisions[node_index * max_forst_sz + i].type = Decision::Type::LEAF;
						decisions[node_index * max_forst_sz + i].cost = cost_leaf;
					}
				}
				else 
				{
					//post order recursive traverse to calculate total primitives in this subtree
					num_primitives =
						//recurse left child
						calculate_cost(node.data.fst_chld_ind, node.aabb.surface_area(), bvh2)
						+
						//recurse right child
						calculate_cost(node.data.fst_chld_ind + 1, node.aabb.surface_area(), bvh2);

					//Case for choosing a single node (i == 1 from paper)
					//use min(Cprim, Cinternal)
					{
						float cost_leaf 	  = num_primitives <= p_max ?  node.aabb.surface_area()  * float(num_primitives)  : INFINITY;
						float cost_distribute = INFINITY;
						char distribute_left  = INVALID;
						char distribute_right = INVALID;

						//Pick min from permutation of costs from left and right subtree
						for (int k = 0; k < max_forst_sz; k++)
						{
							float cost =
								decisions[(node.data.fst_chld_ind) * max_forst_sz + k].cost +
								decisions[(node.data.fst_chld_ind + 1) * max_forst_sz + (max_forst_sz - 1) - k].cost;

							if (cost < cost_distribute)
							{
								cost_distribute = cost;
								distribute_left = k;
								distribute_right = (max_forst_sz - 1) - k;
							}
						}

						float cost_internal = cost_distribute + node.aabb.surface_area();

						//Pick the min cost
						if (cost_leaf < cost_internal)
						{
							decisions[node_index * max_forst_sz].type = Decision::Type::LEAF;
							decisions[node_index * max_forst_sz].cost = cost_leaf;
						}
						else
						{
							decisions[node_index * max_forst_sz].type = Decision::Type::INTERNAL;
							decisions[node_index * max_forst_sz].cost = cost_internal;
						}

						decisions[node_index * max_forst_sz].distribute_left  = distribute_left;
						decisions[node_index * max_forst_sz].distribute_right = distribute_right;

					}


					//Create wide bvh root node for a subtree
					//Case for 1 > i <= 7 ( from paper)
					{
						for (int i = 1; i < max_forst_sz; i++)
						{
							//propagate cheapest option
							float cost_distribute = decisions[node_index * max_forst_sz + (i - 1)].cost;
							char distribute_left  = INVALID;
							char distribute_right = INVALID;

							for (int k = 0; k < i; k++)
							{
								float cost = decisions[(node.data.fst_chld_ind) * max_forst_sz + k].cost +
									decisions[(node.data.fst_chld_ind + 1) * max_forst_sz + i - k - 1].cost;

								if (cost < cost_distribute)
								{
									cost_distribute  = cost;
									distribute_left  = k;
									distribute_right = i - k - 1;
								}
							}

							decisions[node_index * max_forst_sz + i].cost = cost_distribute;

							if (distribute_left != INVALID)
							{
								decisions[node_index * max_forst_sz + i].type			  = Decision::Type::DISTRIBUTE;
								decisions[node_index * max_forst_sz + i].distribute_left  = distribute_left;
								decisions[node_index * max_forst_sz + i].distribute_right = distribute_right;
							}
							else
							{
								decisions[node_index * max_forst_sz + i] = decisions[node_index * max_forst_sz + i - 1];
							}
						}
					}
				}
				return num_primitives;
			}


			//Recursive count of triangles in a subtree
			int count_primitives(int node_index, const rtm::BVH& bvh2)
			{
				const rtm::BVH::Node bvh2node = bvh2.nodes[node_index];

				if (bvh2node.data.is_leaf)
				{
					int count = bvh2node.data.lst_chld_ofst + 1;
					assert(count == 1);

					for (uint32_t i = 0; i < count; i++)
					{
						indices.push_back(bvh2node.data.fst_chld_ind + i);
					}

					return count;
				}

				return
					count_primitives(bvh2node.data.fst_chld_ind, bvh2) +
					count_primitives(bvh2node.data.fst_chld_ind + 1, bvh2);
			}

			void get_children(int node_index, const rtm::BVH& bvh2, int children[n_ary_sz], int& child_count, int i)
			{
				const rtm::BVH::Node& bvh2node = bvh2.nodes[node_index];

				if (bvh2node.data.is_leaf)
				{
					children[child_count++] = node_index; //Return self index if leaf node
					return;
				}

				char distribute_left = decisions[node_index * max_forst_sz + i].distribute_left;
				char distribute_right = decisions[node_index * max_forst_sz + i].distribute_right;

				assert(distribute_left >= 0 && distribute_left < max_forst_sz);
				assert(distribute_right >= 0 && distribute_right < max_forst_sz);

				//Recurse on left child if it needs to distribute
				if (decisions[bvh2node.data.fst_chld_ind * max_forst_sz + distribute_left].type == Decision::Type::DISTRIBUTE)
				{
					get_children(bvh2node.data.fst_chld_ind, bvh2, children, child_count, distribute_left);
				}
				else
				{
					children[child_count++] = bvh2node.data.fst_chld_ind;
				}

				//Recurse on right child if it needs to distribute
				if (decisions[(bvh2node.data.fst_chld_ind + 1) * max_forst_sz + distribute_right].type == Decision::Type::DISTRIBUTE)
				{
					get_children(bvh2node.data.fst_chld_ind + 1, bvh2, children, child_count, distribute_right);
				}
				else
				{
					children[child_count++] = bvh2node.data.fst_chld_ind + 1;
				}
			}

			//MAP n_ary_nodes to each interior node for wide bvh.
			//Each internal node has a distributed forest associated with it
			void collapse(const rtm::BVH& bvh2, int node_index_wbvh, int node_index_bvh2)
			{

				WideBVHNode& node = nodes.at(node_index_wbvh);
				const rtm::AABB aabb = bvh2.nodes.at(node_index_bvh2).aabb;

				node.p = aabb.min;
				constexpr int Nq = 8;		  // 8 Bits Per Plane
				constexpr float denom = 1.0f / float((1 << Nq) - 1);

				rtm::vec3 e(
					exp2f(ceilf(log2f((aabb.max.x - aabb.min.x) * denom))),
					exp2f(ceilf(log2f((aabb.max.y - aabb.min.y) * denom))),
					exp2f(ceilf(log2f((aabb.max.z - aabb.min.z) * denom))));

				rtm::vec3 one_over_e = (1.0f / e.x, 1.0f / e.y, 1.0f / e.z);

				uint32_t u_ex = {};
				uint32_t u_ey = {};
				uint32_t u_ez = {};

				memcpy(&u_ex, &e.x, sizeof(float));
				memcpy(&u_ey, &e.y, sizeof(float));
				memcpy(&u_ez, &e.z, sizeof(float));

				//Only the exponent bits can be non-zero
				assert((u_ex & 0b10000000011111111111111111111111) == 0);
				assert((u_ey & 0b10000000011111111111111111111111) == 0);
				assert((u_ez & 0b10000000011111111111111111111111) == 0);

				//Store Only 8 bit exponent
				node.e[0] = u_ex >> 23;
				node.e[1] = u_ey >> 23;
				node.e[2] = u_ez >> 23;

				int child_count = 0;
				int children[n_ary_sz];
				for (int i = 0; i < n_ary_sz; i++) { children[i] = INVALID; }

				//Get child nodes for this node based on the decision array costs
				get_children(node_index_bvh2, bvh2, children, child_count, 0);
				assert(child_count <= n_ary_sz);

				//TODO: Order children here based on octant traversal order

				node.imask = 0;
				node.base_index_triangle = uint32_t(indices.size());
				node.base_index_child = uint32_t(nodes.size());

				int num_internal_nodes = 0;
				int num_triangles = 0;

				for (int i = 0; i < n_ary_sz; i++)
				{
					int child_index = children[i];
					if (child_index == INVALID) continue;
					const AABB& child_aabb = bvh2.nodes[child_index].aabb;

					//Store the compressed child node
					node.q_min_x[i] = uint8_t(floorf((child_aabb.min.x - node.p.x) * one_over_e.x));
					node.q_min_y[i] = uint8_t(floorf((child_aabb.min.y - node.p.y) * one_over_e.y));
					node.q_min_z[i] = uint8_t(floorf((child_aabb.min.z - node.p.z) * one_over_e.z));
					node.q_max_x[i] = uint8_t(ceilf((child_aabb.max.x - node.p.x) * one_over_e.x));
					node.q_max_y[i] = uint8_t(ceilf((child_aabb.max.y - node.p.y) * one_over_e.y));
					node.q_max_z[i] = uint8_t(ceilf((child_aabb.max.z - node.p.z) * one_over_e.z));

					switch (decisions[child_index * max_forst_sz].type)
					{
						case Decision::Type::LEAF:
						{
							int triangle_count = count_primitives(child_index, bvh2);//collect triangles in the current subtree recursively
							assert(triangle_count > 0 && triangle_count <= p_max);
							//Three highest bits contain unary representation of triangle count
							for (int j = 0; j < triangle_count; j++)
							{
								node.meta[i] |= (1 << (j + 5));
							}
							node.meta[i] |= num_triangles;
							num_triangles += triangle_count;
							assert(num_triangles <= 24);
							break;
						}

						case Decision::Type::INTERNAL:
						{
							node.meta[i] = (i + 24) | 0b00100000;
							node.imask |= (1 << i);
							num_internal_nodes++;
							break;
						}
						default:
							//unreachable
							assert(false);
							break;
					}
				}

				for (int i = 0; i < num_internal_nodes; i++)
				{
					nodes.emplace_back();
				}

			
				assert((nodes.at(node_index_wbvh).base_index_child + num_internal_nodes) == nodes.size());
				assert((nodes.at(node_index_wbvh).base_index_triangle + num_triangles) == indices.size());

				//Recurse on internal nodes
				uint32_t offset = 0;
				for (int i = 0; i < n_ary_sz; i++)
				{
					
					int child_index = children[i];
					if (child_index == INVALID) continue;

					if (nodes.at(node_index_wbvh).imask & (1 << i))
					{
						collapse(bvh2, nodes.at(node_index_wbvh).base_index_child + offset++, child_index);
					}
				}
			}

			void collapseUncompressed(const rtm::BVH& bvh2, int node_index_wbvh, int node_index_bvh2)
			{
				WideBVHNodeUncompressed& bvh8Node = uncompressedNodes[node_index_wbvh];
				const BVH::Node& bvh2Node = bvh2.nodes[node_index_bvh2];
				
				bvh8Node.base_index_child = uncompressedNodes.size();

				int child_count = 0;
				int children[n_ary_sz];
				for (int i = 0; i < n_ary_sz; i++) { children[i] = INVALID; }

				//Get child nodes for this node based on the decision array costs
				get_children(node_index_bvh2, bvh2, children, child_count, 0);
				assert(child_count <= n_ary_sz);

				int num_internal_nodes = 0;
				int index = 0;

				for(int i = 0; i < n_ary_sz; i++)
				{
					int num_triangles = 0;
					int first_child_index = 0;
					int child_index = children[i];
					if (child_index == INVALID) continue;
					switch(decisions[child_index * max_forst_sz].type)
					{
						//Caution: This wide bvh leaf node might have more than 1 leaf node
						case Decision::Type::LEAF:
							first_child_index = indices.size();
							num_triangles = count_primitives(child_index, bvh2);
							bvh8Node.nodeArray[index].data.is_leaf = 1; //make it leaf node
							bvh8Node.nodeArray[index].data.fst_chld_ind  = first_child_index;
							bvh8Node.nodeArray[index].data.lst_chld_ofst = num_triangles - 1;
							bvh8Node.nodeArray[index].aabb = bvh2.nodes[child_index].aabb;

							index++;
							break;
						case Decision::Type::INTERNAL:
							bvh8Node.nodeArray[index] = bvh2.nodes[child_index];
							num_internal_nodes++;
							index++;
							break;
						default:
							assert(true);
							break; 
					}
				}

				
				bvh8Node.childCount = num_internal_nodes; //save the active node count 

				for(int i =0; i < num_internal_nodes; i++)
				{
					uncompressedNodes.emplace_back();
				}

				assert((uncompressedNodes[node_index_wbvh].base_index_child + num_internal_nodes) == uncompressedNodes.size());

				int offset = 0;
				index = 0;
				for(int i = 0; i < n_ary_sz; i++)
				{
					int child_index = children[i];
					if(child_index == INVALID) continue;
					if(uncompressedNodes[node_index_wbvh].nodeArray[index++].data.is_leaf == 0)
					{
						collapseUncompressed(bvh2, uncompressedNodes[node_index_wbvh].base_index_child + offset++, child_index);
					}
				}
			}

			void order_children();
		};
}