#pragma once

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion.

#include "int.hpp"
#include "aabb.hpp"

#ifndef __riscv
#include <vector>
#include <algorithm>
#include <cassert>
#endif

namespace rtm
{

	WideBVH::WideBVH(const rtm::BVH& bvh)
	{
		decisions.resize(bvh.nodes.size() * (n_ary_sz- 1));		//as per implementation in paper
		nodes.emplace_back(); //default init root node
		calculate_cost(0,bvh.nodes[0].aabb.surface_area(),bvh);	//fill in cost table using dynamic programming (bottom up)
		collapse(bvh);											//collapse SBVH into WideBVH using the cost table
	}

	

	int WideBVH::calculate_cost(int node_index, float parent_surface_area, const rtm::BVH& bvh)
	{
		rtm::BVH::Node& node = bvh.nodes[node_index];
		int num_primitives = 0;

		if (node.data.is_leaf)
		{
			num_primitives = node.data.lst_chld_ofst;
			assert(num_primitives == 1); //for wide bvh collapse the bvh2 should be constrained to 1 primitive per leaf node

			//SAH cost for leaf
			float cost_leaf = (node.aabb.surface_area() / parent_surface_area) * float(num_primitives);

			for (int i = 0; i < max_frst_sz; i++)
			{
				decisions[node_index * max_frst_sz + i].type = Decision::Type::LEAF;
				decisions[node_index * max_frst_sz + i].cost = cost_leaf;
			}
		}
		else
		{
			//post order recursive traverse
			num_primitives = calculate_cost( node.data.fst_chld_ind, node.aabb.surface_area(), bvh) + calculate_cost(node.data.fst_chld_ind + 1, node.aabb.surface_area(), bvh);


			//Case for i == 1 (from paper)
			{
				float cost_leaf = num_primitives <= p_max ? float(num_primitives) * (node.aabb.surface_area() / parent_surface_area) : INFINITY;
				float cost_distribute	= INFINITY;
				char distribute_left	= INVALID;
				char distribute_right	= INVALID;

				//pick min from permutation of costs from left and right subtree
				for (int k = 0; k < max_frst_sz; k++)
				{
					float cost = decisions[(node.data.fst_chld_ind) * max_frst_sz + k] + decisions[(node.data.fst_chld_ind + 1) * max_frst_sz + (max_frst_sz - 1) - k];

					if (cost < cost_distribute)
					{
						cost_distribute		= cost;
						distribute_left		= k;
						distribute_right	= (max_frst_sz - 1) - k;
					}
				}

				float cost_internal = cost_distribute + (node.aabb.surface_area() / parent_surface_area);

				if (cost_leaf < cost_internal)
				{
					decisions[node_index * max_frst_sz].type = Decision::Type::LEAF;
					decisions[node_index * max_frst_sz].cost = cost_leaf;
				}
				else
				{
					decisions[node_index * max_frst_sz].type = Decision::Type::INTERNAL;
					decisions[node_index * max_frst_sz].cost = cost_internal;
				}

				decisions[node_index * max_frst_sz].distribute_left	= distribute_left;
				decisions[node_index * max_frst_sz].distribute_right	= distribute_right;

			}

			//Case for 1 > i <= 7 ( from paper)
			{
				for (int i = 1; i < max_frst_sz; i++)
				{
					float cost_distribute	= decisions[node_index * max_frst_sz + (i - 1)].cost;
					char distribute_left	= INVALID;
					char distribute_right	= INVALID;
					
					for (int k = 0; k < i; k++)
					{
						float cost = decisions[(node.data.fst_chld_ind) * max_frst_sz + k].cost +
								     decisions[(node.data.fst_chld_ind + 1) * max_frst_sz + i - k - 1].cost;

						if (cost < cost_distribute)
						{
							cost_distribute		= cost;
							distribute_left		= k;
							distribute_right	= i - k - 1;
						}
					}

					decisions[node_index * max_frst_sz + i].cost = cost_distribute;

					if (distribute_left != INVALID)
					{
						decisions[node_index * max_frst_sz + i].type = Decision::Type::DISTRIBUTE;
						decisions[node_index * max_frst_sz + i].distribute_left	= distribute_left;
						decisions[node_index * max_frst_sz + i].distribute_right = distribute_right;
					}
					else
					{
						decisions[node_index * max_frst_sz + i] = decisions[node_index * max_frst_sz + i - 1];
					}
				}
			}
		}

		return num_primitives;
	}


	void WideBVH::collapse(const rtm::BVH& bvh2, int node_index_wbvh, int node_index_bvh2)
	{

		WideBVHNode& node = nodes[node_index_wbvh];
		const rtm::AABB aabb = bvh2.nodes[node_index_bvh2].aabb();

		node.p = aabb.min();
		constexpr int Nq = 8;
		constexpr float denom = 1.0f / float((1 << Nq) - 1);

		rtm::vec3 e(
			exp2f(ceilf(log2f(aabb.max.x - aabb.min.x) * denom))),
			exp2f(ceilf(log2f(aabb.max.y - aabb.min.y) * denom))),
			exp2f(ceilf(log2f(aabb.max.z - aabb.min.z) * denom)))
			);

			rtm::vec3 one_over_e = (1.0f / e.x, 1.0f / e.y, 1.0f / e.z);

			uint32_t u_ex = {};
			uint32_t u_ey = {};
			uint32_t u_ez = {};

			memcpy(u_ex, e.x, sizeof(float));
			memcpy(u_ey, e.y, sizeof(float));
			memcpy(u_ez, e.z, sizeof(float));

			//Only the exponent bits can be non-zero
			assert((u_ex & 0b10000000011111111111111111111111) == 0);
			assert((u_ey & 0b10000000011111111111111111111111) == 0);
			assert((u_ez & 0b10000000011111111111111111111111) == 0);

			//Store only 8 bit exponent
			node.e[0] = u_ex >> 23;
			node.e[1] = u_ey >> 23;
			node.e[2] = u_ez >> 23;


			int child_count = 0;
			int children[n_ary_sz] = { INVALID, INVALID, INVALID, INVALID, INVALID, INVALID, INVALID, INVALID };

			//get children for this node based on the decision array costs
			get_children(node_index_bvh2, bvh2, children, child_count, 0);
			assert(child_count <= n_ary_sz);
			
			//order children based on octant traversal order

			node.imask = 0;
			node.base_index_triangle = uint32_t(indices.size());
			node.base_index_child = uint32_t(nodes.size());

			int num_internal_nodes = 0;
			int num_triangles = 0;

			for (int = 0; i < n_ary_sz; i++)
			{
				int child_index = children[i];
				if (child_index == INVALID) continue;
				const AABB& child_aabb = bvh2.nodes[child_index].aabb();

				//Store the compressed child node
				node.q_min_x[i] = uint8_t(floorf((child_aabb.min.x - node.p.x) * one_over_e.x));
				node.q_min_y[i] = uint8_t(floorf((child_aabb.min.y - node.p.y) * one_over_e.y));
				node.q_min_z[i] = uint8_t(floorf((child_aabb.min.z - node.p.z) * one_over_e.z));

				node.q_max_x[i] = uint8_t(floorf((child_aabb.max.x - node.p.x) * one_over_e.x));
				node.q_max_y[i] = uint8_t(floorf((child_aabb.max.y - node.p.y) * one_over_e.y));
				node.q_max_z[i] = uint8_t(floorf((child_aabb.max.z - node.p.z) * one_over_e.z));

				switch (decisions[child_index * 7].type)
				{
					case Decision::Type::LEAF:
						int triangle_count = count_primitives(child_index,bvh2);//collect triangles in the current subtree recursively
						assert(triangle_count > 0 && triangle_count <= 3);
						//Three highest bits contain unary representation of triangle count
						for (int j = 0; j < triangle_count; j++)
						{
							node.meta[i] |= (1 << (j + 5));
						}
						node.meta[i] |= num_triangles;
						num_triangles += triangle_count;
						assert(num_triangles <= 24);
						break;
					case Decision::Type::INTERNAL:
						node.meta[i] = (i + 24) | 0b00100000;
						node.imask |= (1 << i);
						num_internal_nodes++;
						break;
					default:
						//unreachable
						break;
				}
			}

			for (int i = 0; i < num_internal_nodes; i++)
			{
				nodes.emplace_back();
			}

			node = nodes[node_index_wbvh];

			assert(node.base_index_child + num_internal_nodes == nodes.size());
			assert(node.base_index_triangle + num_triangles == indices.size());

			//Recurse on internal nodes
			int offset = 0;
			for (int i = 0; i < n_ary_sz; i++)
			{
				int child_index = children[i];
				if (child_index == INVALID) continue;

				if (node.imask & (1 << i))
				{
					collapse(bvh2, node.base_index_child + offset++, child_index);
				}
			}
	}

	//recursive count of triangles in a subtree
	int WideBVH::count_primitives(int node_index, const rtm::BVH& bvh2)
	{
		const rtm::BVH::Node bvh2node = bvh2.nodes[node_index];

		if (bvh2node.data.is_leaf)
		{
			int count = bvh2node.data.lst_chld_ofst;
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


	void WideBVH::get_children(int node_index, const rtm::BVH& bvh2, int children[8], int& child_count, int i)
	{
		const rtm::BVH::Node& bvh2node = bvh2.nodes[node_index];
		
		if (bvh2node.data.is_leaf)
		{
			children[child_count++] = node_index;
			return;
		}

		char distribute_left = decisions[node_index * 7 + i].distribute_left;
		char distribute_right = decisions[node_index * 7 + i].distribute_right;

		assert(distribute_left >= 0 && distribute_left < 7);
		assert(distribute_right >= 0 && distribute_right < 7);

		//recurse on left child if it needs to distribute
		if (decisions[bvh2node.Data.fst_chld_ind * 7 + distribute_left].type == Decision::Type::DISTRIBUTE)
		{
			get_children(bvh2node.Data.fst_chld_ind, bvh2, children, child_count, distribute_right);
		}
		else
		{
			children[child_count++] = bvh2node.Data.fst_chld_ind;
		}

		//recurse on right child if it needs to distribute
		if (decisions[(bvh2node.Data.fst_chld_ind + 1) * 7 + distribute_right].type == Decision::Type::DISTRIBUTE)
		{
			get_children(bvh2node.Data.fst_chld_ind + 1, bvh2, children, child_count, distribute_right);
		}
		else
		{
			children[child_count++] = bvh2node.Data.fst_chld_ind + 1;
		}

	}

}