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


	void WideBVH::collapse(const rtm::BVH& bvh)
	{

	}

	void WideBVH::get_children(int node_index, const rtm::BVH& bvh, int children[8], int& child_count, int i)
	{

	}

}