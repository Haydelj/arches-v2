#pragma once

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion.
//https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs

#include "int.hpp"
#include "aabb.hpp"

#ifndef __riscv
#include <vector>
#include <algorithm>
#include <cassert>
#endif


namespace rtm
{

#define n_ary_sz 8 // tree width
#define max_frst_sz (n_ary_sz-1) //max forest size
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
		WideBVH(const rtm::BVH& bvh);
		~WideBVH();

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

		struct WideBVHNode
		{
			rtm::vec3 p[3];						//anchor point 
			uint8_t  e[3];						//exponent power of 2 for local grid scale
			uint8_t  imask;						//internal node flag
			uint32_t base_index_child;			//base offset in streamlined child node array
			uint32_t base_index_triangle;		//base offset in streamlined primitive array
			uint8_t  meta[8];					//count and indexing offset
			uint8_t  q_min_x[8] = {};			//quantized bb 
			uint8_t  q_min_y[8] = {};
			uint8_t  q_min_z[8] = {};
			uint8_t  q_max_x[8] = {};
			uint8_t  q_max_y[8] = {};
			uint8_t  q_max_z[8] = {};
		};

		std::vector<Decision> decisions; // Finite static memory array to store cost and meta data for the collapse algorithm
		std::vector<WideBVHNode> nodes;  // Linearized nodes buffer for wide bvh
		std::vector<int> indices;		 // index buffer to triangle primitives

		int		calculate_cost(int node_index, float parent_surface_area,const rtm::BVH& bvh);
		void	collapse(const rtm::BVH& bvh);
		void	get_children(int node_index, const rtm::BVH& bvh, int children[8], int& child_count, int i);
		void	order_children();
		int count_primitives(int node_index, const rtm::BVH& bvh2)
			

	};
	

}