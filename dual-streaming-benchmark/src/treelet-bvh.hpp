#pragma once
#include "stdafx.hpp"

struct alignas(32) TreeletNode
{
	rtm::AABB aabb;
	union
	{
		uint32_t data;
		struct
		{
			uint32_t is_leaf         : 1;
			uint32_t is_treelet_leaf : 1;
			uint32_t last_tri_offset : 3;
			uint32_t child_index     : 27;
		};
	};
};

struct alignas(32) TreeletHeader
{
	uint first_child;
	uint num_children;
};

struct TreeletTriangle
{
	rtm::Triangle tri;
	uint          id;
};

#define TREELET_SIZE (64 * 1024)

struct alignas(8 * 1024) Treelet
{
	uint8_t	data[64 * 1024];
};

#ifndef __riscv
class TreeletBVH
{
public:
	std::vector<Treelet> treelets;

	TreeletBVH(const rtm::BVH& bvh, const rtm::Mesh& mesh)
	{
		printf("Treelet BVH Building\n");
		build_treelet(bvh, mesh);
		printf("Treelet BVH Built\n");
	}

	uint get_node_size(uint node, const rtm::BVH& bvh, const rtm::Mesh& mesh)
	{
		uint node_size = sizeof(TreeletNode);
		if(bvh.nodes[node].data.is_leaf)
			node_size += sizeof(TreeletTriangle) * (bvh.nodes[node].data.lst_chld_ofst + 1);

		return node_size;
	}

	void build_treelet(const rtm::BVH& bvh, const rtm::Mesh& mesh)
	{
		//Phase 0 setup
		uint total_footprint = 0;
		std::vector<uint> footprint;
		std::vector<float> area;
		std::vector<float> best_cost;
		for(uint i = 0; i < bvh.nodes.size(); ++i)
		{
			footprint.push_back(get_node_size(i, bvh, mesh));
			total_footprint += footprint.back();
			area.push_back(bvh.nodes[i].aabb.surface_area());
			best_cost.push_back(INFINITY);
		}

		float epsilon = bvh.nodes[0].aabb.surface_area() * sizeof(Treelet) / (10 * total_footprint);

		std::stack<uint> post_stack;
		std::stack<uint> pre_stack; pre_stack.push(0);
		while(!pre_stack.empty())
		{
			uint node = pre_stack.top();
			pre_stack.pop();

			if(!bvh.nodes[node].data.is_leaf)
				for(uint i = 0; i <= bvh.nodes[node].data.lst_chld_ofst; ++i)
					pre_stack.push(bvh.nodes[node].data.fst_chld_ind + i);

			post_stack.push(node);
		}



		//Phase 1 reverse depth first search using dynamic programing to determine treelet costs
		std::vector<uint> subtree_footprint;
		subtree_footprint.resize(bvh.nodes.size(), 0);
		while(!post_stack.empty())
		{
			uint root_node = post_stack.top(); 
			post_stack.pop();

			subtree_footprint[root_node] = footprint[root_node];
			if(!bvh.nodes[root_node].data.is_leaf)
				for(uint i = 0; i <= bvh.nodes[root_node].data.lst_chld_ofst; ++i)
					subtree_footprint[root_node] += subtree_footprint[bvh.nodes[root_node].data.fst_chld_ind + i];

			std::set<uint> cut{root_node};
			uint bytes_remaining = sizeof(Treelet);
			best_cost[root_node] = INFINITY;
			while(true)
			{
				uint best_node = ~0u;
				float best_score = -INFINITY;
				for(auto& n : cut)
				{
					if(footprint[n] <= bytes_remaining)
					{
						float gain = area[n] + epsilon;
						float price = rtm::min(subtree_footprint[n], sizeof(Treelet));
						float score = gain / price;
						if(score > best_score)
						{
							best_node = n;
							best_score = score;
						}
					}
				}
				if(best_node == ~0u) break;

				cut.erase(best_node);
				if(!bvh.nodes[best_node].data.is_leaf)
					for(uint i = 0; i <= bvh.nodes[best_node].data.lst_chld_ofst; ++i)
						cut.insert(bvh.nodes[best_node].data.fst_chld_ind + i);

				bytes_remaining -= footprint[best_node];

				float cost = area[root_node] + epsilon;
				for(auto& n : cut)
					cost += best_cost[n];

				best_cost[root_node] = rtm::min(best_cost[root_node], cost);
			}
		}



		//Phase 2 treelet assignment
		pre_stack.push(0);
		std::vector<std::vector<uint>> treelet_assignments;
		while(!pre_stack.empty())
		{
			treelet_assignments.push_back({});
			uint root_node = pre_stack.top();
			pre_stack.pop();

			std::set<uint> cut{root_node};
			uint bytes_remaining = sizeof(Treelet);
			while(true)
			{
				uint best_node = ~0u;
				float best_score = -INFINITY;
				for(auto& n : cut)
				{
					if(footprint[n] <= bytes_remaining)
					{
						float gain = area[n] + epsilon;
						float price = rtm::min(subtree_footprint[n], sizeof(Treelet));
						float score = gain / price;
						if(score > best_score)
						{
							best_node = n;
							best_score = score;
						}
					}
				}
				if(best_node == ~0u) break;

				treelet_assignments.back().push_back(best_node);

				cut.erase(best_node);
				if(!bvh.nodes[best_node].data.is_leaf)
					for(uint i = 0; i <= bvh.nodes[best_node].data.lst_chld_ofst; ++i)
						cut.insert(bvh.nodes[best_node].data.fst_chld_ind + i);

				bytes_remaining -= footprint[best_node];

				float cost = area[root_node] + epsilon;
				for(auto& n : cut)
					cost += best_cost[n];

				if(cost == best_cost[root_node]) break;
			}

			for(auto& n : cut)
				pre_stack.push(n);
		}



		//Phase 3 construct treelets in memeory
		treelets.resize(treelet_assignments.size());

		std::unordered_map<uint, std::pair<uint, uint>> node_map;

		uint num_nodes_assigned = 0;
		uint num_tris_assigned = 0;

		for(uint i = treelets.size() - 1; i != ~0u; --i)
		{
			uint num_nodes = treelet_assignments[i].size();
			num_nodes_assigned += num_nodes;

			for(uint j = 0; j < num_nodes; ++j)
			{
				uint node_id = treelet_assignments[i][j];
				rtm::BVH::Node node = bvh.nodes[node_id];
				node_map[node_id].first = i;
				node_map[node_id].second = j * (sizeof(TreeletNode) / 4);
			}

			uint current_word = num_nodes * (sizeof(TreeletNode) / 4);

			for(uint j = 0; j < num_nodes; ++j)
			{
				uint node_id = treelet_assignments[i][j];
				rtm::BVH::Node node = bvh.nodes[node_id];

				TreeletNode& tnode = ((TreeletNode*)treelets[i].data)[j];
				tnode.aabb = node.aabb;
				tnode.is_leaf = node.data.is_leaf;

				if(node.data.is_leaf)
				{
					tnode.last_tri_offset = node.data.lst_chld_ofst;
					tnode.child_index = current_word;

					TreeletTriangle* tris = (TreeletTriangle*)(&treelets[i].data[current_word * 4]);
					for(uint k = 0; k <= node.data.lst_chld_ofst; ++k)
					{
						num_tris_assigned++;
						tris[k].id = node.data.fst_chld_ind + k;

						tris[k].tri = mesh.get_triangle(node.data.fst_chld_ind + k);
					}

					current_word += (sizeof(TreeletTriangle) / 4) * (node.data.lst_chld_ofst + 1);
				}
				else
				{
					tnode.is_treelet_leaf = node_map[node.data.fst_chld_ind].first != i;
					if(tnode.is_treelet_leaf) tnode.child_index = node_map[node.data.fst_chld_ind].first;
					else                      tnode.child_index = node_map[node.data.fst_chld_ind].second;

				}
			}
		}

		printf("Nodes Assigned: %u/%zu\n", num_nodes_assigned, bvh.nodes.size());
		printf("Tris Assigned: %u/%zu\n", num_tris_assigned, mesh.vertex_indices.size());
		printf("Treelets: %zu\n", treelets.size());
		printf("Bytes/Treelet: %zu\n", total_footprint / treelets.size());
	}
};
#endif
