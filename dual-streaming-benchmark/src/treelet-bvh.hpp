#pragma once
#include "stdafx.hpp"

#define TREELET_SIZE (7 * 8 * 1024)

struct alignas(8 * 1024) Treelet
{
	struct alignas(32) Header
	{
		uint first_child;
		uint num_children;
	};

	struct alignas(32) Node
	{
		union Data
		{
			struct
			{
				uint32_t is_leaf    : 1;
				uint32_t num_tri    : 3;
				uint32_t tri0_word0 : 16;
			};

			struct
			{
				uint32_t            : 1;
				uint32_t is_treelet : 1;
				uint32_t index      : 30;
			}
			child[2];
		};

		rtm::AABB aabb;
		Data      data;
	};

	struct Triangle
	{
		rtm::Triangle tri;
		uint          id;
	};

	union
	{
		struct
		{
			Header header;
			Node   nodes[(TREELET_SIZE - sizeof(Header)) / sizeof(Node)];
		};
		uint32_t words[TREELET_SIZE / sizeof(uint32_t)];
	};

	Treelet() {}
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
		uint node_size = sizeof(Treelet::Node);
		if(bvh.nodes[node].data.is_leaf)
			node_size += sizeof(Treelet::Triangle) * (bvh.nodes[node].data.lst_chld_ofst + 1);

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
			uint bytes_remaining = sizeof(Treelet) - sizeof(Treelet::Header);
			best_cost[root_node] = INFINITY;
			while(cut.size() < 16)
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
		std::vector<Treelet::Header> treelet_headers;
		std::vector<std::vector<uint>> treelet_assignments;
		std::unordered_map<uint, uint> root_node_treelet;

		std::queue<uint> root_node_queue;
		root_node_queue.push(0);
		while(!root_node_queue.empty())
		{
			uint root_node = root_node_queue.front();
			root_node_queue.pop();

			root_node_treelet[root_node] = treelet_assignments.size();
			treelet_assignments.push_back({});

			std::vector<uint> cut{root_node};
			uint bytes_remaining = sizeof(Treelet);
			while(cut.size() < 16)
			{
				uint best_index = ~0u;
				float best_score = -INFINITY;
				for(uint i = 0; i < cut.size(); ++i)
				{
					uint n = cut[i];
					if(footprint[n] <= bytes_remaining)
					{
						float gain = area[n] + epsilon;
						float price = rtm::min(subtree_footprint[n], sizeof(Treelet));
						float score = gain / price;
						if(score > best_score)
						{
							best_index = i;
							best_score = score;
						}
					}
				}
				if(best_index == ~0u) break;

				treelet_assignments.back().push_back(cut[best_index]);

				//maintain the cut in breadth first ordering so that sibling nodes that are treelet roots are placed in adjacent treelets
				uint best_node = cut[best_index];
				cut.erase(cut.begin() + best_index);
				if(!bvh.nodes[best_node].data.is_leaf)
					for(uint i = 0; i <= bvh.nodes[best_node].data.lst_chld_ofst; ++i)
						cut.insert(cut.begin() + best_index + i, bvh.nodes[best_node].data.fst_chld_ind + i);

				bytes_remaining -= footprint[best_node];

				float cost = area[root_node] + epsilon;
				for(auto& n : cut)
					cost += best_cost[n];

				if(cost == best_cost[root_node]) break;
			}

			treelet_headers.push_back({(uint)(root_node_queue.size() + treelet_assignments.size()), (uint)cut.size()});

			//we use a queue so that treelets are breadth first in memory
			for(auto& n : cut)
				root_node_queue.push(n);
		}



		//Phase 3 construct treelets in memeory
		treelets.resize(treelet_assignments.size());

		for(uint treelet_index = 0; treelet_index < treelets.size(); ++treelet_index)
		{
			uint nodes_mapped = 0;
			std::unordered_map<uint, uint> node_map;
			node_map[treelet_assignments[treelet_index][0]] = nodes_mapped++;

			Treelet& treelet = treelets[treelet_index];
			treelet.header = treelet_headers[treelet_index];
			uint primative_start = (treelet_assignments[treelet_index].size() * sizeof(Treelet::Node) + sizeof(Treelet::Header)) / 4;
			for(uint j = 0; j < treelet_assignments[treelet_index].size(); ++j)
			{
				uint node_id = treelet_assignments[treelet_index][j];
				const rtm::BVH::Node& node = bvh.nodes[node_id];

				assert(node_map.find(node_id) != node_map.end());
				uint tnode_id = node_map[node_id];
				Treelet::Node& tnode = treelets[treelet_index].nodes[tnode_id];

				tnode.aabb = node.aabb;
				tnode.data.is_leaf = node.data.is_leaf;
				if(node.data.is_leaf)
				{
					tnode.data.num_tri = node.data.lst_chld_ofst;
					tnode.data.tri0_word0 = primative_start;

					Treelet::Triangle* tris = (Treelet::Triangle*)(&treelet.words[primative_start]);
					for(uint k = 0; k <= node.data.lst_chld_ofst; ++k)
					{
						tris[k].id = node.data.fst_chld_ind + k;
						tris[k].tri = mesh.get_triangle(node.data.fst_chld_ind + k);
					}

					primative_start += (sizeof(Treelet::Triangle) / 4) * (node.data.lst_chld_ofst + 1);
				}
				else
				{
					assert(node.data.lst_chld_ofst == 1);
					for(uint i = 0; i < 2; ++i)
					{
						uint child_node_id = node.data.fst_chld_ind + i;
						if(root_node_treelet.find(child_node_id) != root_node_treelet.end())
						{
							tnode.data.child[i].is_treelet = 1;
							tnode.data.child[i].index = root_node_treelet[child_node_id];
						}
						else
						{
							tnode.data.child[i].is_treelet = 0;
							tnode.data.child[i].index = node_map[child_node_id] = nodes_mapped++;
						}
					}
				}
			}
		}

		printf("Treelets: %zu\n", treelets.size());
		printf("Treelet Fill Rate: %.1f%%\n", 100.0f * total_footprint / treelets.size() / (sizeof(Treelet) - sizeof(Treelet::Header)));
	}
};
#endif
