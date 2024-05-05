#pragma once

#include "packed-bvh.hpp"
#include "mesh.hpp"

#ifndef __riscv
#include <map>
#include <set>
#include <unordered_map>
#include <stack>
#include <queue>
#endif

namespace rtm {

#define PACK_SIZE 2

struct PackedTreelet
{
	const static uint size = 16 * 1024 * 1024; // 16 MB

	struct alignas(32 * PACK_SIZE) Header
	{
		uint first_child;
		uint num_children;
		uint subtree_size;
		uint depth;
	};

	struct alignas(32 * PACK_SIZE) Node
	{
		union Data
		{
			struct
			{
				uint32_t is_leaf : 1;
				uint32_t num_tri : 3;
				uint32_t tri_offset : 28;
			};
			struct
			{
				uint32_t                  : 1;
				uint32_t is_child_treelet : 1;
				uint32_t child_index      : 30;
			};
		};

		//rtm::AABB aabb;
		rtm::AABB aabb[PACK_SIZE];
		Data      data[PACK_SIZE];
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
			Node   nodes[(size - sizeof(Header)) / sizeof(Node)];
		};
		uint8_t bytes[size];
	};

	PackedTreelet() {}
};

#ifndef __riscv
class PackedTreeletBVH
{
public:
	std::vector<PackedTreelet> treelets;

	PackedTreeletBVH(){}
	PackedTreeletBVH(const rtm::PackedBVH2& bvh, const std::vector<Triangle>& triangles)
	{
		printf("Packed Treelet BVH Building\n");
		build_treelet(bvh, triangles);
		printf("Packed Treelet BVH Built\n");
	}

	uint get_node_size(uint node, const rtm::PackedBVH2& bvh)
	{
		uint node_size = sizeof(PackedTreelet::Node);
		for(uint i = 0; i < PACK_SIZE; ++i)
			if (bvh.nodes[node].data[i].is_leaf)
				node_size += sizeof(PackedTreelet::Triangle) * (bvh.nodes[node].data[i].num_tri + 1);
		return node_size;
	}

	void build_treelet(const rtm::PackedBVH2& bvh, const std::vector<Triangle>& triangles, uint max_cut_size = 1024)
	{
		size_t usable_space = sizeof(PackedTreelet) - sizeof(PackedTreelet::Header);

		//Phase 0 setup
		uint total_footprint = 0;
		std::vector<uint> footprint;
		std::vector<float> area;
		std::vector<float> best_cost;
		for (uint i = 0; i < bvh.nodes.size(); ++i)
		{
			footprint.push_back(get_node_size(i, bvh));
			total_footprint += footprint.back();

			AABB aabb;
			for(uint j = 0; j < PACK_SIZE; ++j)
				aabb.add(bvh.nodes[i].aabb[j]);

			area.push_back(aabb.surface_area());
			best_cost.push_back(INFINITY);
		}

		float epsilon = area[0] * usable_space / (10 * total_footprint);

		std::stack<uint> post_stack;
		std::stack<uint> pre_stack; pre_stack.push(0);
		while (!pre_stack.empty())
		{
			uint node = pre_stack.top();
			pre_stack.pop();

			for(uint i = 0; i < PACK_SIZE; ++i)
				if (!bvh.nodes[node].data[i].is_leaf)
					pre_stack.push(bvh.nodes[node].data[i].child_index);

			post_stack.push(node);
		}



		//Phase 1 reverse depth first search using dynamic programing to determine treelet costs
		std::vector<uint> subtree_footprint;
		subtree_footprint.resize(bvh.nodes.size(), 0);
		while (!post_stack.empty())
		{
			uint root_node = post_stack.top();
			post_stack.pop();

			subtree_footprint[root_node] = footprint[root_node];
			
			for(uint i = 0; i < PACK_SIZE; ++i)
				if (!bvh.nodes[root_node].data[i].is_leaf)
					subtree_footprint[root_node] += subtree_footprint[bvh.nodes[root_node].data[i].child_index];

			std::set<uint> cut{ root_node };
			uint bytes_remaining = usable_space;
			best_cost[root_node] = INFINITY;
			while (cut.size() < max_cut_size)
			{
				uint best_node = ~0u;
				float best_score = -INFINITY;
				for (auto& n : cut)
				{
					if (footprint[n] <= bytes_remaining)
					{
						float gain = area[n] + epsilon;
						float price = rtm::min(subtree_footprint[n], usable_space);
						float score = gain / price;
						if (score > best_score)
						{
							best_node = n;
							best_score = score;
						}
					}
				}
				if (best_node == ~0u) break;

				cut.erase(best_node);
				for(uint i = 0; i < PACK_SIZE; ++i)
					if (!bvh.nodes[best_node].data[i].is_leaf)
						cut.insert(bvh.nodes[best_node].data[i].child_index);

				bytes_remaining -= footprint[best_node];

				float cost = area[root_node] + epsilon;
				for (auto& n : cut)
					cost += best_cost[n];

				best_cost[root_node] = rtm::min(best_cost[root_node], cost);
			}
		}



		//Phase 2 treelet assignment
		std::vector<PackedTreelet::Header> treelet_headers;
		std::vector<std::vector<uint>> treelet_assignments;
		std::unordered_map<uint, uint> root_node_treelet;

		std::map<uint, uint> parent_map;
		std::vector<uint> parent;

		std::queue<uint> root_node_queue;
		root_node_queue.push(0);
		while (!root_node_queue.empty())
		{
			uint root_node = root_node_queue.front();

			uint parent_treelet = ~0u;
			if (parent_map.count(root_node))
				parent_treelet = parent_map[root_node];
			else
				assert(root_node == 0);

			root_node_queue.pop();

			root_node_treelet[root_node] = treelet_assignments.size();
			treelet_assignments.push_back({});

			std::vector<uint> cut{ root_node };
			uint bytes_remaining = usable_space;
			while (cut.size() < max_cut_size)
			{
				uint best_index = ~0u;
				float best_score = -INFINITY;
				for (uint i = 0; i < cut.size(); ++i)
				{
					uint n = cut[i];
					if (footprint[n] <= bytes_remaining)
					{
						float gain = area[n] + epsilon;
						float price = rtm::min(subtree_footprint[n], usable_space);
						float score = gain / price;
						if (score > best_score)
						{
							best_index = i;
							best_score = score;
						}
					}
				}
				if (best_index == ~0u) break;

				treelet_assignments.back().push_back(cut[best_index]);

				//maintain the cut in breadth first ordering so that sibling nodes that are treelet roots are placed in adjacent treelets
				uint best_node = cut[best_index];
				cut.erase(cut.begin() + best_index);

				uint j = 0;
				for(uint i = 0; i < PACK_SIZE; ++i)
					if (!bvh.nodes[best_node].data[i].is_leaf)
						cut.insert(cut.begin() + best_index + j++, bvh.nodes[best_node].data[i].child_index);

				bytes_remaining -= footprint[best_node];

				float cost = area[root_node] + epsilon;
				for (auto& n : cut)
					cost += best_cost[n];

				if (cost == best_cost[root_node]) break;
			}

			uint depth = parent_treelet == ~0u ? 0 : treelet_headers[parent_treelet].depth + 1;
			treelet_headers.push_back({ (uint)(root_node_queue.size() + treelet_assignments.size()), (uint)cut.size(), 1, depth });
			parent.push_back(parent_treelet);

			//we use a queue so that treelets are breadth first in memory
			for (auto& n : cut)
			{
				parent_map[n] = treelet_assignments.size() - 1;
				root_node_queue.push(n);
			}
		}

		assert(parent.size() == treelet_headers.size());
		for (int i = parent.size() - 1; i >= 0; i--) 
		{
			int fa = parent[i];
			if (fa != -1) 
			{
				assert(fa < i);
				treelet_headers[fa].subtree_size += treelet_headers[i].subtree_size;
			}
		}

		//Phase 3 construct treelets in memeory
		treelets.resize(treelet_assignments.size());

		for (uint treelet_index = 0; treelet_index < treelets.size(); ++treelet_index)
		{
			uint nodes_mapped = 0;
			std::unordered_map<uint, uint> node_map;
			node_map[treelet_assignments[treelet_index][0]] = nodes_mapped++;

			PackedTreelet& treelet = treelets[treelet_index];
			treelet.header = treelet_headers[treelet_index];
			uint primative_start = treelet_assignments[treelet_index].size() * sizeof(PackedTreelet::Node) + sizeof(PackedTreelet::Header);
			for (uint i = 0; i < treelet_assignments[treelet_index].size(); ++i)
			{
				uint node_id = treelet_assignments[treelet_index][i];
				const rtm::PackedBVH2::Node& node = bvh.nodes[node_id];

				assert(node_map.find(node_id) != node_map.end());
				uint tnode_id = node_map[node_id];
				PackedTreelet::Node& tnode = treelets[treelet_index].nodes[tnode_id];

				for(uint j = 0; j < PACK_SIZE; ++j)
				{
					tnode.data[j].is_leaf = node.data[j].is_leaf;
					tnode.aabb[j] = node.aabb[j];
					if (node.data[j].is_leaf)
					{
						tnode.data[j].num_tri = node.data[j].num_tri;
						tnode.data[j].tri_offset = primative_start;

						PackedTreelet::Triangle* tris = (PackedTreelet::Triangle*)(&treelet.bytes[primative_start]);
						for (uint k = 0; k <= node.data[j].num_tri; ++k)
						{
							tris[k].id = node.data[j].tri_index + k;
							tris[k].tri = triangles[tris[k].id];
							primative_start += sizeof(PackedTreelet::Triangle);
						}
					}
					else
					{
						uint child_node_id = node.data[j].child_index;
						if (root_node_treelet.find(child_node_id) != root_node_treelet.end())
						{
							tnode.data[j].is_child_treelet = 1;
							tnode.data[j].child_index = root_node_treelet[child_node_id];
						}
						else
						{
							tnode.data[j].is_child_treelet = 0;
							tnode.data[j].child_index = node_map[child_node_id] = nodes_mapped++;
						}
					}
				}
			}
		}

		printf("Treelets: %zu\n", treelets.size());
		printf("PackedTreelet Fill Rate: %.1f%%\n", 100.0 * total_footprint / treelets.size() / usable_space);
	}
};
#endif

} // namespace rtm