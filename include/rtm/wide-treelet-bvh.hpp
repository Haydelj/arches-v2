#pragma once

#include "wide-bvh.hpp"
#include "mesh.hpp"

#ifndef __riscv
#include <map>
#include <set>
#include <unordered_map>
#include <stack>
#include <queue>
#endif

namespace rtm {

class WideTreeletBVH
{
public:
	const static uint WIDTH = WideBVH::WIDTH;

	struct Treelet
	{
		const static uint SIZE = (16 << 20);

		struct alignas(64) Header
		{
			uint first_child;
			uint num_children;
			uint subtree_size;
			uint depth;
			uint num_nodes;
			uint bytes;

			float median_page_sah[8];
		};

		struct alignas(64) Node
		{
			union Data
			{
				struct
				{
					uint32_t is_int : 1;
					uint32_t is_child_treelet : 1;
					uint32_t : 1;
					uint32_t child_index : 29;
				};
				struct
				{
					uint32_t : 1;
					uint32_t num_tri : 2;
					uint32_t triangle_index : 29;
				};
			};

			union ParentData
			{
				struct
				{
					uint32_t parent_treelet_index : 12;		// parent treelet index
					uint32_t parent_node_index : 20;		// parent node index in the parent treelet
				};
			};

			ParentData parent_data;
			Data data[WIDTH];
			AABB aabb[WIDTH];

			bool is_valid(uint i) const { return data[i].is_int || data[i].num_tri > 0; }
		};

		struct Triangle
		{
			rtm::Triangle tri;
			uint          id;
		};

		Header   header;
		union
		{
			uint8_t  data[SIZE - sizeof(Header)];
			Node    nodes[1];
		};

		Treelet() {}
	};

#ifndef __riscv
	std::vector<Treelet> treelets;

	WideTreeletBVH(const rtm::WideBVH& bvh, const rtm::Mesh& mesh, uint max_cut_size = 1024)
	{
		build(bvh, mesh, max_cut_size);
	}

	uint get_node_size(uint node, const rtm::WideBVH& bvh, const rtm::Mesh& mesh)
	{
		uint node_size = sizeof(Treelet::Node);
		for(uint i = 0; i < WIDTH; ++i)
			if(!bvh.nodes[node].data[i].is_int)
				node_size += sizeof(Treelet::Triangle) * bvh.nodes[node].data[i].num_prims;
		return node_size;
	}

	void build(const rtm::WideBVH& bvh, const rtm::Mesh& mesh, uint max_cut_size = 1024)
	{
		printf("Building Wide Treelet BVH\n");
		size_t usable_space = Treelet::SIZE - sizeof(Treelet::Header);

		//Phase 0 setup
		uint total_footprint = 0;
		std::vector<uint> footprint;
		std::vector<float> area;
		std::vector<float> best_cost;
		for(uint i = 0; i < bvh.nodes.size(); ++i)
		{
			footprint.push_back(get_node_size(i, bvh, mesh));
			total_footprint += footprint.back();

			AABB aabb;
			for(uint j = 0; j < WIDTH; ++j)
				if(bvh.nodes[i].is_valid(j))
					aabb.add(bvh.nodes[i].aabb[j]);

			area.push_back(aabb.surface_area());
			best_cost.push_back(INFINITY);
		}

		float epsilon = area[0] * usable_space / (10 * total_footprint);

		std::stack<uint> post_stack;
		std::stack<uint> pre_stack; pre_stack.push(0);
		while(!pre_stack.empty())
		{
			uint node = pre_stack.top();
			pre_stack.pop();

			for(uint i = 0; i < WIDTH; ++i)
				if(bvh.nodes[node].data[i].is_int)
					pre_stack.push(bvh.nodes[node].data[i].child_index);

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

			for(uint i = 0; i < WIDTH; ++i)
				if(bvh.nodes[root_node].data[i].is_int)
					subtree_footprint[root_node] += subtree_footprint[bvh.nodes[root_node].data[i].child_index];

			std::set<uint> cut{root_node};
			uint bytes_remaining = usable_space;
			best_cost[root_node] = INFINITY;
			while(cut.size() < max_cut_size)
			{
				uint best_node = ~0u;
				float best_score = -INFINITY;
				for(auto& n : cut)
				{
					if(footprint[n] <= bytes_remaining)
					{
						float gain = area[n] + epsilon;
						float price = rtm::min(subtree_footprint[n], usable_space);
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
				for(uint i = 0; i < WIDTH; ++i)
					if(bvh.nodes[best_node].data[i].is_int)
						cut.insert(bvh.nodes[best_node].data[i].child_index);

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

		std::map<uint, uint> parent_map;
		std::vector<uint> parent;

		std::queue<uint> root_node_queue;
		root_node_queue.push(0);
		while(!root_node_queue.empty())
		{
			uint root_node = root_node_queue.front();

			uint parent_treelet = ~0u;
			if(parent_map.count(root_node))
				parent_treelet = parent_map[root_node];
			else
				assert(root_node == 0);

			root_node_queue.pop();

			root_node_treelet[root_node] = treelet_assignments.size();
			treelet_assignments.push_back({});

			std::vector<uint> cut{root_node};
			uint bytes_remaining = usable_space;
			while(cut.size() < max_cut_size)
			{
				uint best_index = ~0u;
				float best_score = -INFINITY;
				for(uint i = 0; i < cut.size(); ++i)
				{
					uint n = cut[i];
					if(footprint[n] <= bytes_remaining)
					{
						float gain = area[n] + epsilon;
						float price = rtm::min(subtree_footprint[n], usable_space);
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

				uint j = 0;
				for(uint i = 0; i < WIDTH; ++i)
					if(bvh.nodes[best_node].data[i].is_int)
						cut.insert(cut.begin() + best_index + j++, bvh.nodes[best_node].data[i].child_index);

				bytes_remaining -= footprint[best_node];

				float cost = area[root_node] + epsilon;
				for(auto& n : cut)
					cost += best_cost[n];

				if(cost == best_cost[root_node]) break;
			}

			uint depth = parent_treelet == ~0u ? 0 : treelet_headers[parent_treelet].depth + 1;
			treelet_headers.emplace_back();
			treelet_headers.back().first_child = (uint)(root_node_queue.size() + treelet_assignments.size());
			treelet_headers.back().num_children = (uint)cut.size();
			treelet_headers.back().subtree_size = 1;
			treelet_headers.back().depth = depth;
			treelet_headers.back().num_nodes = treelet_assignments.back().size();
			treelet_headers.back().bytes = usable_space - bytes_remaining;

			parent.push_back(parent_treelet);

			//we use a queue so that treelets are breadth first in memory
			for(auto& n : cut)
			{
				parent_map[n] = treelet_assignments.size() - 1;
				root_node_queue.push(n);
			}
		}

		assert(parent.size() == treelet_headers.size());
		for(int i = parent.size() - 1; i >= 0; i--)
		{
			int fa = parent[i];
			if(fa != -1)
			{
				assert(fa < i);
				treelet_headers[fa].subtree_size += treelet_headers[i].subtree_size;
			}
		}



		//Phase 3 construct treelets in memeory
		treelets.resize(treelet_assignments.size());

		for(uint treelet_index = 0; treelet_index < treelets.size(); ++treelet_index)
		{
			std::vector<uint> odered_nodes(treelet_assignments[treelet_index]);
			std::sort(odered_nodes.begin(), odered_nodes.end(), [&](uint a, uint b) -> bool
			{
				AABB aabb_a, aabb_b;
				for(uint j = 0; j < WIDTH; ++j)
				{
					aabb_a.add(bvh.nodes[a].aabb[j]);
					aabb_b.add(bvh.nodes[b].aabb[j]);
				};

				return aabb_a.surface_area() > aabb_b.surface_area();
			});

			std::unordered_map<uint, uint> node_map;
			for(uint i = 0; i < odered_nodes.size(); ++i)
				node_map[odered_nodes[i]] = i;

			Treelet& treelet = treelets[treelet_index];
			treelet.header = treelet_headers[treelet_index];

			uint primatives_offset = odered_nodes.size() * (sizeof(Treelet::Node) / 4);
			for(uint i = 0; i < odered_nodes.size(); ++i)
			{
				uint node_id = odered_nodes[i];
				assert(node_map.find(node_id) != node_map.end());

				const rtm::WideBVH::Node wnode = bvh.nodes[node_id];
				Treelet::Node& tnode = treelets[treelet_index].nodes[i];

				for(uint j = 0; j < WIDTH; ++j)
				{
					tnode.aabb[j] = wnode.aabb[j];
					tnode.data[j].is_int = wnode.data[j].is_int;
					if(wnode.data[j].is_int)
					{
						uint child_node_id = wnode.data[j].child_index;
						if(root_node_treelet.find(child_node_id) != root_node_treelet.end())
						{
							tnode.data[j].is_child_treelet = 1;
							tnode.data[j].child_index = root_node_treelet[child_node_id];
							// set parent data
							Treelet::Node& child_node = treelets[tnode.data[j].child_index].nodes[0];
							child_node.parent_data.parent_treelet_index = treelet_index;
							child_node.parent_data.parent_node_index = i;
						}
						else
						{
							tnode.data[j].is_child_treelet = 0;
							tnode.data[j].child_index = node_map[child_node_id];
							// set parent data
							Treelet::Node& child_node = treelets[treelet_index].nodes[tnode.data[j].child_index];
							child_node.parent_data.parent_treelet_index = treelet_index;
							child_node.parent_data.parent_node_index = i;
						}
					}
					else
					{
						tnode.data[j].num_tri = wnode.data[j].num_prims;
						tnode.data[j].triangle_index = primatives_offset;

						Treelet::Triangle* tris = (Treelet::Triangle*)((uint32_t*)treelet.nodes + primatives_offset);
						primatives_offset += tnode.data[j].num_tri * sizeof(Treelet::Triangle) / 4;
						 
						for(uint k = 0; k < wnode.data[j].num_prims; ++k)
						{
							uint tri_id = wnode.data[j].prim_index + k;
							tris[k].tri = mesh.get_triangle(tri_id);
							tris[k].id = tri_id;
						}
					}
				}
			}

			//fill_page_median_sah(treelet);
		}

		printf("Built Wide Treelet BVH\n");
		printf("Treelets: %zu\n", treelets.size());
		printf("Treelet Size: %d\n", Treelet::SIZE);
		printf("Treelet Fill Rate: %.1f%%\n", 100.0 * total_footprint / treelets.size() / usable_space);
	}
#endif
};

} // namespace rtm