#pragma once

#include "wide-treelet-bvh.hpp"
#include "compressed-wide-bvh.hpp"
#include "mesh.hpp"

#ifndef __riscv
#include <map>
#include <set>
#include <unordered_map>
#include <stack>
#include <queue>
#endif

namespace rtm {

class CompressedWideTreeletBVH
{
public:
	const static uint WIDTH = CompressedWideBVH::WIDTH;

	struct Treelet
	{
		const static uint SIZE = WideTreeletBVH::Treelet::SIZE;

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
			union CompressedData
			{
				struct
				{
					uint8_t is_int : 1;
					uint8_t is_child_treelet : 1;
					uint8_t : 6;
				};
				struct
				{
					uint8_t : 1;
					uint8_t num_tris : 2;
					uint8_t offset : 5;
				};
			};

			uint64_t base_child_index : 20;	//base offset in streamlined child node array
			uint64_t base_triangle_index : 24;	//base offset in streamlined primitive array
			uint64_t base_treelet_index : 12;	//base offset in streamlined primitive array
			uint64_t e0 : 8;
			uint32_t parent_treelet_index : 12;
			uint32_t parent_node_index : 20;
			vec3 p;
			uint8_t e1;
			uint8_t e2;

			CompressedData cdata[WIDTH];
			CompressedWideBVH::Node::CompressedAABB caabb[WIDTH];

			WideTreeletBVH::Treelet::Node decompress() const
			{
				WideTreeletBVH::Treelet::Node node;

				uint32_t e0u32 = uint32_t(e0) << 23;
				uint32_t e1u32 = uint32_t(e1) << 23;
				uint32_t e2u32 = uint32_t(e2) << 23;

				float e0f = *reinterpret_cast<float*>(&e0u32);
				float e1f = *reinterpret_cast<float*>(&e1u32);
				float e2f = *reinterpret_cast<float*>(&e2u32);

				for(int i = 0; i < WIDTH; i++)
				{
					node.aabb[i].min.x = p.x + e0f * float(caabb[i].min.x);
					node.aabb[i].min.y = p.y + e1f * float(caabb[i].min.y);
					node.aabb[i].min.z = p.z + e2f * float(caabb[i].min.z);
					node.aabb[i].max.x = p.x + e0f * float(caabb[i].max.x);
					node.aabb[i].max.y = p.y + e1f * float(caabb[i].max.y);
					node.aabb[i].max.z = p.z + e2f * float(caabb[i].max.z);

					node.data[i].is_int = cdata[i].is_int;
					if(cdata[i].is_int)
					{
						node.data[i].is_child_treelet = cdata[i].is_child_treelet;
						if(cdata[i].is_child_treelet)
						{
							node.data[i].child_index = (uint32_t)base_treelet_index + cdata[i].offset;
						}
						else
						{
							node.data[i].child_index = (uint32_t)base_child_index + cdata[i].offset;
						}
					}
					else
					{
						node.data[i].num_tri = cdata[i].num_tris;
						node.data[i].triangle_index = (uint32_t)base_triangle_index + cdata[i].offset * sizeof(Triangle) / sizeof(uint32_t);
					}
				}

				return node;
			}
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
			Node     nodes[1];
		};

		Treelet() {}

	};

#ifndef __riscv

public:
	std::vector<CompressedWideTreeletBVH::Treelet> treelets;

	CompressedWideTreeletBVH() {}
	CompressedWideTreeletBVH(const rtm::CompressedWideBVH& bvh, const rtm::Mesh& mesh)
	{
		build(bvh, mesh);
	}

	uint get_node_size(uint node, const rtm::CompressedWideBVH& bvh, const rtm::Mesh& mesh)
	{
		uint node_size = sizeof(CompressedWideTreeletBVH::Treelet::Node);
		for(uint i = 0; i < WIDTH; ++i)
			if(!bvh.nodes[node].cdata[i].is_int)
				node_size += sizeof(CompressedWideTreeletBVH::Treelet::Triangle) * bvh.nodes[node].cdata[i].num_prims;
		return node_size;
	}

	void build(const rtm::CompressedWideBVH& bvh, const rtm::Mesh& mesh, uint max_cut_size = 1024)
	{
		printf("Building Compressed Wide Treelet BVH\n");
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
			for(uint j = 0; j <WIDTH; ++j)
				if(bvh.nodes[i].decompress().is_valid(j))
					aabb.add(bvh.nodes[i].decompress().aabb[j]);

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
				if(bvh.nodes[node].cdata[i].is_int)
					pre_stack.push(bvh.nodes[node].decompress().data[i].child_index);

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
				if(bvh.nodes[root_node].cdata[i].is_int)
					subtree_footprint[root_node] += subtree_footprint[bvh.nodes[root_node].decompress().data[i].child_index];

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
					if(bvh.nodes[best_node].decompress().data[i].is_int)
						cut.insert(bvh.nodes[best_node].decompress().data[i].child_index);

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
					if(bvh.nodes[best_node].cdata[i].is_int)
						cut.insert(cut.begin() + best_index + j++, bvh.nodes[best_node].decompress().data[i].child_index);

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
			std::sort(odered_nodes.begin() + 1, odered_nodes.end(), [&](uint a, uint b) -> bool
			{
				return a < b;
			});

			std::unordered_map<uint, uint> node_map;
			for(uint i = 0; i < odered_nodes.size(); ++i)
				node_map[odered_nodes[i]] = i;

			CompressedWideTreeletBVH::Treelet& treelet = treelets[treelet_index];
			treelet.header = treelet_headers[treelet_index];

			uint base_triangle_index = odered_nodes.size() * (sizeof(CompressedWideTreeletBVH::Treelet::Node&) / 4);
			for(uint i = 0; i < odered_nodes.size(); ++i)
			{
				uint node_id = odered_nodes[i];
				assert(node_map.find(node_id) != node_map.end());

				const rtm::CompressedWideBVH::Node& cwnode = bvh.nodes[node_id];
				CompressedWideTreeletBVH::Treelet::Node& tnode = treelets[treelet_index].nodes[i];

				tnode.e0 = cwnode.e0;
				tnode.e1 = cwnode.e1;
				tnode.e2 = cwnode.e2;
				tnode.p = cwnode.p;

				uint base_child_index = ~0u;
				uint base_treelet_index = ~0u;
				for(uint j = 0; j < WIDTH; ++j)
				{
					if(cwnode.cdata[j].is_int)
					{
						uint child_node_id = cwnode.decompress().data[j].child_index;
						if(root_node_treelet.find(child_node_id) != root_node_treelet.end())
						{
							base_treelet_index = min(base_treelet_index, root_node_treelet[child_node_id]);
						}
						else
						{
							base_child_index = min(base_child_index, node_map[child_node_id]);
						}
					}
				}

				tnode.base_child_index = base_child_index;
				tnode.base_treelet_index = base_treelet_index;
				tnode.base_triangle_index = base_triangle_index;

				uint triangle_offset = 0;
				for(uint j = 0; j < WIDTH; ++j)
				{
					tnode.caabb[j] = cwnode.caabb[j];
					tnode.cdata[j].is_int = cwnode.cdata[j].is_int;
					if(cwnode.cdata[j].is_int)
					{
						uint child_node_id = cwnode.decompress().data[j].child_index;
						if(root_node_treelet.find(child_node_id) != root_node_treelet.end())
						{
							tnode.cdata[j].is_child_treelet = 1;
							uint offset = root_node_treelet[child_node_id] - tnode.base_treelet_index;
							assert(offset < 32);
							tnode.cdata[j].offset = offset;
						}
						else
						{
							tnode.cdata[j].is_child_treelet = 0;
							uint offset = node_map[child_node_id] - tnode.base_child_index;;
							assert(offset < 32);
							tnode.cdata[j].offset = offset;
						}
					}
					else
					{
						assert(triangle_offset < 32);
						tnode.cdata[j].num_tris = cwnode.cdata[j].num_prims;
						tnode.cdata[j].offset = triangle_offset;

						CompressedWideTreeletBVH::Treelet::Triangle* tris = (CompressedWideTreeletBVH::Treelet::Triangle*)((uint32_t*)treelet.nodes + base_triangle_index) + tnode.cdata[j].offset;
						triangle_offset += cwnode.cdata[j].num_prims;
						
						for(uint k = 0; k < cwnode.cdata[j].num_prims; ++k)
						{
							uint tri_id = cwnode.decompress().data[j].prim_index + k;
							tris[k].tri = mesh.get_triangle(tri_id);
							tris[k].id = tri_id;
						}

					}
				}
				base_triangle_index += triangle_offset * sizeof(CompressedWideTreeletBVH::Treelet::Triangle) / 4;
			}

			//fill_page_median_sah(treelet);
		}

		printf("Built Compressed Wide Treelet BVH\n");
		printf("Treelets: %zu\n", treelets.size());
		printf("Treelet Size: %d\n", CompressedWideTreeletBVH::Treelet::SIZE);
		printf("Treelet Fill Rate: %.1f%%\n", 100.0 * total_footprint / treelets.size() / usable_space);
	}
#endif

};


} // namespace rtm