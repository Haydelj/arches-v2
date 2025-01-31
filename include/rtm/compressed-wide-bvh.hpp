#pragma once

#include "wide-bvh.hpp"

//Following wide BVH algorithm takes in a binary BVH as input and then collapses its notes into wide nodes in a SAH-optimal fashion,  collapsing can be further followed by compression 
//https://research.nvidia.com/publication/2017-07_efficient-incoherent-ray-traversal-gpus-through-compressed-wide-bvhs

namespace rtm {

class CompressedWideBVH
{
public:
	const static uint WIDTH = WideBVH::WIDTH;

	struct alignas(64) Node
	{
		struct CompressedData
		{
			uint8_t is_int    : 1;
			uint8_t num_prims : 2;
			uint8_t offset    : 5;
		};

		//Quantized aabb
		struct CompressedAABB
		{
			struct
			{
				uint8_t x;
				uint8_t y;
				uint8_t z;
			}
			min;

			struct
			{
				uint8_t x;
				uint8_t y;
				uint8_t z;
			}
			max;

			//AABB decompress(const vec3& p, const uint8_t e[3]) {}
		};

		uint64_t base_child_index    : 28; //base offset in node array
		uint64_t base_triangle_index : 28; //base offset in primitive array
		uint64_t e0 : 8;
		rtm::vec3 p;
		uint8_t e1;
		uint8_t e2;
		CompressedData cdata[WIDTH];
		CompressedAABB caabb[WIDTH];

		Node() : p(0.0f, 0.0f, 0.0f), e0 {0}, e1{ 0 }, e2{ 0 }, base_child_index(0), base_triangle_index(0)
		{
			for(int i = 0; i < WIDTH; i++)
			{
				cdata[i].is_int = 0u;
				cdata[i].num_prims = 0u;
				cdata[i].offset = 0u;
				caabb[i].min.x = 0u;
				caabb[i].min.y = 0u;
				caabb[i].min.z = 0u;
				caabb[i].max.x = 0u;
				caabb[i].max.y = 0u;
				caabb[i].max.z = 0u;
			}
		};
		~Node() = default;

		WideBVH::Node decompress() const
		{
			WideBVH::Node node;

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
					node.data[i].child_index = (uint32_t)base_child_index + cdata[i].offset;
				}
				else
				{
					node.data[i].num_prims = cdata[i].num_prims;
					node.data[i].prim_index = (uint32_t)base_triangle_index + cdata[i].offset;
				}
			}

			return node;
		}
	};

	struct TreeletTriangle
	{
		rtm::Triangle tri;
		uint id;
	};

#ifndef __riscv
	std::vector<Node> nodes;
	std::vector<uint8_t> treelets;

public:
	/// <summary>
	/// Creates compressed wide bvh with branching factor equal to WIDE_BVH_SIZE from a wide bvh
	///  Note: bvh2 should have max 1 prim per leaf node in order for this to work
	/// </summary>
	/// <param name="_wbvh"></param>
	CompressedWideBVH(const rtm::WideBVH& wbvh)
	{
		printf("Building Compressed Wide BVH\n");

		assert(wbvh.nodes.size() != 0);

		nodes.clear();
		for(const WideBVH::Node& wnode : wbvh.nodes)
		{
			AABB wnode_aabb;
			uint base_index_child = ~0u;
			uint base_triangle_index = ~0u;
			for(uint i = 0; i < WIDTH; ++i)
			{
				if(!wnode.is_valid(i)) continue;

				wnode_aabb.add(wnode.aabb[i]);
				if(wnode.data[i].is_int)
				{
					base_index_child = min(base_index_child, wnode.data[i].child_index);
				}
				else
				{
					base_triangle_index = min(base_triangle_index, wnode.data[i].prim_index);
				}
			}

			nodes.emplace_back();
			CompressedWideBVH::Node& cwnode = nodes.back();

			cwnode.p = wnode_aabb.min;
			constexpr int Nq = 8;
			constexpr float denom = 1.0f / float((1 << Nq) - 1);

			const AABB& aabb = wnode_aabb;
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

			cwnode.base_child_index = base_index_child;
			cwnode.base_triangle_index = base_triangle_index;

			uint32_t num_triangles = 0;
			uint32_t num_children = 0;

			//Loop over all the uncompressed child nodes
			for(int i = 0; i < WIDTH; i++)
			{
				if(!wnode.is_valid(i))
				{
					cwnode.cdata[i].is_int = 0;
					cwnode.cdata[i].num_prims = 0;
					continue;
				}

				cwnode.caabb[i].min.x = uint8_t(floorf((wnode.aabb[i].min.x - cwnode.p.x) * one_over_e.x));
				cwnode.caabb[i].min.y = uint8_t(floorf((wnode.aabb[i].min.y - cwnode.p.y) * one_over_e.y));
				cwnode.caabb[i].min.z = uint8_t(floorf((wnode.aabb[i].min.z - cwnode.p.z) * one_over_e.z));
					   
				cwnode.caabb[i].max.x = uint8_t(ceilf((wnode.aabb[i].max.x - cwnode.p.x) * one_over_e.x));
				cwnode.caabb[i].max.y = uint8_t(ceilf((wnode.aabb[i].max.y - cwnode.p.y) * one_over_e.y));
				cwnode.caabb[i].max.z = uint8_t(ceilf((wnode.aabb[i].max.z - cwnode.p.z) * one_over_e.z));

				cwnode.cdata[i].is_int = wnode.data[i].is_int;
				cwnode.cdata[i].num_prims = wnode.data[i].num_prims;
				
				if(wnode.data[i].is_int)
				{
					cwnode.cdata[i].offset = num_children++;
				}
				else
				{
					cwnode.cdata[i].offset = wnode.data[i].prim_index - base_triangle_index;
				}
			}
		}

		printf("Built Compressed Wide BVH\n");
	}

	uint get_node_size(uint node, const rtm::Mesh& mesh)
	{
		uint node_size = sizeof(Node);
		for(uint i = 0; i < WIDTH; ++i)
			if(!nodes[node].cdata[i].is_int)
				node_size += sizeof(TreeletTriangle) * nodes[node].cdata[i].num_prims;
		return node_size;
	}

	/*
	void treeletize(const rtm::Mesh& mesh, uint treelet_size = 1 << 20, uint max_cut_size = 1024)
	{
		printf("Building Compressed Wide Treelet BVH\n");
		size_t usable_space = treelet_size;

		//Phase 0 setup
		uint total_footprint = 0;
		std::vector<uint> footprint;
		std::vector<float> area;
		std::vector<float> best_cost;
		for(uint i = 0; i < nodes.size(); ++i)
		{
			footprint.push_back(get_node_size(i, mesh));
			total_footprint += footprint.back();

			AABB aabb;
			for(uint j = 0; j < WIDTH; ++j)
				if(nodes[i].decompress().is_valid(j))
					aabb.add(nodes[i].decompress().aabb[j]);

			area.push_back(aabb.surface_area());
			best_cost.push_back(INFINITY);
		}

		float epsilon = area[0] * usable_space / (10 * total_footprint);

		std::deque<uint> post_stack;
		std::stack<uint> pre_stack; pre_stack.push(0);
		while(!pre_stack.empty())
		{
			uint node = pre_stack.top();
			pre_stack.pop();

			for(uint i = 0; i < WIDTH; ++i)
				if(nodes[node].cdata[i].is_int)
					pre_stack.push(nodes[node].decompress().data[i].child_index);

			post_stack.push_front(node);
		}



		//Phase 1 reverse depth first search using dynamic programing to determine treelet costs
		std::vector<uint> subtree_footprint;
		subtree_footprint.resize(nodes.size(), 0);
		for(auto& root_node : post_stack)
		{
			subtree_footprint[root_node] = footprint[root_node];

			for(uint i = 0; i < WIDTH; ++i)
				if(nodes[root_node].cdata[i].is_int)
					subtree_footprint[root_node] += subtree_footprint[nodes[root_node].decompress().data[i].child_index];

			std::set<std::set<uint>> cut{{root_node}};
			uint bytes_remaining = usable_space;
			best_cost[root_node] = INFINITY;
			while(cut.size() < max_cut_size)
			{
				std::set<uint> best_node_set = {};
				float best_score = -INFINITY;
				for(auto& node_set : cut)
				{
					uint range_footprint = 0;
					for(auto& a : node_set)
						range_footprint += footprint[a];

					if(range_footprint <= bytes_remaining)
					{
						float gain = area[*node_set.begin()] + epsilon;
						float price = rtm::min(subtree_footprint[*node_set.begin()], usable_space);
						float score = gain / price;
						if(score > best_score)
						{
							best_node_set = node_set;
							best_score = score;
						}
					}
				}
				if(best_node_set.empty()) break;

				cut.erase(best_node_set);

				//insert children into cut
				for(auto& node : best_node_set)
				{
					std::set<uint> child_set;
					for(uint i = 0; i < WIDTH; ++i)
						if(nodes[node].cdata[i].is_int)
							child_set.insert(nodes[node].decompress().data[i].child_index);

					if(child_set.size() > 0)
						cut.insert(child_set);

					bytes_remaining -= footprint[node];
				}

				float cost = area[root_node] + epsilon;
				for(auto& n : cut)
					cost += best_cost[*n.begin()];
				best_cost[root_node] = rtm::min(best_cost[root_node], cost);
			}
		}



		//Phase 2 treelet assignment
		std::vector<std::vector<uint>> nodes_in_treelet({});
		std::vector<uint> new_node_id(nodes.size(), ~0u);

		std::queue<std::set<uint>> root_set_queue;
		root_set_queue.push({0});

		while(!root_set_queue.empty())
		{
			std::set<uint> root_set = root_set_queue.front();
			root_set_queue.pop();

			nodes_in_treelet.push_back({});

			std::set<std::set<uint>> cut{{root_node}};
			uint bytes_remaining = usable_space;
			best_cost[root_node] = INFINITY;
			while(cut.size() < max_cut_size)
			{
				std::set<uint> best_node_set = {};
				float best_score = -INFINITY;
				for(auto& node_set : cut)
				{
					uint range_footprint = 0;
					for(auto& a : node_set)
						range_footprint += footprint[a];

					if(range_footprint <= bytes_remaining)
					{
						float gain = area[*node_set.begin()] + epsilon;
						float price = rtm::min(subtree_footprint[*node_set.begin()], usable_space);
						float score = gain / price;
						if(score > best_score)
						{
							best_node_set = node_set;
							best_score = score;
						}
					}
				}
				if(best_node_set.empty()) break;

				for(auto& node : best_node_set)
				{
					new_node_id[node] = nodes_in_treelet.size() * treelet_size + nodes_in_treelet.back().size();
					nodes_in_treelet.back().push_back(node);
				}

				cut.erase(best_node_set);

				//insert children into cut
				for(auto& node : best_node_set)
				{
					std::set<uint> child_set;
					for(uint i = 0; i < WIDTH; ++i)
						if(nodes[node].cdata[i].is_int)
							child_set.insert(nodes[node].decompress().data[i].child_index);

					if(child_set.size() > 0)
						cut.insert(child_set);

					bytes_remaining -= footprint[node];
				}

				float cost = area[root_node] + epsilon;
				for(auto& n : cut)
					cost += best_cost[*n.begin()];
				best_cost[root_node] = rtm::min(best_cost[root_node], cost);
			}

			for(auto& set : cut)
				for(auto& node : set)

		}



		//Phase 3 construct treelets in memeory
		treelets.resize(nodes_in_treelet.size() * treelet_size);

		for(uint treelet_index = 0; treelet_index < nodes_in_treelet.size(); ++treelet_index)
		{
			uint base_triangle_index = (treelet_index * treelet_size + nodes_in_treelet[treelet_index].size() * sizeof(Node)) / 4;
			for(uint i = 0; i < nodes_in_treelet[treelet_index].size(); ++i)
			{
				uint node_id = nodes_in_treelet[treelet_index][i];
				uint tnode_id = new_node_id[node_id];
				assert(tnode_id != ~0);

				const Node& cwnode = nodes[node_id];
				Node& tnode = *(Node*)&treelets[sizeof(Node) * tnode_id];

				tnode.e0 = cwnode.e0;
				tnode.e1 = cwnode.e1;
				tnode.e2 = cwnode.e2;
				tnode.p = cwnode.p;

				uint base_child_index = ~0u;
				for(uint j = 0; j < WIDTH; ++j)
				{
					if(cwnode.cdata[j].is_int)
					{
						uint child_node_id = new_node_id[cwnode.decompress().data[j].child_index];
						base_child_index = min(base_child_index, child_node_id);
					}
				}

				tnode.base_child_index = base_child_index;
				tnode.base_triangle_index = base_triangle_index;

				uint triangle_offset = 0;
				for(uint j = 0; j < WIDTH; ++j)
				{
					tnode.caabb[j] = cwnode.caabb[j];
					tnode.cdata[j].is_int = cwnode.cdata[j].is_int;
					if(cwnode.cdata[j].is_int)
					{
						uint child_node_id = new_node_id[cwnode.decompress().data[j].child_index];
						uint offset = child_node_id - tnode.base_child_index;
						assert(offset < 32);
						tnode.cdata[j].offset = offset;
					}
					else
					{
						assert(triangle_offset < 32);
						tnode.cdata[j].num_prims = cwnode.cdata[j].num_prims;
						tnode.cdata[j].offset = triangle_offset;

						TreeletTriangle* tris = (TreeletTriangle*)((uint32_t*)treelets.data() + base_triangle_index) + tnode.cdata[j].offset;
						triangle_offset += cwnode.cdata[j].num_prims;

						for(uint k = 0; k < cwnode.cdata[j].num_prims; ++k)
						{
							uint tri_id = cwnode.decompress().data[j].prim_index + k;
							tris[k].tri = mesh.get_triangle(tri_id);
							tris[k].id = tri_id;
						}

					}
				}
				base_triangle_index += triangle_offset * sizeof(TreeletTriangle) / 4;
			}

			//fill_page_median_sah(treelet);
		}

		printf("Built Compressed Wide Treelet BVH\n");
		printf("Treelets: %zu\n", treelets.size());
		printf("Treelet Size: %d\n", treelet_size);
		printf("Treelet Fill Rate: %.1f%%\n", 100.0 * total_footprint / treelets.size() / usable_space);
	}
	*/

	~CompressedWideBVH() = default;
#endif
};

}