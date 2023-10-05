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
		build_treelet_breadth_first(bvh, mesh);
		printf("Treelet BVH Built\n");
	}

	uint get_node_size(uint node, const rtm::BVH& bvh, const rtm::Mesh& mesh)
	{
		uint node_size = sizeof(TreeletNode);
		if(bvh.nodes[node].data.is_leaf)
			node_size += sizeof(TreeletTriangle) * (bvh.nodes[node].data.lst_chld_ofst + 1);

		return node_size;
	}

	void build_treelet_breadth_first(const rtm::BVH& bvh, const rtm::Mesh& mesh)
	{
		std::vector<std::vector<uint>> treelet_assignments;

		std::queue<uint> root_node_queue;
		root_node_queue.push(bvh.nodes[0].data.fst_chld_ind);
		
		size_t bytes = 0;

		while(!root_node_queue.empty())
		{
			treelet_assignments.push_back({});

			std::queue<uint> node_queue;
			node_queue.push(root_node_queue.front()); root_node_queue.pop();

			uint bytes_left = sizeof(Treelet);
			while(!node_queue.empty())
			{
				uint node = node_queue.front(); node_queue.pop();
				uint node_size = get_node_size(node + 0, bvh, mesh) + get_node_size(node + 1, bvh, mesh);

				if(node_size < bytes_left)
				{
					bytes_left -= node_size;

					treelet_assignments.back().push_back(node + 0);
					treelet_assignments.back().push_back(node + 1);

					if(!bvh.nodes[node + 0].data.is_leaf) node_queue.push(bvh.nodes[node + 0].data.fst_chld_ind);
					if(!bvh.nodes[node + 1].data.is_leaf) node_queue.push(bvh.nodes[node + 1].data.fst_chld_ind);
				}
				else root_node_queue.push(node);
			}
			bytes += sizeof(Treelet) - bytes_left;
		}

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
		printf("Bytes/Treelet: %zu\n", bytes / treelets.size());
	}

};
#endif
