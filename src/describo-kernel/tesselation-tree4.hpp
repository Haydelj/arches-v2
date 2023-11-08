#pragma once
#include "stdafx.hpp"

#include "mesh.hpp"
#include "tesselation-tree.hpp"

class TesselationTree4
{
public:
	struct alignas(16) CompressedNode
	{
		AABB16 aabb;
		uint32_t dbs;
	};

	struct alignas(64) CompressedNode4
	{
		CompressedNode nodes[4];
	};

	struct alignas(64) Header
	{
		CompressedNode node;

		float max_db;
		uint root_node_offset;
		uint root_tri_offset;
		struct
		{
			uint id_offset : 28;
			uint last_lod : 4;
		};

		CompactTri tri;
	};


#ifdef ARCH_X86
	std::vector<Header> headers;
	std::vector<CompactTri> triangles;
	std::vector<CompressedNode4> nodes;
	std::vector<rtm::uvec3> vertex_indices;
	std::vector<float> sah_costs;

	TesselationTree4(std::string file_path)
	{
		printf("Loading: %s\r", file_path.c_str());

		std::ifstream stream(file_path, std::ios::binary);
		if(!stream.is_open()) return;

		_deserialize_vector(headers, stream);
		_deserialize_vector(nodes, stream);
		_deserialize_vector(triangles, stream);
		_deserialize_vector(vertex_indices, stream);
		_deserialize_vector(sah_costs, stream);

		stream.close();

		printf("Loaded: %s \n", file_path.c_str());
	};

	uint size() { return (uint)headers.size(); };
	void reorder(std::vector<BuildObject>& ordered_build_objects)
	{
		std::vector<Header> temp_headers(headers);
		std::vector<glm::uvec3> temp_vertex_indices(vertex_indices);
		for(uint i = 0; i < ordered_build_objects.size(); ++i)
		{
			headers[i] = temp_headers[ordered_build_objects[i].index];
			vertex_indices[i] = temp_vertex_indices[ordered_build_objects[i].index];
		}
	};

	BuildObject get_build_object(uint index)
	{
		BuildObject object;
		object.aabb = decompress(headers[index].node.aabb);
		object.cost = sah_costs[index];
		object.index = index;
		return object;
	};
#endif
};

struct TesselationTree4Pointers
{
	TesselationTree4::Header* headers;
	TesselationTree4::CompressedNode4* nodes;
	CompactTri* triangles;
	rtm::uvec3* vertex_indices;
};

struct TesselationTree4SecondaryRayData
{
	const TesselationTree4Pointers& tes_tree;
	const Ray& last_ray;
	uint last_patch_index;
	TesselationTree4SecondaryRayData(const TesselationTree4Pointers& tes_tree, uint last_patch_index, const Ray& last_ray) : tes_tree(tes_tree), last_ray(last_ray), last_patch_index(last_patch_index) {}
};