#pragma once

#include "mesh.hpp"

namespace rtm {

//fp32 triangle block with compressed index buffer/primID
struct alignas(32) FTB
{
	const static uint BLOCK_SIZE = 64;
	const static uint MAX_PRIMS = 1;
	const static uint MAX_VRTS = 15;

	uint32_t prim_id0 : 29;
	uint8_t num_vrts : 4; //at most 15 vrtices
	uint8_t num_tris : 4; //at most 15 triangles

	const static uint DATA_SECTOR_SIZE = BLOCK_SIZE * 8 - 40;
	BitArray<DATA_SECTOR_SIZE> data; //indices then vrtices
};
static_assert(sizeof(FTB) == FTB::BLOCK_SIZE);

//near lossless 24 bit quantized triangle block with compressed vertex buffer/index buffer/primID
struct alignas(32) QTB
{
	const static uint BLOCK_SIZE = 128;
	const static uint MAX_PRIMS = 31;
	const static uint MAX_VRTS = 31;

	uint64_t prim_id0 : 29;
	uint64_t exp : 8;
	uint64_t num_vrts : 5; //at most 31 vrtices
	uint64_t num_tris : 5; //at most 31 triangles
	uint64_t bx : 5; //bits per plane
	uint64_t bz : 5;
	uint64_t by : 5;
	int16_t px;
	int16_t py;
	int16_t pz;

	const static uint DATA_SECTOR_SIZE = BLOCK_SIZE * 8 - 112;
	BitArray<DATA_SECTOR_SIZE> data; //indices then vrtices
};
static_assert(sizeof(QTB) == QTB::BLOCK_SIZE);

inline bool compress(const uint* prims, uint num_prims, const uint prim_id0, const Mesh& mesh, FTB& block)
{
	sizeof(FTB);
	if(num_prims > FTB::MAX_PRIMS) return false;

	block.prim_id0 = prim_id0;
	block.num_tris = num_prims;

	AABB aabb;
	for(uint i = 0; i < block.num_tris; ++i)
		aabb.add(mesh.get_triangle(prims[i]).aabb());

	uvec3 index_buffer[FTB::MAX_PRIMS];
	for(uint i = 0; i < block.num_tris; ++i)
		index_buffer[i] = mesh.vertex_indices[prims[i]];

	block.num_vrts = 0;
	rtm::vec3 vertex_buffer[FTB::MAX_VRTS];
	std::map<uint, uint> vertex_map;

	uint32_t xmax = 0, ymax = 0, zmax = 0;
	for(uint i = 0; i < block.num_tris; ++i)
	{
		for(uint j = 0; j < 3; ++j)
		{
			uint vertex_index = index_buffer[i][j];
			if(vertex_map.count(vertex_index) == 0)
			{
				if(block.num_vrts >= FTB::MAX_VRTS) return false;
				vertex_map[vertex_index] = block.num_vrts;
				vertex_buffer[block.num_vrts] = mesh.vertices[vertex_index];
				block.num_vrts++;
			}
			index_buffer[i][j] = vertex_map[vertex_index];
		}
	}

	uint bits_per_index = log2i(block.num_vrts) + 1;
	uint ib_size = bits_per_index * 3 * block.num_tris;
	uint vb_size = 32 * 3 * block.num_vrts;

	if(ib_size + vb_size > FTB::DATA_SECTOR_SIZE) return false;

	//encode indices
	uint block_ptr = 0;
	for(uint i = 0; i < block.num_tris; ++i)
	{
		block.data.write(block_ptr, bits_per_index, index_buffer[i][0]); block_ptr += bits_per_index;
		block.data.write(block_ptr, bits_per_index, index_buffer[i][1]); block_ptr += bits_per_index;
		block.data.write(block_ptr, bits_per_index, index_buffer[i][2]); block_ptr += bits_per_index;
	}
	//encode vertices
	for(uint i = 0; i < block.num_vrts; ++i)
	{
		block.data.write(block_ptr, 32, to_u32(vertex_buffer[i][0])); block_ptr += 32;
		block.data.write(block_ptr, 32, to_u32(vertex_buffer[i][1])); block_ptr += 32;
		block.data.write(block_ptr, 32, to_u32(vertex_buffer[i][2])); block_ptr += 32;
	}

	return true;
}

inline uint decompress(const rtm::FTB& block, uint strip_id, rtm::IntersectionTriangle* tris)
{
	uint bits_per_index = log2i(block.num_vrts) + 1;
	uint ind_size = bits_per_index * 3 * block.num_tris;

	uint block_ptr = 0;
	uvec3 ib[FTB::MAX_PRIMS];
	for(uint i = 0; i < block.num_tris; ++i)
	{
		ib[i][0] = block.data.read(block_ptr, bits_per_index); block_ptr += bits_per_index;
		ib[i][1] = block.data.read(block_ptr, bits_per_index); block_ptr += bits_per_index;
		ib[i][2] = block.data.read(block_ptr, bits_per_index); block_ptr += bits_per_index;
	}

	rtm::vec3 vb[FTB::MAX_VRTS];
	for(uint i = 0; i < block.num_vrts; ++i)
	{
		vb[i][0] = to_f32(block.data.read(block_ptr, 32)); block_ptr += 32;
		vb[i][1] = to_f32(block.data.read(block_ptr, 32)); block_ptr += 32;
		vb[i][2] = to_f32(block.data.read(block_ptr, 32)); block_ptr += 32;
	}

	for(uint i = 0; i < block.num_tris; ++i)
	{
		tris[i].tri = Triangle(vb[ib[i][0]], vb[ib[i][1]], vb[ib[i][2]]);
		tris[i].id = block.prim_id0 + i;
	}

	return block.num_tris;
}

inline bool compress(const uint* prims, uint num_prims, const uint prim_id0, const Mesh& mesh, QTB& block)
{
	if(num_prims > QTB::MAX_PRIMS) return false;

	block.prim_id0 = prim_id0;
	block.num_tris = num_prims;
	block.exp = mesh.exp;

	AABB aabb;
	for(uint i = 0; i < block.num_tris; ++i)
		aabb.add(mesh.get_triangle(prims[i]).aabb());

	block.px = f32_to_i16(aabb.min.x, block.exp, -1);
	block.py = f32_to_i16(aabb.min.y, block.exp, -1);
	block.pz = f32_to_i16(aabb.min.z, block.exp, -1);

	uvec3 index_buffer[QTB::MAX_PRIMS];
	for(uint i = 0; i < block.num_tris; ++i)
		index_buffer[i] = mesh.vertex_indices[prims[i]];

	block.num_vrts = 0;
	uvec3 vertex_buffer[QTB::MAX_VRTS];
	std::map<uint, uint> vertex_map;

	uint32_t xmax = 0, ymax = 0, zmax = 0;
	for(uint i = 0; i < block.num_tris; ++i)
	{
		for(uint j = 0; j < 3; ++j)
		{
			uint vertex_index = index_buffer[i][j];
			if(vertex_map.count(vertex_index) == 0)
			{
				if(block.num_vrts >= QTB::MAX_VRTS) return false;
				vertex_map[vertex_index] = block.num_vrts;
				vertex_buffer[block.num_vrts][0] = (int32_t)mesh.quantized_vertices[vertex_index][0] - block.px * 256;
				vertex_buffer[block.num_vrts][1] = (int32_t)mesh.quantized_vertices[vertex_index][1] - block.py * 256;
				vertex_buffer[block.num_vrts][2] = (int32_t)mesh.quantized_vertices[vertex_index][2] - block.pz * 256;
				xmax = max(xmax, vertex_buffer[block.num_vrts][0]);
				ymax = max(ymax, vertex_buffer[block.num_vrts][1]);
				zmax = max(zmax, vertex_buffer[block.num_vrts][2]);
				block.num_vrts++;
			}
			index_buffer[i][j] = vertex_map[vertex_index];
		}
	}

	block.bx = log2i(xmax) + 1;
	block.by = log2i(ymax) + 1;
	block.bz = log2i(zmax) + 1;

	uint bits_per_index = log2i(block.num_vrts) + 1;
	uint ib_size = bits_per_index * 3 * block.num_tris;
	uint vb_size = (block.bx + block.by + block.bz) * block.num_vrts;

	if(ib_size + vb_size > QTB::DATA_SECTOR_SIZE) return false;

	//encode indices
	uint block_ptr = 0;
	for(uint i = 0; i < block.num_tris; ++i)
	{
		block.data.write(block_ptr, bits_per_index, index_buffer[i][0]); block_ptr += bits_per_index;
		block.data.write(block_ptr, bits_per_index, index_buffer[i][1]); block_ptr += bits_per_index;
		block.data.write(block_ptr, bits_per_index, index_buffer[i][2]); block_ptr += bits_per_index;
	}
	//encode vertices
	for(uint i = 0; i < block.num_vrts; ++i)
	{
		block.data.write(block_ptr, block.bx, vertex_buffer[i][0]); block_ptr += block.bx;
		block.data.write(block_ptr, block.by, vertex_buffer[i][1]); block_ptr += block.by;
		block.data.write(block_ptr, block.bz, vertex_buffer[i][2]); block_ptr += block.bz;
	}

	return true;
}

inline uint decompress(const rtm::QTB& block, uint strip_id, rtm::IntersectionTriangle* tris)
{
	uint bits_per_index = log2i(block.num_vrts) + 1;
	uint ind_size = bits_per_index * 3 * block.num_tris;

	uint block_ptr = 0;
	uvec3 ib[QTB::MAX_PRIMS];
	for(uint i = 0; i < block.num_tris; ++i)
	{
		ib[i][0] = block.data.read(block_ptr, bits_per_index); block_ptr += bits_per_index;
		ib[i][1] = block.data.read(block_ptr, bits_per_index); block_ptr += bits_per_index;
		ib[i][2] = block.data.read(block_ptr, bits_per_index); block_ptr += bits_per_index;
	}

	rtm::vec3 vb[QTB::MAX_VRTS];
	for(uint i = 0; i < block.num_vrts; ++i)
	{
		vb[i][0] = i24_to_f32((int32_t)block.data.read(block_ptr, block.bx) + ((int32_t)block.px) * 256, block.exp); block_ptr += block.bx;
		vb[i][1] = i24_to_f32((int32_t)block.data.read(block_ptr, block.by) + ((int32_t)block.py) * 256, block.exp); block_ptr += block.by;
		vb[i][2] = i24_to_f32((int32_t)block.data.read(block_ptr, block.bz) + ((int32_t)block.pz) * 256, block.exp); block_ptr += block.bz;
	}

	for(uint i = 0; i < block.num_tris; ++i)
	{
		tris[i].tri = Triangle(vb[ib[i][0]], vb[ib[i][1]], vb[ib[i][2]]);
		tris[i].id = block.prim_id0 + i;
	}

	return block.num_tris;
}

}