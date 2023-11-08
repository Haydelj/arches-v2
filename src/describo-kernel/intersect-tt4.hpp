#pragma once
#include "stdafx.hpp"

#include "intersect-tt.hpp"
#include "tesselation-tree4.hpp"

inline static bool intersect( const uint32_t patch_index, const TesselationTree4Pointers& tes_tree, const Ray& ray, Hit& hit)
{
	struct NodeStackEntry
	{
		float t;
		struct
		{
			uint32_t node_index : 26;
			uint32_t lod : 4;
			uint32_t tri_type : 2;
		};
		uint32_t dbs;
	}
	node_stack[3 * 8];

	struct TriangleStackEntry
	{
		Triangle tri;//TODO maybe remove?
		Triangle new_center_tri;
		bool child_transformed;
	}
	tri_stack[8];

	TesselationTree4::Header header;
	move_to_stack(header, tes_tree.headers[patch_index]);

	glm::vec3 inv_d = glm::vec3(1.0f) / ray.d;
	float max_db_over_max_error = header.max_db * ray.rcp_max_error;
	const TesselationTree4::CompressedNode4* nodes = &tes_tree.nodes[header.root_node_offset];
	const CompactTri* triangles = &tes_tree.triangles[header.root_tri_offset];

	uint32_t node_stack_size = 1u;
	node_stack[0].t = intersect(decompress(header.node.aabb), ray, inv_d);
	node_stack[0].node_index = 0;
	node_stack[0].lod = 0;
	node_stack[0].tri_type = 0;
	node_stack[0].dbs = header.node.dbs;

	uint32_t hit_lod = ~0u;
	do
	{
		const NodeStackEntry current_node_entry = node_stack[--node_stack_size];
		if(current_node_entry.t >= hit.t) continue;

		uint32_t dbs = current_node_entry.dbs;
		uint32_t node_index = current_node_entry.node_index;
		uint32_t lod = current_node_entry.lod;
		uint32_t tri_type = current_node_entry.tri_type;

		if(lod == 0) tri_stack[lod].tri = decompact(header.tri);
		else         tri_stack[lod].tri = reconstruct_triangle(tri_stack[lod - 1u].tri, tri_stack[lod - 1u].new_center_tri, tri_type);

		//determine if triangle needs subdivision. Probably should use hardware
		glm::vec3 edge_states = evalute_deformation_bounds(dbs, max_db_over_max_error, tri_stack[lod].tri, ray);

		//if acceptable intersect otherwise traverse into the next LOD
		if(edge_states[0] <= 0.0f && edge_states[1] <= 0.0f && edge_states[2] <= 0.0f)
		{
			if(intersect(tri_stack[lod].tri, ray, hit))
			{
				hit.id = node_index;
				hit_lod = lod;
			}
			continue;
		}

		//load the center triangle in the 4 triangles that subdivide the current triangle. This can be used along with the parent to recontruct the other 3 children effectivley reducing memory trafic (I think).		
		uint32_t absoulte_node_index = lod_node_offset[lod] + node_index;
		CompactTri new_tri; 
		move_to_stack(new_tri, triangles[absoulte_node_index]);
		tri_stack[lod].new_center_tri = get_transformed_triangle(tri_stack[lod].tri, decompact(new_tri), edge_states);

		//if the next LOD is the leaf intersect othewise push the child nodes onto the stack
		uint32_t next_level = lod + 1;
		uint32_t first_child_index = node_index << 2; //compute first child index
		
		if(next_level == header.last_lod) //leaf node
		{
			for(uint32_t i = 0; i < 4; ++i)
			{
				if(intersect(reconstruct_triangle(tri_stack[lod].tri, tri_stack[lod].new_center_tri, i), ray, hit))
				{
					hit.id = first_child_index + i;
					hit_lod = next_level;
				}
			}
		}
		else //interior node
		{
			tri_stack[lod].child_transformed = (edge_states[0] < 1.0f) || (edge_states[1] < 1.0f) || (edge_states[2] < 1.0f) || ((lod != 0) && tri_stack[lod - 1].child_transformed);

			uint32_t temp_node_stack_size = node_stack_size;
			TesselationTree4::CompressedNode4 node4;
			move_to_stack(node4, nodes[absoulte_node_index]);
			
			for(uint32_t i = 0; i < 4; ++i)
			{
				AABB aabb = decompress(node4.nodes[i].aabb);
				if(tri_stack[lod].child_transformed)
				{
					Triangle tri = reconstruct_triangle(tri_stack[lod].tri, tri_stack[lod].new_center_tri, i);
					aabb.add(tri.aabb());
				}

				NodeStackEntry entry;
				entry.t = intersect(aabb, ray, inv_d);
				entry.node_index = first_child_index + i;
				entry.lod = next_level;
				entry.tri_type = i;
				entry.dbs = node4.nodes[i].dbs;
				if(entry.t < hit.t) insert(entry, node_stack, node_stack_size++, temp_node_stack_size);
			}
		}
	} while(node_stack_size);

	//if we found a hit convert barycentric coords to max lod
	if(hit_lod != ~0u)
	{
	#if 0
		for(uint32_t i = hit_lod; i < lli; ++i)
		{
			uint32_t tri_type;
			hit.bc *= glm::vec2(2.0f);

			if(hit.bc.x > 1.0f)
			{
				tri_type = 0;
				hit.bc -= glm::vec2(1.0f, 0.0f);
			}
			else if(hit.bc.y > 1.0f)
			{
				tri_type = 1;
				hit.bc -= glm::vec2(0.0f, 1.0f);
			}
			else if(hit.bc.x + hit.bc.y < 1.0f)
			{
				tri_type = 2;
			}
			else
			{
				tri_type = 3;
				hit.bc = glm::vec2(1.0f, 1.0f) - hit.bc;
			}

			hit.id <<= 2;
			hit.id |= tri_type;
		}
	#endif
		hit.id += header.id_offset;
	}

	return hit_lod != ~0u;
}



///////////////////////////////////secondary\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

inline bool intersect(const uint32_t patch_index, const TesselationTree4SecondaryRayData& data, const Ray& ray, Hit& hit)
{
	struct NodeStackEntry
	{
		float t;
		struct
		{
			uint32_t node_index : 26;
			uint32_t lod : 4;
			uint32_t tri_type : 2;
		};
		uint32_t dbs;
	} node_stack[3 * 8];


	struct TriangleStackEntry
	{
		Triangle tri;//TODO maybe remove?
		Triangle new_center_tri;
		uint8_t shared_edge;
		bool child_transformed;
	}
	tri_stack[8];

	TesselationTree4::Header header;
	move_to_stack(header, data.tes_tree.headers[patch_index]);

	glm::vec3 inv_d = glm::vec3(1.0f) / ray.d;
	float max_db_over_max_error = header.max_db * ray.rcp_max_error;
	const TesselationTree4::CompressedNode4* nodes = &data.tes_tree.nodes[header.root_node_offset];
	const CompactTri* triangles = &data.tes_tree.triangles[header.root_tri_offset];

	uint32_t node_stack_size = 1u;
	node_stack[0].t = intersect(decompress(header.node.aabb), ray, inv_d);
	node_stack[0].dbs = header.node.dbs;
	node_stack[0].node_index = 0;
	node_stack[0].lod = 0;
	node_stack[0].tri_type = 0;

	glm::uvec3 previous_patch_indices(0);
	if(data.last_patch_index != ~0) previous_patch_indices  = data.tes_tree.vertex_indices[data.last_patch_index];

	uint32_t hit_lod = ~0u;
	do
	{
		const NodeStackEntry current_node_entry = node_stack[--node_stack_size];
		if(current_node_entry.t >= hit.t) continue;

		uint32_t dbs = current_node_entry.dbs;
		uint32_t node_index = current_node_entry.node_index;
		uint32_t lod = current_node_entry.lod;
		uint32_t tri_type = current_node_entry.tri_type;

		if(lod == 0)
		{
			tri_stack[lod].tri = decompact(header.tri);
			if(data.last_patch_index != ~0) tri_stack[0].shared_edge = find_shared_edge(data.tes_tree.vertex_indices[patch_index], previous_patch_indices);
			else                            tri_stack[0].shared_edge = 3;
		}
		else
		{
			tri_stack[lod].tri = reconstruct_triangle(tri_stack[lod - 1u].tri, tri_stack[lod - 1u].new_center_tri, tri_type);
			tri_stack[lod].shared_edge = propagate_shared_edge(tri_stack[lod - 1u].shared_edge, tri_type);
		}

		//determine if triangle needs subdivision. Probably should use hardware
		bool edge_mask[3] = {patch_index == data.last_patch_index || tri_stack[lod].shared_edge == 0, patch_index == data.last_patch_index || tri_stack[lod].shared_edge == 1, patch_index == data.last_patch_index || tri_stack[lod].shared_edge == 2};
		glm::vec3 edge_states = evalute_deformation_bounds(dbs, max_db_over_max_error, tri_stack[lod].tri, data.last_ray, ray, edge_mask);

		//if acceptable intersect otherwise traverse into the next LOD
		if(edge_states[0] <= 0.0f && edge_states[1] <= 0.0f && edge_states[2] <= 0.0f)
		{
			if(intersect(tri_stack[lod].tri, ray, hit))
			{
				hit.id = node_index;
				hit.patch_index = patch_index;
				hit_lod = lod;
			}
			continue;
		}

		//load the center triangle in the 4 triangles that subdivide the current triangle. This can be used along with the parent to recontruct the other 3 children effectivley reducing memory trafic (I think).		
		uint32_t absoulte_node_index = lod_node_offset[lod] + node_index;
		CompactTri new_tri;
		move_to_stack(new_tri, triangles[absoulte_node_index]);
		tri_stack[lod].new_center_tri = get_transformed_triangle(tri_stack[lod].tri, decompact(new_tri), edge_states);

		//if the next LOD is the leaf intersect othewise push the child nodes onto the stack
		uint32_t next_level = lod + 1;
		uint32_t first_child_index = node_index << 2; //compute first child idnex
		
		if(next_level == header.last_lod) //leaf node
		{
			for(uint32_t i = 0; i < 4; ++i)
			{
				if(intersect(reconstruct_triangle(tri_stack[lod].tri, tri_stack[lod].new_center_tri, i), ray, hit))
				{
					hit.id = first_child_index + i;
					hit.patch_index = patch_index;
					hit_lod = next_level;
				}
			}
		}
		else //interior node
		{
			tri_stack[lod].child_transformed = (edge_states[0] < 1.0f) || (edge_states[1] < 1.0f) || (edge_states[2] < 1.0f) || ((lod != 0) && tri_stack[lod - 1].child_transformed);

			uint32_t temp_node_stack_size = node_stack_size;
			TesselationTree4::CompressedNode4 node4;
			move_to_stack(node4, nodes[absoulte_node_index]);
			
			for(uint32_t i = 0; i < 4; ++i)
			{
				AABB aabb = decompress(node4.nodes[i].aabb);
				if(tri_stack[lod].child_transformed)
				{
					Triangle tri = reconstruct_triangle(tri_stack[lod].tri, tri_stack[lod].new_center_tri, i);
					aabb.add(tri.aabb());
				}

				NodeStackEntry entry;
				entry.t = intersect(aabb, ray, inv_d);
				entry.node_index = first_child_index + i;
				entry.lod = next_level;
				entry.tri_type = i;
				entry.dbs = node4.nodes[i].dbs;
				if(entry.t < hit.t) insert(entry, node_stack, node_stack_size++, temp_node_stack_size);
			}
		}
	} while(node_stack_size);

	//if we found a hit convert barycentric coords to max lod
	if(hit_lod != ~0u)
	{
	#if 1
		for(uint32_t i = hit_lod; i <  header.last_lod; ++i)
		{
			uint32_t tri_type;
			hit.bc *= glm::vec2(2.0f);

			if(hit.bc.x > 1.0f)
			{
				tri_type = 0;
				hit.bc -= glm::vec2(1.0f, 0.0f);
			}
			else if(hit.bc.y > 1.0f)
			{
				tri_type = 1;
				hit.bc -= glm::vec2(0.0f, 1.0f);
			}
			else if(hit.bc.x + hit.bc.y < 1.0f)
			{
				tri_type = 2;
			}
			else
			{
				tri_type = 3;
				hit.bc = glm::vec2(1.0f, 1.0f) - hit.bc;
			}

			hit.id <<= 2;
			hit.id |= tri_type;
		}
	#endif
		hit.id += header.id_offset;
	}

	return hit_lod != ~0u;
}