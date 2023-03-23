#pragma once
#include "stdafx.hpp"

#include "intersect-tt.hpp"
#include "tesselation-tree2.hpp"

inline static bool intersect(const TesselationTree2Pointers& tes_tree, const float recip_max_error, const uint32_t patch_index, const Ray& ray, const glm::vec3& inv_d, Hit& hit)
{
	const TesselationTree2::Header header = tes_tree.headers[patch_index];

	struct NodeStackEntry
	{
		float t;
		uint32_t dbs;

		union
		{
			uint32_t data;
			struct
			{
				uint32_t node_index : 26;
				uint32_t lod : 4;
				uint32_t tri_type : 2;
			};
		};

		IAO iao;

		bool operator <(const NodeStackEntry& other) const { return t < other.t; }
	};

	struct TriangleStackEntry
	{
		Triangle tri;//TODO maybe remove?
		Triangle new_center_tri;
		bool child_transformed;
	};

	TriangleStackEntry tri_stack[8];
	NodeStackEntry node_stack[3 * 8];

	float max_db_over_max_error = header.max_db * recip_max_error;
	uint lli = header.last_lod;
	const TesselationTree2::Node* nodes = &tes_tree.nodes[header.root_node_offset];
	const glm::vec3* vertices = &tes_tree.vertices[header.root_vertex_offset];

	uint32_t node_stack_size = 1u;
	node_stack[0].t = intersect(nodes[0].aabb, ray, inv_d);
	node_stack[0].dbs = nodes[0].dbs;
	node_stack[0].node_index = 0;
	node_stack[0].lod = 0;
	node_stack[0].tri_type = 0;
	node_stack[0].iao.i = 0;
	node_stack[0].iao.a = 0;
	node_stack[0].iao.o = 0;

	uint32_t hit_lod = ~0u;
	do
	{
		NodeStackEntry& current_node_entry = node_stack[--node_stack_size];
		if(current_node_entry.t >= hit.t) continue;

		uint32_t dbs = current_node_entry.dbs;
		uint32_t node_index = current_node_entry.node_index;
		uint32_t lod = current_node_entry.lod;
		uint32_t tri_type = current_node_entry.tri_type;
		IAO iao = current_node_entry.iao;

		if(lod == 0) tri_stack[lod].tri = Triangle(tes_tree.vertices[header.vi[0]], tes_tree.vertices[header.vi[1]], tes_tree.vertices[header.vi[2]]);
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
		glm::uvec3 vi = iao_to_vi(iao, lod);
		tri_stack[lod].new_center_tri = get_transformed_triangle(tri_stack[lod].tri, Triangle(vertices[vi[0]], vertices[vi[1]], vertices[vi[2]]), edge_states);

		//if the next LOD is the leaf intersect othewise push the child nodes onto the stack
		uint32_t next_level = lod + 1;
		uint32_t first_child_index = node_index << 2; //compute first child idnex
		
		if((next_level) == header.last_lod) //leaf node
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

			iao.i = iao.i * 2 + (uint32_t)iao.a * (uint32_t)iao.a;
			iao.a = iao.a * 2;

			uint32_t temp_node_stack_size = node_stack_size;
			uint32_t absolute_first_child_index = lod_node_offset[next_level] + first_child_index;
			for(uint32_t i = 0; i < 4; ++i)
			{
				TesselationTree2::Node node = nodes[absolute_first_child_index + i];
				if(tri_stack[lod].child_transformed)
				{
					Triangle tri = reconstruct_triangle(tri_stack[lod].tri, tri_stack[lod].new_center_tri, i);
					for(uint32_t j = 0; j < 3; ++j)
					{
						node.aabb.min = glm::min(node.aabb.min, tri.vrts[j]);
						node.aabb.max = glm::max(node.aabb.max, tri.vrts[j]);
					}
				}

				float t = intersect(node.aabb, ray, inv_d);
				if(t < hit.t)
				{
					uint32_t j = node_stack_size++;
					for(; j > temp_node_stack_size; --j)
					{
						if(node_stack[j - 1].t >= t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].node_index = first_child_index + i;
					node_stack[j].lod = next_level;
					node_stack[j].tri_type = i;
					node_stack[j].iao = iao_to_child_iao(iao, i);
					node_stack[j].dbs = node.dbs;
				}
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

inline bool intersect(const TesselationTree2Pointers tes_tree, const float recip_max_error, const Ray& ray, Hit& hit)
{
	glm::vec3 inv_d = glm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		BVH::NodeData data;
	};

	float root_hit_t = intersect(tes_tree.blas[0].aabb, ray, inv_d);
	if(root_hit_t >= hit.t) return false;

	NodeStackEntry node_stack[3 * 16];
	uint32_t node_stack_size = 1u;
	node_stack[0].t = root_hit_t;
	node_stack[0].data = tes_tree.blas[0].data;

	bool found_hit = false;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) return found_hit;

		if(!current_entry.data.is_leaf)
		{
			for(uint32_t i = 0; i <= current_entry.data.lst_chld_ofst; ++i)
			{
				float t = intersect(tes_tree.blas[current_entry.data.fst_chld_ind + i].aabb, ray, inv_d);
				if(t < hit.t)
				{
					uint32_t j = node_stack_size++;
					for(; j != 0; --j)
					{
						if(node_stack[j - 1].t >= t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].data = tes_tree.blas[current_entry.data.fst_chld_ind + i].data;
				}
			}
		}
		else
		{
			for(uint32_t i = 0; i <= current_entry.data.lst_chld_ofst; ++i)
				found_hit |= intersect(tes_tree, recip_max_error, current_entry.data.fst_chld_ind + i, ray, inv_d, hit);
		}
	} while(node_stack_size);

	return found_hit;
}









///////////////////////////////////secondary\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

inline static uint find_shared_edge(const glm::uvec3& crnt_ptch_inds, const glm::uvec3& prev_ptch_inds)
{
	uint32_t match_mask = 0;
	for(uint i = 0; i < 3; ++i)
		for(uint j = 0; j < 3; ++j)
			if(crnt_ptch_inds[i] == prev_ptch_inds[j])
			{
				match_mask |= 1 << i;
				break;
			}

	if(match_mask == 6) return 0;
	if(match_mask == 5) return 1; 
	if(match_mask == 3) return 2;
	return 3;
}

inline static uint propagate_shared_edge(uint prnt_shrd_edg, uint tri_typ)
{
	if(prnt_shrd_edg == 3 || tri_typ == 3 || prnt_shrd_edg == tri_typ) return 3;
	return prnt_shrd_edg;
}

inline glm::vec3 evalute_deformation_bounds(uint32_t qdb, float max_db_over_max_error, const Triangle& tri, const Ray& last_ray,  const Ray& ray, const bool edge_mask[3])
{
	glm::vec3 fdbs((float)(qdb >> 00 & 0x3ff), (float)(qdb >> 10 & 0x3ff), (float)(qdb >> 20 & 0x3ff));
	fdbs *= (1.0f / 1023.0f);
	fdbs *= fdbs;
	fdbs *= max_db_over_max_error;

	glm::vec3 dlen(glm::dot(tri.vrts[0] - ray.o, ray.d), glm::dot(tri.vrts[1] - ray.o, ray.d), glm::dot(tri.vrts[2] - ray.o, ray.d));
	glm::vec3 last_dlen(glm::dot(tri.vrts[0] - last_ray.o, last_ray.d), glm::dot(tri.vrts[1] - last_ray.o, last_ray.d), glm::dot(tri.vrts[2] - last_ray.o, last_ray.d));
	
	glm::vec3 r;
	if(edge_mask[0]) r[0] = std::min(last_dlen[1], last_dlen[2]) * last_ray.drdt + last_ray.radius;
	else             r[0] = std::min(     dlen[1],      dlen[2]) *      ray.drdt +      ray.radius;
	if(edge_mask[1]) r[1] = std::min(last_dlen[2], last_dlen[0]) * last_ray.drdt + last_ray.radius;
	else             r[1] = std::min(     dlen[2],      dlen[0]) *      ray.drdt +      ray.radius;
	if(edge_mask[2]) r[2] = std::min(last_dlen[0], last_dlen[1]) * last_ray.drdt + last_ray.radius;
	else             r[2] = std::min(     dlen[0],      dlen[1]) *      ray.drdt +      ray.radius;
	glm::vec3 recip_r(_rcp(r[0]), _rcp(r[1]), _rcp(r[2]));

	glm::vec3 delta = fdbs * recip_r - 1.0f;
	return glm::clamp(delta, 0.0f, 1.0f);
}

inline bool intersect(const TesselationTree2Pointers& tes_tree, const float recip_max_error, const uint32_t last_patch_index, const Ray& last_ray, const uint32_t patch_index, const Ray& ray, const glm::vec3& inv_d, Hit& hit)
{
	glm::uvec3 previous_patch_indices(0);
	if(last_patch_index != ~0) previous_patch_indices  = tes_tree.headers[last_patch_index].vi;
	const TesselationTree2::Header header = tes_tree.headers[patch_index];

	struct NodeStackEntry
	{
		float t;
		uint32_t dbs;

		union
		{
			uint32_t data;
			struct
			{
				uint32_t node_index : 26;
				uint32_t lod : 4;
				uint32_t tri_type : 2;
			};
		};

		IAO iao;

		bool operator <(const NodeStackEntry& other) const { return t < other.t; }
	};

	struct TriangleStackEntry
	{
		Triangle tri;//TODO maybe remove?
		Triangle new_center_tri;
		uint8_t shared_edge;
		bool child_transformed;
	};

	TriangleStackEntry tri_stack[8];
	NodeStackEntry node_stack[3 * 8];

	float max_db_over_max_error = header.max_db * recip_max_error;
	uint lli = header.last_lod;
	const TesselationTree2::Node* nodes = &tes_tree.nodes[header.root_node_offset];
	const glm::vec3* vertices = &tes_tree.vertices[header.root_vertex_offset];

	uint32_t node_stack_size = 1u;
	node_stack[0].t = intersect(nodes[0].aabb, ray, inv_d);
	node_stack[0].dbs = nodes[0].dbs;
	node_stack[0].node_index = 0;
	node_stack[0].lod = 0;
	node_stack[0].tri_type = 0;
	node_stack[0].iao.i = 0;
	node_stack[0].iao.a = 0;
	node_stack[0].iao.o = 0;

	uint32_t hit_lod = ~0u;
	do
	{
		NodeStackEntry& current_node_entry = node_stack[--node_stack_size];
		if(current_node_entry.t >= hit.t) continue;

		uint32_t dbs = current_node_entry.dbs;
		uint32_t node_index = current_node_entry.node_index;
		uint32_t lod = current_node_entry.lod;
		uint32_t tri_type = current_node_entry.tri_type;
		IAO iao = current_node_entry.iao;

		//if(lod == 0) tri_stack[lod].tri = Triangle(tes_tree.vertices[header.vi[0]], tes_tree.vertices[header.vi[1]], tes_tree.vertices[header.vi[2]]);
		//else         tri_stack[lod].tri = reconstruct_triangle(tri_stack[lod - 1u].tri, tri_stack[lod - 1u].new_center_tri, tri_type);

		if(lod == 0)
		{
			tri_stack[lod].tri = Triangle(tes_tree.vertices[header.vi[0]], tes_tree.vertices[header.vi[1]], tes_tree.vertices[header.vi[2]]);
			if(last_patch_index != ~0) tri_stack[0].shared_edge = find_shared_edge(header.vi, previous_patch_indices);
			else                       tri_stack[0].shared_edge = 3;
		}
		else
		{
			tri_stack[lod].tri = reconstruct_triangle(tri_stack[lod - 1u].tri, tri_stack[lod - 1u].new_center_tri, tri_type);
			tri_stack[lod].shared_edge = propagate_shared_edge(tri_stack[lod - 1u].shared_edge, tri_type);
		}

		//determine if triangle needs subdivision. Probably should use hardware
		bool edge_mask[3] = {patch_index == last_patch_index || tri_stack[lod].shared_edge == 0, patch_index == last_patch_index || tri_stack[lod].shared_edge == 1, patch_index == last_patch_index || tri_stack[lod].shared_edge == 2};
		glm::vec3 edge_states = evalute_deformation_bounds(dbs, max_db_over_max_error, tri_stack[lod].tri, last_ray, ray, edge_mask);

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
		glm::uvec3 vi = iao_to_vi(iao, lod);
		tri_stack[lod].new_center_tri = get_transformed_triangle(tri_stack[lod].tri, Triangle(vertices[vi[0]], vertices[vi[1]], vertices[vi[2]]), edge_states);

		//if the next LOD is the leaf intersect othewise push the child nodes onto the stack
		uint32_t next_level = lod + 1;
		uint32_t first_child_index = node_index << 2; //compute first child idnex
		
		if((next_level) == header.last_lod) //leaf node
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

			iao.i = iao.i * 2 + (uint32_t)iao.a * (uint32_t)iao.a;
			iao.a = iao.a * 2;

			uint32_t temp_node_stack_size = node_stack_size;
			uint32_t absolute_first_child_index = lod_node_offset[next_level] + first_child_index;
			for(uint32_t i = 0; i < 4; ++i)
			{
				TesselationTree2::Node node = nodes[absolute_first_child_index + i];
				if(tri_stack[lod].child_transformed)
				{
					Triangle tri = reconstruct_triangle(tri_stack[lod].tri, tri_stack[lod].new_center_tri, i);
					for(uint32_t j = 0; j < 3; ++j)
					{
						node.aabb.min = glm::min(node.aabb.min, tri.vrts[j]);
						node.aabb.max = glm::max(node.aabb.max, tri.vrts[j]);
					}
				}

				float t = intersect(node.aabb, ray, inv_d);
				if(t < hit.t)
				{
					uint32_t j = node_stack_size++;
					for(; j > temp_node_stack_size; --j)
					{
						if(node_stack[j - 1].t >= t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].node_index = first_child_index + i;
					node_stack[j].lod = next_level;
					node_stack[j].tri_type = i;
					node_stack[j].iao = iao_to_child_iao(iao, i);
					node_stack[j].dbs = node.dbs;
				}
			}
		}
	} while(node_stack_size);

	//if we found a hit convert barycentric coords to max lod
	if(hit_lod != ~0u)
	{
	#if 1
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

inline bool intersect(const TesselationTree2Pointers& tes_tree, const float recip_max_error, const uint32_t last_patch_index, const Ray& last_ray, const Ray& ray, Hit& hit)
{
	glm::vec3 inv_d = glm::vec3(1.0f) / ray.d;

	struct NodeStackEntry
	{
		float t;
		BVH::NodeData data;
	};

	float root_hit_t = intersect(tes_tree.blas[0].aabb, ray, inv_d);
	if(root_hit_t >= hit.t) return false;

	NodeStackEntry node_stack[3 * 16];
	uint32_t node_stack_size = 1u;
	node_stack[0].t = root_hit_t;
	node_stack[0].data = tes_tree.blas[0].data;

	bool found_hit = false;
	do
	{
		NodeStackEntry current_entry = node_stack[--node_stack_size];
		if(current_entry.t >= hit.t) return found_hit;

		if(!current_entry.data.is_leaf)
		{
			for(uint32_t i = 0; i <= current_entry.data.lst_chld_ofst; ++i)
			{
				float t = intersect(tes_tree.blas[current_entry.data.fst_chld_ind + i].aabb, ray, inv_d);
				if(t < hit.t)
				{
					uint32_t j = node_stack_size++;
					for(; j != 0; --j)
					{
						if(node_stack[j - 1].t >= t) break;
						node_stack[j] = node_stack[j - 1];
					}

					node_stack[j].t = t;
					node_stack[j].data = tes_tree.blas[current_entry.data.fst_chld_ind + i].data;
				}
			}
		}
		else
		{
			for(uint32_t i = 0; i <= current_entry.data.lst_chld_ofst; ++i)
				found_hit |= intersect(tes_tree, recip_max_error, last_patch_index, last_ray, current_entry.data.fst_chld_ind + i, ray, inv_d, hit);
		}
	} while(node_stack_size);

	return found_hit;
}