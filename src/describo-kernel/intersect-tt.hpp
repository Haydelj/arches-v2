#pragma once
#include "stdafx.hpp"

#include "intersect.hpp"
#include "tesselation-tree.hpp"

inline Triangle decompact(const CompactTri& ctri)
{
	uint64_t dw0 = ctri.data[0];
	uint64_t dw1 = ctri.data[1];
	uint64_t dw2 = ctri.data[2];
	uint64_t dw3 = ctri.data[3];

	float f0 = u24_to_f32(dw0 & 0xffffff);
	float f1 = u24_to_f32((dw0 >> 24) & 0xffffff);
	float f2 = u24_to_f32(((dw0 >> 48) | (dw1 << 16)) & 0xffffff);
	float f3 = u24_to_f32((dw1 >> 8) & 0xffffff);
	float f4 = u24_to_f32((dw1 >> 32) & 0xffffff);
	float f5 = u24_to_f32(((dw1 >> 56) | (dw2 << 8)) & 0xffffff);
	float f6 = u24_to_f32((dw2 >> 16) & 0xffffff);
	float f7 = u24_to_f32((dw2 >> 40) & 0xffffff);
	float f8 = u24_to_f32(dw3 & 0xffffff);

	return Triangle(glm::vec3(f0, f1, f2), glm::vec3(f3, f4, f5), glm::vec3(f6, f7, f8));
}

inline glm::vec3 evalute_deformation_bounds(uint32_t qdb, float max_db_over_max_error, const Triangle& tri, const Ray& ray)
{
	glm::vec3 fdbs((float)(qdb >> 00 & 0x3ff), (float)(qdb >> 10 & 0x3ff), (float)(qdb >> 20 & 0x3ff));
	fdbs *= (1.0f / 1023.0f);
	fdbs *= fdbs;
	fdbs *= max_db_over_max_error;

	glm::vec3 dlen(glm::dot(tri.vrts[0] - ray.o, ray.d), glm::dot(tri.vrts[1] - ray.o, ray.d), glm::dot(tri.vrts[2] - ray.o, ray.d));
	glm::vec3 mdlen(std::min(dlen[1], dlen[2]), std::min(dlen[2], dlen[0]), std::min(dlen[0], dlen[1]));
	glm::vec3 r = mdlen * ray.drdt + ray.radius;
	glm::vec3 recip_r(_rcp(r[0]), _rcp(r[1]), _rcp(r[2]));

	glm::vec3 delta = fdbs * recip_r - 1.0f;
	return glm::clamp(delta, 0.0f, 1.0f);
}

inline Triangle get_transformed_triangle(const Triangle& prnt_tri, const Triangle& cntr_tri, const glm::vec3& edge_states)
{
	return Triangle(
		mix((prnt_tri.vrts[1] + prnt_tri.vrts[2]) * 0.5f, cntr_tri.vrts[0], edge_states[0]),
		mix((prnt_tri.vrts[2] + prnt_tri.vrts[0]) * 0.5f, cntr_tri.vrts[1], edge_states[1]),
		mix((prnt_tri.vrts[0] + prnt_tri.vrts[1]) * 0.5f, cntr_tri.vrts[2], edge_states[2])
	);
}

inline static Triangle reconstruct_triangle(const Triangle& prnt_tri, const Triangle& cntr_tri, const uint32_t tri_type)
{
	if(tri_type == 0) return Triangle(prnt_tri.vrts[0], cntr_tri.vrts[2], cntr_tri.vrts[1]);
	else if(tri_type == 1) return Triangle(cntr_tri.vrts[2], prnt_tri.vrts[1], cntr_tri.vrts[0]);
	else if(tri_type == 2) return Triangle(cntr_tri.vrts[1], cntr_tri.vrts[0], prnt_tri.vrts[2]);
	else /*if(tri_type == 3)*/ return Triangle(cntr_tri.vrts[0], cntr_tri.vrts[1], cntr_tri.vrts[2]);
}

const uint lod_node_offset[16] = {0, 1, 5, 21, 85, 341, 1365, 5461, 21845, 87381, 349525};

struct IAO
{
	uint32_t i;
	uint16_t a;
	uint8_t o;
};

inline IAO iao_to_child_iao(const IAO& in, uint type)
{
	IAO out;
	type |= in.o << 2;
	switch(type)
	{
	case 0b000:
		out.a = in.a + 1;
		out.i = in.i + in.a + 2;
		out.o = 0;
		break;

	case 0b001:
		out.a = in.a;
		out.i = in.i;
		out.o = 0;
		break;

	case 0b010:
		out.a = in.a + 1;
		out.i = in.i + in.a + 1;
		out.o = 0;
		break;

	case 0b011:
		out.a = in.a + 1;
		out.i = in.i + in.a + 1;
		out.o = 1;
		break;

	case 0b100:
		out.a = in.a;
		out.i = in.i;
		out.o = 1;
		break;

	case 0b101:
		out.a = in.a + 1;
		out.i = in.i + in.a + 2;
		out.o = 1;
		break;

	case 0b110:
		out.a = in.a;
		out.i = in.i + 1;
		out.o = 1;
		break;

	case 0b111:
		out.a = in.a;
		out.i = in.i + 1;
		out.o = 0;
		break;
	}
	return out;
}

const uint lod_face_offset[16] = {0, 1, 4, 14, 50, 186, 714, 2784, 11050, 43946, 175274};

inline glm::uvec3 iao_to_vi(const IAO& iao, uint lod)
{
	uint face_start = (lod_face_offset[lod] + iao.i) * 3;
	if(iao.o == 0) //up
	{
		glm::uvec3 vi;
		vi[0] = face_start + 0;
		vi[1] = face_start + 1;
		vi[2] = face_start + 2;
		return vi;
	}
	else //down
	{
		glm::uvec3 vi;
		vi[0] = face_start + 3;
		vi[1] = face_start + 1 - (iao.a * 3);
		vi[2] = face_start + 2;
		return vi;
	}
}

inline Triangle get_tri(const IAO& iao, uint lod, const CompactTri* triangles)
{
	uint face = lod_face_offset[lod] + iao.i;
	if(iao.o == 0) //up
	{
		const CompactTri& tri = triangles[face];
		return decompact(tri);
	}
	else //down
	{
		const CompactTri& tri0 = triangles[face + 1];
		const CompactTri& tri1 = triangles[face - iao.a];
		const CompactTri& tri2 = triangles[face];
		return Triangle(decompact(tri0).vrts[0], decompact(tri1).vrts[1], decompact(tri2).vrts[2]);
	}
}

inline uint find_shared_edge(const glm::uvec3& crnt_ptch_inds, const glm::uvec3& prev_ptch_inds)
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

inline uint propagate_shared_edge(uint prnt_shrd_edg, uint tri_typ)
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