#pragma once

#include "vec2.hpp"
#include "vec3.hpp"
#include "uvec3.hpp"
#include "triangle.hpp"
#include "bvh.hpp"
#include "bvh-simtrax.hpp"

#ifndef __riscv
#include <vector>
#include <string>
#include <fstream>
#include <cassert>
#include <map>
#include <set>
#include <unordered_set>
#include <deque>

namespace rtm 
{

class Mesh
{
public:
	std::vector<rtm::uvec3> vertex_indices;
	std::vector<rtm::uvec3> normal_indices;
	std::vector<rtm::uvec3> tex_coord_indices;

	std::vector<rtm::vec3> vertices;
	std::vector<rtm::vec3> normals;
	std::vector<rtm::vec2> tex_coords;

	std::vector<uint> material_indices;
	std::vector<std::string> material_names;
	std::string mtl_lib;

public:
	Mesh(std::string file_path) : mtl_lib("")
	{
 		load_obj(file_path.c_str());
	}

	static inline rtm::vec2 read_vec2(char* line)
	{
		rtm::vec2 v;
		uint index = 0;
		for(uint i = 0; i < 2; ++i)
		{
			uint start = index;
			while(line[index] != ' ' && line[index] != '\0') index++;
			char c = line[index];
			line[index] = '\0';
			v[i] = std::atof(&line[start]);

			if(c == '\0') break; //can't consume endline outside of consume line by convetion
			index++;
		}

		return v;
	}

	static inline rtm::vec3 read_vec3(char* line)
	{
		rtm::vec3 v;
		uint index = 0;
		for(uint i = 0; i < 3; ++i)
		{
			uint start = index;
			while(line[index] != ' ' && line[index] != '\0') index++;
			char c = line[index];
			line[index] = '\0';
			v[i] = std::atof(&line[start]);

			if(c == '\0') break;
			index++;
		}

		return v;
	}

	static inline std::string read_str(char* data)
	{
		std::string str = data;
		while(str.back() == '\n' || str.back() == '\r') str.pop_back();
		return str;
	}

	static inline void read_face(char* line, rtm::uvec3& vrt_inds, rtm::uvec3& txcd_inds, rtm::uvec3& nrml_inds)
	{
		uint index = 0;
		for(uint i = 0; i < 3; ++i)
		{
			uint start = index;
			while(line[index] != '/' && line[index] != ' ' && line[index] != '\0') ++index;
			char c = line[index];
			line[index] = '\0';
			vrt_inds[i] = std::atoi(&line[start]) - 1;

			if(c == '\0') break;
			index++;
			if(c == ' ') continue;

			start = index;
			while(line[index] != '/' && line[index] != ' ' && line[index] != '\0') index++;
			c = line[index];
			line[index] = '\0';
			uint txcd_ind = std::atoi(&line[start]);
			if(txcd_ind != 0) txcd_inds[i] = txcd_ind - 1;

			if(c == '\0') break;
			index++;
			if(c == ' ') continue;

			start = index;
			while(line[index] != ' ' && line[index] != '\0') index++;
			c = line[index];
			line[index] = '\0';
			uint nrml_ind = std::atoi(&line[start]);
			if(nrml_ind != 0) nrml_inds[i] = nrml_ind - 1;

			if(c == '\0') break; //can't consume endline outside of consume line by convetion
			index++;
		}
	}


	bool load_obj(const char* file_path)
	{
		printf("Loading: %s\n", file_path);

		std::ifstream is(file_path);
		if(!is.is_open()) return false;
		is.seekg(0, std::ios_base::end);
		std::size_t size = is.tellg();
		is.seekg(0, std::ios_base::beg);

		std::vector<char> data(size + 2);
		is.read((char*)&data[0], size);
		data[size] = '\n';
		data[size+1] = '\0';
		is.close();

		//simple hashes for switch statement
		constexpr uint8_t v = 'v' + ' ';

		constexpr uint8_t vt = 'v' + 't';
		constexpr uint8_t vn = 'v' + 'n';
		constexpr uint8_t vp = 'v' + 'p';

		constexpr uint8_t f = 'f' + ' ';
		constexpr uint8_t l = 'l' + ' ';

		constexpr uint8_t usemtl = 'u' + 's';
		constexpr uint8_t mtllib = 'm' + 't';

		constexpr uint8_t o = 'o' + ' ';
		constexpr uint8_t g = 'g' + ' ';

		constexpr uint8_t s = 's' + ' ';

		uint64_t next_line_start = 0;
		uint64_t line_number = 0;

		char c = data[next_line_start];
		while(next_line_start < size)
		{
			data[next_line_start] = c;

			//get the next line
			uint64_t line_size = 0;
			char* line = &data[next_line_start];
			while(line[line_size++] != '\n');
			next_line_start += line_size;

			//insert null charter after newline
			c = data[next_line_start];
			data[next_line_start] = '\0';

			//ignore comments and empty lines
			if(line[0] != 'v' &&
				line[0] != 'f' &&
				line[0] != 'l' &&
				line[0] != 'u' &&
				line[0] != 'm' &&
				line[0] != 'o' &&
				line[0] != 'g' &&
				line[0] != 's'
				) continue;

			//simple hash of first two characters on the line
			uint8_t type = line[0] + line[1];

			//advance to start of data
			uint data_start_index = 0;
			while(line[data_start_index] != ' ' && data_start_index < line_size) data_start_index++;
			while(line[data_start_index] == ' ' && data_start_index < line_size) data_start_index++;
			if(data_start_index == line_size) continue;

			switch(type)
			{
			case v:
				vertices.push_back(read_vec3(line + data_start_index));
				break;

			case vt:
				tex_coords.push_back(read_vec2(line + data_start_index));
				break;

			case vn:
				normals.push_back(rtm::normalize(read_vec3(line + data_start_index)));
				break;

			case f:
				vertex_indices.emplace_back(0);
				tex_coord_indices.emplace_back(~0x0u);
				normal_indices.emplace_back(~0x0u);
				material_indices.emplace_back(material_names.size() - 1u);
				read_face(line + data_start_index, vertex_indices.back(), tex_coord_indices.back(), normal_indices.back());
				break;

			case mtllib:
				mtl_lib = read_str(line + data_start_index);
				break;

			case usemtl:
				material_names.push_back(read_str(line + data_start_index));
				break;

			case vp: //na
			case l: //na
			case o: //na
			case g: //na
			case s: //na
				break;

			default:
				printf("\nInvalid line: %jd\n", line_number);
				break;
			}

			line_number++;
		}
		tex_coords.emplace_back(0.0f, 0.0f);
		for(uint i = 0; i < tex_coord_indices.size(); ++i)
		{
			if(tex_coord_indices[i][0] == ~0x0u)
			{
				tex_coord_indices[i][0] = tex_coord_indices[i][1] = tex_coord_indices[i][2] = tex_coords.size() - 1;
			}
		}

		for(uint i = 0; i < normal_indices.size(); ++i)
		{
			if(normal_indices[i][0] == ~0x0u)
			{
				rtm::vec3 gn = rtm::normalize(rtm::cross(vertices[vertex_indices[i][1]] - vertices[vertex_indices[i][0]], vertices[vertex_indices[i][2]] - vertices[vertex_indices[i][0]]));
				normal_indices[i][0] = normal_indices[i][1] = normal_indices[i][2] = normals.size();
				normals.push_back(gn);
			}
		}

		printf("Loaded: %s\n", file_path);
		return true;
	}

	uint size() const { return (uint)vertex_indices.size(); }

	Triangle get_triangle(uint32_t index) const
	{
		return {vertices[vertex_indices[index][0]], vertices[vertex_indices[index][1]], vertices[vertex_indices[index][2]]};
	}

	void get_triangles(std::vector<Triangle>& triangles) const
	{
		triangles.clear();
		for(uint32_t i = 0; i < vertex_indices.size(); ++i)
			triangles.emplace_back(get_triangle(i));
	}

	BVH2::BuildObject get_build_object(uint i) const
	{
		Triangle triangle = get_triangle(i);
		BVH2::BuildObject build_object;
		build_object.aabb = triangle.aabb();
		build_object.cost = triangle.cost();
		build_object.index = i;
		return build_object;
	}

	void get_build_objects(std::vector<BVH2::BuildObject>& build_objects) const
	{
		build_objects.clear();
		for(uint32_t i = 0; i < vertex_indices.size(); ++i)
			build_objects.push_back(get_build_object(i));
	}

	BVHSIMTRAX::BuildObject get_build_object_sim(uint i) const
	{
		Triangle triangle = get_triangle(i);
		BVHSIMTRAX::BuildObject build_object;
		build_object.aabb = triangle.aabb();
		build_object.cost = triangle.cost();
		build_object.index = i;
		return build_object;
	}

	void get_build_objects(std::vector<BVHSIMTRAX::BuildObject>& build_objects) const
	{
		build_objects.clear();
		for(uint32_t i = 0; i < vertex_indices.size(); ++i)
			build_objects.push_back(get_build_object_sim(i));
	}

	float normalize_verts()
	{
		float32_bf bf(0.0f);

		uint8_t max_exp = 0;
		for(auto& v : vertices)
			for(uint i = 0; i < 3; ++i)
			{
				bf.f32 = v[i];
				max_exp = rtm::max(max_exp, bf.exp);
			}

		int delta = (126 - max_exp);
		for(auto& v : vertices)
			for(uint i = 0; i < 3; ++i)
			{
				bf.f32 = v[i];
				bf.exp += delta;
				v[i] = bf.f32;
			}

		return float32_bf(0, 127 + delta, 0).f32;
	}
	
	void quantize_verts()
	{
		for(auto& v : vertices)
			for(uint i = 0; i < 3; ++i)
				v[i] = u24_to_f32(f32_to_u24(v[i]));
	}

	void make_strips(std::vector<TriangleStrip>& strips)
	{
		//build face graph
		std::vector<rtm::uvec3> face_graph(vertex_indices.size(), uvec3(~0u));
		{
			std::map<std::pair<uint32_t, uint32_t>, std::pair<uint32_t, uint32_t>> edge_to_face_map;
			for(uint f = 0; f < vertex_indices.size(); ++f)
			{
				uvec3 face = vertex_indices[f];
				for(uint e = 0; e < 3; ++e)
				{
					std::pair<uint32_t, uint32_t> edge(face[(e + 1) % 3], face[(e + 2) % 3]);
					if(edge.first > edge.second) std::swap(edge.first, edge.second);
					if(edge_to_face_map.find(edge) != edge_to_face_map.end())
					{
						//link
						uint32_t other_f = edge_to_face_map[edge].first;
						uint32_t other_e = edge_to_face_map[edge].second;
						face_graph[f][e] = other_f;
						face_graph[other_f][other_e] = f;
					}
					else edge_to_face_map[edge] = {f, e};
				}
			}
		}

		for(auto& face : face_graph)
		{
			if(face[0] == face[1] && face[1] == face[2])
			{
				face[0] = ~0u;
				face[1] = ~0u;
				face[2] = ~0u;
			}
			if(face[0] == face[1])
			{
				face[0] = ~0u;
				face[1] = ~0u;
			}
			if(face[1] == face[2])
			{
				face[1] = ~0u;
				face[2] = ~0u;
			}
			if(face[2] == face[0])
			{
				face[2] = ~0u;
				face[0] = ~0u;
			}
		}

		//create a set of all faces
		std::vector<uint32_t> sorted_faces;
		std::set<uint32_t> remaining_faces;
		for(uint f = 0; f < vertex_indices.size(); ++f)
		{
			remaining_faces.insert(f);
			sorted_faces.push_back(f);
		}
		//std::sort(sorted_faces.begin(), sorted_faces.end(), [&](uint32_t& a, uint32_t& b)->bool
		//{
		//	return get_triangle(a).aabb().surface_area() < get_triangle(b).aabb().surface_area();
		//});

		//keep face indices for reordering
		std::vector<uint> face_indices;
		while(!remaining_faces.empty())
		{
			uint start_face = *remaining_faces.begin();
			//while(!sorted_faces.empty())
			//{
			//	start_face = sorted_faces.back();
			//	sorted_faces.pop_back();
			//	if(remaining_faces.count(start_face) > 0) break;
			//}

			//build a list of faces greedily optimizing the AABB SA
			std::deque<uint32_t> face_list = {start_face};
			remaining_faces.erase(face_list.back());

			AABB aabb;
			aabb.add(get_triangle(face_list.back()).aabb());
			while(face_list.size() < rtm::TriangleStrip::MAX_TRIS)
			{
				std::set<std::pair<uint, uint>> options;
				for(uint i = 0; i < 3; ++i)
				{
					options.insert({face_graph[face_list.back()][i], 1});
					options.insert({face_graph[face_list.front()][i], 0});
				}

				float best_sa = INFINITY;
				std::pair<uint, uint> best_face = {~0u, 0};
				for(auto& candidate_face : options)
				{
					if(candidate_face.first == ~0u || remaining_faces.count(candidate_face.first) == 0) continue;

					AABB candidate_aabb = aabb; candidate_aabb.add(get_triangle(candidate_face.first).aabb());
					float candidate_sa = candidate_aabb.surface_area();
					if(candidate_sa < best_sa)
					{
						best_face = candidate_face;
						best_sa = candidate_sa;
					}
				}

				if(best_face.first == ~0u) break;

				aabb.add(get_triangle(best_face.first).aabb());
				remaining_faces.erase(best_face.first);

				if(best_face.second) face_list.push_back(best_face.first);
				else                 face_list.push_front(best_face.first);

			}

			//extract the best substrip
			float best_sah = INFINITY;
			uint best_start = 0, best_end = face_list.size();
			//for(uint start = 0; start < 1; ++start)
			//	for(uint end = start + 1; end <= face_list.size(); ++end)
			//	{
			//		AABB aabb;
			//		uint size = end - start;
			//		if(size > TriangleStrip::MAX_TRIS) continue;
			//		for(uint i = start; i < end; ++i)
			//			aabb.add(get_triangle(face_list[i]).aabb());
			//	
			//		float sah = (float)face_list.size() / size * aabb.surface_area();
			//		if(sah < best_sah)
			//		{
			//			best_sah = sah;
			//			best_start = start;
			//			best_end = end;
			//		}
			//	}

			std::vector<uint> faces;
			{
				for(uint i = 0; i < best_start; ++i) remaining_faces.insert(face_list[i]);
				for(uint i = best_start; i < best_end; ++i) faces.push_back(face_list[i]);
				for(uint i = best_end; i < face_list.size(); ++i) remaining_faces.insert(face_list[i]);
			}

			//generate edge list
			std::vector<uint> exit_edges;
			std::vector<uint> entry_edges;
			for(uint i = 0; i < (faces.size() - 1); ++i)
			{
				uint current_face = faces[i];
				uint next_face = faces[i + 1];

				uint j;
				for(j = 0; j < 3; ++j)
				{
					uint adjacent_face = face_graph[current_face][j];
					if(adjacent_face == next_face) break;
				}
				assert(j < 3);
				exit_edges.push_back(j);

				for(j = 0; j < 3; ++j)
				{
					uint adjacent_face = face_graph[next_face][j];
					if(adjacent_face == current_face) break;
				}
				assert(j < 3);
				entry_edges.push_back(j);
			}

			//encode strip
			std::vector<uint> vis;
			for(uint i = 0; i < faces.size(); ++i)
			{
				uvec3 last_face;
				if(i == 0)
				{
					uint e = 0;
					if(faces.size() > 1)
					{
						e = exit_edges[i];
						exit_edges[i] = 0;
					}
					for(uint j = 0; j < 3; ++j)
					{
						uint vi = vertex_indices[faces[i]][(j + e) % 3];
						vis.push_back(vi);
					}
				}
				else
				{
					bool vrt_added = false;
					for(uint j = 0; j < 3; ++j)
					{
						uint vi = vertex_indices[faces[i]][j];
						bool vi_found = false;
						for(uint k = 0; k < 3; ++k)
							if(last_face[k] == vi)
								vi_found = true;

						if(vi_found) continue;

						vrt_added = true;
						vis.push_back(vi);
						if(i < (faces.size() - 1))
						{
							uint ne = (exit_edges[i] + (2 - j)) % 3;
							assert(ne < 2);
							exit_edges[i] = ne;
						}
					}

					if(!vrt_added)
					{
						assert(false);
						//uint j = (exit_edges[i] + 2) % 3;
						//uint vi = vertex_indices[faces[i]][j];
						//vis.push_back(vi);
						//if(i < (faces.size() - 1))
						//{
						//	uint ne = (exit_edges[i] + (2 - j)) % 3;
						//	assert(ne < 2);
						//	exit_edges[i] = ne;
						//}
					}
				}
				last_face = vertex_indices[faces[i]];
			}

			if(vis.size() < exit_edges.size() + 3) continue;

			TriangleStrip strip; 
			strip.id = face_indices.size();
			strip.num_tris = faces.size();
			strip.edge_mask = 0;
			for(uint i = 0; i < vis.size(); ++i) strip.vrts[i] = vertices[vis[i]];
			for(uint i = 0; i < exit_edges.size(); ++i) strip.edge_mask |= exit_edges[i] << i;
			strips.push_back(strip);

			for(uint i = 0; i < faces.size(); ++i)
				face_indices.push_back(faces[i]);
		}

		reorder(face_indices);
		printf("Tris per strip: %f\n", (float)face_indices.size() / strips.size());
	}

	void reorder(std::vector<BVH2::BuildObject>& ordered_build_objects)
	{
		assert(ordered_build_objects.size() == vertex_indices.size());
		std::vector<rtm::uvec3> tmp_vrt_inds(vertex_indices);
		std::vector<rtm::uvec3> tmp_nrml_inds(normal_indices);
		std::vector<rtm::uvec3> tmp_txcd_inds(tex_coord_indices);
		std::vector<uint>       tmp_mat_inds(material_indices);
		for (uint32_t i = 0; i < ordered_build_objects.size(); ++i)
		{
			vertex_indices[i]    = tmp_vrt_inds [ordered_build_objects[i].index];
			normal_indices[i]    = tmp_nrml_inds[ordered_build_objects[i].index];
			tex_coord_indices[i] = tmp_txcd_inds[ordered_build_objects[i].index];
			material_indices[i]  = tmp_mat_inds [ordered_build_objects[i].index];
			ordered_build_objects[i].index = i;
		}
	}

	void reorder(std::vector<BVHSIMTRAX::BuildObject>& ordered_build_objects)
	{
		assert(ordered_build_objects.size() == vertex_indices.size());
		std::vector<rtm::uvec3> tmp_vrt_inds(vertex_indices);
		std::vector<rtm::uvec3> tmp_nrml_inds(normal_indices);
		std::vector<rtm::uvec3> tmp_txcd_inds(tex_coord_indices);
		std::vector<uint>       tmp_mat_inds(material_indices);
		for (uint32_t i = 0; i < ordered_build_objects.size(); ++i)
		{
			vertex_indices[i] = tmp_vrt_inds[ordered_build_objects[i].index];
			normal_indices[i] = tmp_nrml_inds[ordered_build_objects[i].index];
			tex_coord_indices[i] = tmp_txcd_inds[ordered_build_objects[i].index];
			material_indices[i] = tmp_mat_inds[ordered_build_objects[i].index];
			ordered_build_objects[i].index = i;
		}
	}

	void reorder(const std::vector<uint>& face_indices)
	{
		assert(face_indices.size() == vertex_indices.size());
		std::vector<rtm::uvec3> tmp_vrt_inds(vertex_indices);
		std::vector<rtm::uvec3> tmp_nrml_inds(normal_indices);
		std::vector<rtm::uvec3> tmp_txcd_inds(tex_coord_indices);
		std::vector<uint>       tmp_mat_inds(material_indices);
		for(uint32_t i = 0; i < face_indices.size(); ++i)
		{
			vertex_indices[i] = tmp_vrt_inds[face_indices[i]];
			normal_indices[i] = tmp_nrml_inds[face_indices[i]];
			tex_coord_indices[i] = tmp_txcd_inds[face_indices[i]];
			material_indices[i] = tmp_mat_inds[face_indices[i]];
		}
	}
};

}

#endif