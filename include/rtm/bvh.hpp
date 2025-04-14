#pragma once

#include "int.hpp"
#include "aabb.hpp"

#ifndef __riscv
#include <vector>
#include <algorithm>
#include <cassert>
#include <fstream>
#endif

namespace rtm {

#ifdef _DEBUG
#define BUILD_QUALITY 0
#else
#define BUILD_QUALITY 2
#endif

class BVH2
{
public:
	const static uint32_t VERSION = 2395794618; //random number used to validate the cache
	const static uint MAX_PRIMS = 8;

	struct BuildObject
	{
		AABB  aabb{};
		float cost{0.0f};
		uint  index{~0u};
		uint64_t morton_code{~0u};
	};

	struct alignas(32) Node
	{
		struct Data
		{
			struct
			{
				uint32_t is_leaf    : 1;
				uint32_t num_prims  : 5; //num prim - 1
				uint32_t prim_index : 26; //first prim
			};
			struct
			{
				uint32_t             : 1;
				uint32_t child_index : 31; //left child
			};
		};

		AABB       aabb;
		Data       data;
		uint32_t   _pad;
	};

#ifndef __riscv
	float sah_cost{0.0f};
	std::vector<Node> nodes;

private:
	struct BuildEvent
	{
		uint start;
		uint end;
		uint node_index;

		uint split_build_objects(AABB aabb, BuildObject* build_objects, uint quality)
		{
			uint size = end - start;
			if(size <= 1) return ~0u;

			if(quality == 0 && size > MAX_PRIMS)
				return split_build_objects_radix(aabb, build_objects);

			if(quality == 1 && size > 64)
				return split_build_objects_radix(aabb, build_objects);

			return split_build_objects_sah(aabb, build_objects);
		}

		uint split_build_objects_sah(AABB aabb, BuildObject* build_objects)
		{
			uint size = end - start;
			if(size <= 1) return ~0u;

			uint best_axis = ~0u;
			uint best_spliting_index = 0;
			float best_spliting_cost = FLT_MAX;

			float aabb_sa = aabb.surface_area();
			float inv_aabb_sa = 1.0f / aabb_sa;

			uint axis = aabb.longest_axis();
			std::vector<float> costs;
			costs.resize(3, FLT_MAX);
			std::vector<int> splits;
			splits.resize(3);
			for(axis = 0; axis < 3; ++axis)
			{
				std::vector<BuildObject> objects;
				for(uint i = start; i < end; ++i)
				{
					objects.push_back(build_objects[i]);
				}

				std::sort(objects.begin(), objects.end(), [&](const BuildObject& a, const BuildObject& b)
				{
					return a.aabb.centroid()[axis] < b.aabb.centroid()[axis];
				});

				std::vector<float> cost_left(size);
				std::vector<float> cost_right(size);

				AABB left_aabb, right_aabb;
				float left_cost_sum = 0.0f, right_cost_sum = 0.0f;

				for(uint i = 0; i < size; ++i)
				{
					if (start == 4 && end == 532 && (axis == 0) && (start + i == 471))
						int te = 0;
					cost_left[i] = left_cost_sum * left_aabb.surface_area() * inv_aabb_sa;
					left_aabb.add(objects[i].aabb);
					left_cost_sum += objects[i].cost;
				}
				cost_left[0] = 0.0f;

				for(uint i = size - 1; i < size; --i)
				{
					right_cost_sum += objects[i].cost;
					right_aabb.add(objects[i].aabb);
					/*if (start == 4 && end == 532 && (axis == 0) && (start + i >= 471))
						printf("i: %d, Right box: boxMin[0]: %f, boxMin[1]: %f, boxMin[2]: %f, boxMax[0]: %f, boxMax[1]: %f, boxMax[2]: %f\n", i, right_aabb.min[0], right_aabb.min[1], right_aabb.min[2], right_aabb.max[0], right_aabb.max[1], right_aabb.max[2]);*/
					cost_right[i] = right_cost_sum * right_aabb.surface_area() * inv_aabb_sa;
				}

				for(uint i = 0; i < size; ++i)
				{
					float cost;
					if(i == 0) cost = left_cost_sum;
					else cost = cost_left[i] + cost_right[i] + AABB::cost();
					// costs.push_back(cost);
					if(cost < costs[axis])
					{
						costs[axis] = cost;
						splits[axis] = start + i;
					}
						
					if(cost < best_spliting_cost)
					{
						best_spliting_index = start + i;
						best_spliting_cost = cost;
						best_axis = axis;
					}
					/*if(start == 4 && end == 532 && (axis == 0) && (start + i == 447 || start + i == 471))
						printf("i = %d, cost = %f\n", start + i, cost);*/
				}
			}
			// if(axis == 3) axis = 2;
			
			/*if(start == 4 && end == 532)
				printf("Start: %d, end: %d, best_axis: %d, best cost: %f, cost 0: %f, cost 1: %f, cost 2: %f, split 0: %d, split 1: %d, split 2: %d\n", start, end, best_axis, best_spliting_cost, costs[0], costs[1], costs[2], splits[0], splits[1], splits[2]);*/
			
			// float max_cost = 0.0f;
			// for(uint i = start; i < end; ++i)
			// 	max_cost += build_objects[i].cost;

			// if(best_spliting_cost >= max_cost)
			// 	return ~0u;

			if(best_axis != ~0u)
			{
				std::sort(build_objects + start, build_objects + end, [&](const BuildObject& a, const BuildObject& b)
				{
					return a.aabb.centroid()[best_axis] < b.aabb.centroid()[best_axis];
				});
				//printf("Resort from %d to %d, based on axis: %d\n", start, end, best_axis);

				if((best_spliting_index == start) || (best_spliting_index == end))
				{
					if(size < MAX_PRIMS)  return ~0u;
					else                  return (start + size / 2);
				}
				else
					return best_spliting_index;
			}
			else
				return ~0u;
		}

		uint split_build_objects_radix(AABB aabb, BuildObject* build_objects)
		{
			uint size = end - start;
			if(size <= 1) return ~0u;

			uint64_t common_prefix = build_objects[start].morton_code;
			uint64_t common_prefix_size = 64;
			for(uint i = start; i < end; ++i)
			{
				uint64_t mask = common_prefix ^ build_objects[i].morton_code;
				common_prefix_size = std::min(common_prefix_size, _lzcnt_u64(mask));
			}

			//All keys are identical. An arbitrary split
			if(common_prefix_size == 64)
			{
				if(size <= MAX_PRIMS) return ~0u;
				else                  return (start + end) / 2;
			}

			uint64_t sort_bit_mask = 1ull << (63 - common_prefix_size);
			uint head = start;
			uint tail = end - 1;

			while(head != tail)
			{
				if(!(build_objects[head].morton_code & sort_bit_mask))
				{
					head++;
					continue;
				}

				if((build_objects[tail].morton_code & sort_bit_mask))
				{
					tail--;
					continue;
				}

				std::swap(build_objects[head], build_objects[tail]);
			}

			return head;
		}
	};

	struct FileHeader
	{
		uint32_t version;
		uint8_t  quality;
		float    sah_cost;
		uint32_t num_nodes;
		uint32_t num_build_objects;
	};

public:
	BVH2() = default;

	BVH2(std::vector<BuildObject>& build_objects, uint quality = 2)
	{
		build(build_objects, quality);
	}

	BVH2(std::string cache, std::vector<BuildObject>& build_objects, uint quality = 2)
	{
		if(!deserialize(cache, build_objects, quality))
		{
			build(build_objects, quality);
			serialize(cache, build_objects, quality);
		}
	}

	void serialize(std::string file_path, const std::vector<BuildObject>& build_objects, uint quality)
	{
		std::ofstream file_stream(file_path, std::ios::binary);

		FileHeader header;
		header.version = VERSION;
		header.quality = quality;
		header.sah_cost = sah_cost;
		header.num_nodes = nodes.size();
		header.num_build_objects = build_objects.size();

		file_stream.write((char*)&header, sizeof(FileHeader));
		file_stream.write((char*)nodes.data(), sizeof(Node) * nodes.size());
		file_stream.write((char*)build_objects.data(), sizeof(BuildObject) * build_objects.size());
	}

	bool deserialize(std::string file_path, std::vector<BuildObject>& build_objects, uint quality = ~0u)
	{
		printf("Loading BVH2: %s\n", file_path.c_str());

		bool succeeded = false;
		std::ifstream file_stream(file_path, std::ios::binary);
		if(file_stream.is_open())
		{
			FileHeader header;
			file_stream.read((char*)&header, sizeof(FileHeader));

			if(header.version == VERSION
				&& (header.num_build_objects == build_objects.size())
				&& (quality == ~0u || header.quality == quality))
			{
				nodes.resize(header.num_nodes);
				file_stream.read((char*)nodes.data(), sizeof(Node) * nodes.size());
				file_stream.read((char*)build_objects.data(), sizeof(BuildObject) * build_objects.size());
				sah_cost = header.sah_cost;

				succeeded = true;
				printf("Loaded BVH2: %s\n", file_path.c_str());
				printf("Quality: %d\n", header.quality);
				printf("Cost: %f\n", header.sah_cost);
				printf("Nodes: %d\n", header.num_nodes);
				printf("Objects: %d\n", header.num_build_objects);
			}
		}

		if(!succeeded)
			printf("Failed to load BVH2: %s\n", file_path.c_str());

		return succeeded;
	}

	void build(std::vector<BuildObject>& build_objects, uint quality = 2)
	{
		printf("Building BVH2\n");
		nodes.clear();
		int num_int_nodes = 0, num_leaf_nodes = 0;

		//Build morton codes for build objects
		AABB cent_aabb;
		for(auto& build_object : build_objects)
			cent_aabb.add(build_object.aabb.centroid());

		rtm::vec3 scale = (cent_aabb.max + (1.0f / (1 << 20))) - cent_aabb.min;
		for(auto& build_object : build_objects)
		{
			rtm::vec3 cent = build_object.aabb.centroid();
			cent = (cent - cent_aabb.min) / scale;

			uint64_t x = cent.x * (1ull << 20);
			uint64_t y = cent.y * (1ull << 20);
			uint64_t z = cent.z * (1ull << 20);

			build_object.morton_code = 0;
			build_object.morton_code |= _pdep_u64(x, 0b001001001001001001001001001001001001001001001001001001001001ull);
			build_object.morton_code |= _pdep_u64(y, 0b010010010010010010010010010010010010010010010010010010010010ull);
			build_object.morton_code |= _pdep_u64(z, 0b100100100100100100100100100100100100100100100100100100100100ull);
		}

		std::vector<BuildEvent> event_stack;
		event_stack.emplace_back();
		event_stack.back().start = 0;
		event_stack.back().end = build_objects.size();
		event_stack.back().node_index = 0; nodes.emplace_back();
		printf("Start building, start: %d, end: %d\n", 0, build_objects.size());

		while(!event_stack.empty())
		{
			BuildEvent current_build_event = event_stack.back(); event_stack.pop_back();

			AABB aabb;
			for(uint i = current_build_event.start; i < current_build_event.end; ++i)
				aabb.add(build_objects[i].aabb);

			uint splitting_index = current_build_event.split_build_objects(aabb, build_objects.data(), quality);
			if(splitting_index != ~0)
			{
				//if(num_int_nodes < 100)
					//printf("Generate node: %d, start: %d, end: %d, split: %d\n", num_int_nodes, current_build_event.start, current_build_event.end, splitting_index);
				num_int_nodes++;
				nodes[current_build_event.node_index].aabb = aabb;
				nodes[current_build_event.node_index].data.is_leaf = 0;
				nodes[current_build_event.node_index].data.child_index = nodes.size();

				for(uint i = 0; i < 2; ++i)
					nodes.emplace_back();

				event_stack.push_back({splitting_index, current_build_event.end, (uint)nodes.size() - 1});
				event_stack.push_back({current_build_event.start, splitting_index, (uint)nodes.size() - 2});
			}
			else
			{
				num_leaf_nodes++;
				//didn't do any splitting meaning this build event can become a leaf node
				uint size = current_build_event.end - current_build_event.start;
				//assert(size <= MAX_PRIMS && size >= 1);
				//if(num_leaf_nodes < 100)
					//printf("Generate tri: %d, start: %d, end: %d, num tris: %d\n", num_leaf_nodes, current_build_event.start, current_build_event.end, size);

				nodes[current_build_event.node_index].aabb = aabb;
				nodes[current_build_event.node_index].data.is_leaf = 1;
				nodes[current_build_event.node_index].data.num_prims = size - 1;
				nodes[current_build_event.node_index].data.prim_index = current_build_event.start;
			}
		}

		//compute sah
		std::vector<float> costs(nodes.size());
		for(uint i = nodes.size() - 1; i < nodes.size(); --i)
		{
			costs[i] = 0.0f;
			if(nodes[i].data.is_leaf)
			{
				for(uint j = 0; j <= nodes[i].data.num_prims; ++j)
					costs[i] += build_objects[nodes[i].data.prim_index + j].cost;
			}
			else
			{
				float sa = nodes[i].aabb.surface_area();
				if(sa > 0.0f)
				{
					for(uint j = 0; j < 2; ++j)
					{
						uint ci = nodes[i].data.child_index + j;
						costs[i] += AABB::cost() + costs[ci] * std::max(nodes[ci].aabb.surface_area(), 0.0f) / sa;
					}
				}
			}
		}
		sah_cost = costs[0];

		printf("Built BVH2\n");
		printf("Quality: %d\n", quality);
		printf("Cost: %f\n", sah_cost);
		printf("Nodes: %d, interior nodes: %d, leaf nodes: %d\n", (uint)nodes.size(), num_int_nodes, num_leaf_nodes);
		printf("Objects: %d\n", (uint)build_objects.size());
	}
#endif
};

}