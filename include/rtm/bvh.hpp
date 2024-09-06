#pragma once

#include "int.hpp"
#include "aabb.hpp"

#ifndef __riscv
#include <vector>
#include <algorithm>
#include <cassert>
#endif

namespace rtm {

#ifdef _DEBUG
#define BUILD_QUALITY 0
#else
#define BUILD_QUALITY 2
#endif
	

#if defined(WIDE_COMPRESSED_BVH)
	constexpr uint max_children = 1;
#else
	constexpr uint max_children = 8;
#endif

class BVH2
{
public:
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
			uint32_t is_leaf : 1;
			uint32_t lst_chld_ofst : 3;
			uint32_t fst_chld_ind : 28;
		};

		AABB       aabb;
		Data       data;
		uint32_t   _pad;
	};

#ifndef __riscv
	float cost{0.0f};
	std::vector<Node> nodes;

private:
	struct BuildEvent
	{
		uint start;
		uint end;
		uint node_index;

		uint split_build_objects(AABB aabb, BuildObject* build_objects)
		{
			uint size = end - start;
			if(size <= 1) return ~0u;

		#if BUILD_QUALITY == 0
			if(size > max_children)
				return split_build_objects_radix(aabb, build_objects);
		#elif BUILD_QUALITY == 1
			if(size > 64)
				return split_build_objects_radix(aabb, build_objects);
		#endif

			uint best_axis = 0;
			uint best_spliting_index = 0;
			float best_spliting_cost = FLT_MAX;

			float aabb_sa = aabb.surface_area();
			float inv_aabb_sa = 1.0f / aabb_sa;

			uint axis = aabb.longest_axis();
		#if BUILD_QUALITY > 0
			for(axis = 0; axis < 3; ++axis)
		#endif
			{
				std::sort(build_objects + start, build_objects + end, [&](const BuildObject& a, const BuildObject& b)
				{
					return a.aabb.centroid()[axis] < b.aabb.centroid()[axis];
				});

				std::vector<float> cost_left(size);
				std::vector<float> cost_right(size);

				AABB left_aabb, right_aabb;
				float left_cost_sum = 0.0f, right_cost_sum = 0.0f;

				for(uint i = 0; i < size; ++i)
				{
					cost_left[i] = AABB::cost() + left_cost_sum * left_aabb.surface_area() * inv_aabb_sa;
					left_aabb.add(build_objects[start + i].aabb);
					left_cost_sum += build_objects[start + i].cost;
				}
				cost_left[0] = 0.0f;

				for(uint i = size - 1; i < size; --i)
				{
					right_cost_sum += build_objects[start + i].cost;
					right_aabb.add(build_objects[start + i].aabb);
					cost_right[i] = AABB::cost() + right_cost_sum * right_aabb.surface_area() * inv_aabb_sa;
				}

				std::vector<float> costs;
				for(uint i = 0; i < size; ++i)
				{
					float cost;
					if(i == 0) cost = left_cost_sum;
					else       cost = cost_left[i] + cost_right[i];
					costs.push_back(cost);

					if(cost < best_spliting_cost)
					{
						best_spliting_index = start + i;
						best_spliting_cost = cost;
						best_axis = axis;
					}
				}
			}
			if(axis == 3) axis = 2;

			if(axis != best_axis)
			{
				std::sort(build_objects + start, build_objects + end, [&](const BuildObject& a, const BuildObject& b)
				{
					return a.aabb.centroid()[best_axis] < b.aabb.centroid()[best_axis];
				});
			}

			if(best_spliting_index == start)
			{
				if(size <= max_children) return ~0;
				else                     return (start + end) / 2;
			}

			return best_spliting_index;
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
				if(size <= max_children) return ~0u;
				else                     return (start + end) / 2;
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

public:
	void build(std::vector<BuildObject>& build_objects)
	{
		printf("BVH2 Building\n");
		nodes.clear();

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

		while(!event_stack.empty())
		{
			BuildEvent current_build_event = event_stack.back(); event_stack.pop_back();

			AABB aabb;
			for(uint i = current_build_event.start; i < current_build_event.end; ++i)
				aabb.add(build_objects[i].aabb);

			uint splitting_index = current_build_event.split_build_objects(aabb, build_objects.data());
			if(splitting_index != ~0)
			{
				nodes[current_build_event.node_index].aabb = aabb;
				nodes[current_build_event.node_index].data.is_leaf = 0;
				nodes[current_build_event.node_index].data.lst_chld_ofst = 1;
				nodes[current_build_event.node_index].data.fst_chld_ind = nodes.size();

				for(uint i = 0; i < 2; ++i)
					nodes.emplace_back();

				event_stack.push_back({splitting_index, current_build_event.end, (uint)nodes.size() - 1});
				event_stack.push_back({current_build_event.start, splitting_index, (uint)nodes.size() - 2});
			}
			else
			{
				//didn't do any splitting meaning this build event can become a leaf node
				uint size = current_build_event.end - current_build_event.start;
				assert(size <= max_children && size >= 1);

				nodes[current_build_event.node_index].aabb = aabb;
				nodes[current_build_event.node_index].data.is_leaf = 1;
				nodes[current_build_event.node_index].data.lst_chld_ofst = size - 1;
				nodes[current_build_event.node_index].data.fst_chld_ind = current_build_event.start;
			}
		}

		//compute sah
		std::vector<float> costs(nodes.size());
		for(uint i = nodes.size() - 1; i < nodes.size(); --i)
		{
			costs[i] = 0.0f;
			if(nodes[i].data.is_leaf)
			{
				for(uint j = 0; j <= nodes[i].data.lst_chld_ofst; ++j)
					costs[i] += build_objects[nodes[i].data.fst_chld_ind + j].cost;
			}
			else
			{
				float sa = nodes[i].aabb.surface_area();
				if(sa > 0.0f)
				{
					for(uint j = 0; j <= nodes[i].data.lst_chld_ofst; ++j)
					{
						uint ci = nodes[i].data.fst_chld_ind + j;
						costs[i] +=  AABB::cost() + costs[ci] * std::max(nodes[ci].aabb.surface_area(), 0.0f) / sa;
					}
				}
			}
		}
		cost = costs[0];

		printf("BVH2 Built: %.2f\n", cost);
	}
#endif
};

}