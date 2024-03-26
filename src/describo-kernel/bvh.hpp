#pragma once
#include "stdafx.hpp"

#include "aabb.hpp"

constexpr uint max_children = 8;

#define BVH_SPLITS 2
#define BVH_FACTOR (1 << BVH_SPLITS)
#define USE_PACKED_NODES

struct BuildObject
{
	AABB  aabb;
	float cost;
	uint  index;
};

class BVH
{
public:
	union NodeData
	{
		struct
		{
			uint32_t is_leaf : 1;
			uint32_t lst_chld_ofst : 3;
			uint32_t fst_chld_ind : 28;
		};
		uint32_t _u32;
	};

	struct alignas(32) Node
	{
		AABB     aabb;
		NodeData data;
		uint32_t _pad;
	};

	struct alignas(16) CompressedNode
	{
		AABB16 aabb;
		NodeData data;
	};

	struct alignas(64) CompressedNode4
	{
		CompressedNode nodes[4];
	};

#ifdef ARCH_X86
	float cost{1.0f};
	std::vector<Node> nodes;

	void compress_and_pack(std::vector<CompressedNode4>& packed_nodes)
	{
		size_t num_packed_nodes = 0;
		struct NodeSet
		{
			BVH::NodeData data;
			uint32_t      index;

			NodeSet() = default;
			NodeSet(BVH::NodeData data, uint32_t index) : data(data), index(index) {};
		};

		packed_nodes.clear();

		std::vector<NodeSet> stack; stack.reserve(96);
		stack.emplace_back(nodes[0].data, 0);
		packed_nodes.emplace_back();

		while(!stack.empty())
		{
			NodeSet current_set = stack.back();
			stack.pop_back();

			for(uint i = 0; i < 4; ++i)
			{
				CompressedNode4& current_packed_node = packed_nodes[current_set.index];
				current_packed_node.nodes[i].aabb.min[0] = 0.0f;
				current_packed_node.nodes[i].aabb.max[0] = 0.0f;
				current_packed_node.nodes[i].aabb.min[1] = 0.0f;
				current_packed_node.nodes[i].aabb.max[1] = 0.0f;
				current_packed_node.nodes[i].aabb.min[2] = 0.0f;
				current_packed_node.nodes[i].aabb.max[2] = 0.0f;
				current_packed_node.nodes[i].data.fst_chld_ind = 0u;
				current_packed_node.nodes[i].data.lst_chld_ofst = 0u;
				current_packed_node.nodes[i].data.is_leaf = 1;
			}

			for(uint i = 0; i <= current_set.data.lst_chld_ofst; ++i)
			{
				uint index = current_set.data.fst_chld_ind + i;
				BVH::Node node = nodes[index];

				CompressedNode4& current_packed_node = packed_nodes[current_set.index];
				current_packed_node.nodes[i].aabb = AABB16(node.aabb);
				current_packed_node.nodes[i].data = node.data;

				if(!node.data.is_leaf)
				{
					stack.emplace_back(node.data, packed_nodes.size());
					current_packed_node.nodes[i].data.fst_chld_ind = packed_nodes.size();
					packed_nodes.emplace_back();
				}
			}
		}
	}

private:
	class _BuildObjectComparatorSpacial
	{
	public:
		_BuildObjectComparatorSpacial(uint axis) : _axis(axis) {}
		bool operator()(const BuildObject& a, const BuildObject& b) { return a.aabb.centroid()[_axis] < b.aabb.centroid()[_axis]; };
	private:
		uint _axis;
	};

	struct _BuildEvent
	{
		uint start{0};
		uint end{0};
		uint node_index{~0u};

		uint split_build_objects(AABB aabb, BuildObject* build_objects)
		{
			uint size = end - start;
			if(size <= 1) return ~0u;

			uint best_axis = 0;
			uint best_spliting_index = 0;
			float best_spliting_cost = FLT_MAX;

			float inv_aabb_sa = 1.0f / aabb.surface_area();

			uint axis = aabb.longest_axis();
			for (axis = 0; axis < 3; ++axis)
			{
				_BuildObjectComparatorSpacial comparator(axis);
				std::sort(build_objects + start, build_objects + end, comparator);

				std::vector<float> cost_left(size);
				std::vector<float> cost_right(size);

				AABB left_aabb, right_aabb;
				float left_cost_sum = 0.0f, right_cost_sum = 0.0f;

				for (uint i = 0; i < size; ++i)
				{
					cost_left[i] = left_cost_sum * left_aabb.surface_area() * inv_aabb_sa;
					left_cost_sum += build_objects[start + i].cost;
					left_aabb.add(build_objects[start + i].aabb);

				}
				cost_left[0] = 0.0f;

				for(uint i = size - 1; i < size; --i)
				{
					right_cost_sum += build_objects[start + i].cost;
					right_aabb.add(build_objects[start + i].aabb);
					cost_right[i] = right_cost_sum * right_aabb.surface_area() * inv_aabb_sa;
				}

				for (uint i = 0; i < size; ++i)
				{
					float cost = cost_left[i] + cost_right[i];
					if(i != 0) cost += AABB::cost() * 2.0f;
					if (cost < best_spliting_cost)
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
				_BuildObjectComparatorSpacial comparator(best_axis);
				std::sort(build_objects + start, build_objects + end, comparator);
			}

			if (best_spliting_index == start)
			{
				if (start - end <= max_children) return ~0;
				else                             return (start + end) / 2;
			}

			return best_spliting_index;
		}
	};

public:
	void build(std::vector<BuildObject>& build_objects, uint splits = 2)
	{
		printf("BVH%d Building\r", 1 << splits);
		nodes.clear();

		std::vector<_BuildEvent> event_stack;
		event_stack.emplace_back();
		event_stack.back().start = 0;
		event_stack.back().end = build_objects.size();
		event_stack.back().node_index = 0; nodes.emplace_back();

		while(!event_stack.empty())
		{
			_BuildEvent current_build_event = event_stack.back(); event_stack.pop_back();

			AABB aabb;
			for(uint i = current_build_event.start; i < current_build_event.end; ++i)
				aabb.add(build_objects[i].aabb);

			std::vector<_BuildEvent> build_events = {current_build_event};
			for(uint i = 0; i < splits; ++i)
			{
				std::vector<_BuildEvent> new_build_events;
				for(auto& build_event : build_events)
				{
					uint splitting_index = build_event.split_build_objects(aabb, build_objects.data());
					if(splitting_index != ~0)
					{
						{
							_BuildEvent new_event;
							new_event.start = build_event.start;
							new_event.end = splitting_index;
							new_build_events.push_back(new_event);
						}
						{
							_BuildEvent new_event;
							new_event.start = splitting_index;
							new_event.end = build_event.end;
							new_build_events.push_back(new_event);
						}
					}
					else
					{
						new_build_events.push_back(build_event);
					}
				}

				if(new_build_events.size() == build_events.size()) break;

				build_events.clear();
				build_events = new_build_events;
			}

			//didn't do any splitting meaning this build event can become a leaf node
			if(build_events.size() == 1)
			{
				uint size = current_build_event.end - current_build_event.start;
				assert(size <= max_children && size >= 1);

				nodes[current_build_event.node_index].aabb = aabb;
				nodes[current_build_event.node_index].data.is_leaf = 1;
				nodes[current_build_event.node_index].data.lst_chld_ofst = size - 1;
				nodes[current_build_event.node_index].data.fst_chld_ind = current_build_event.start;
			}
			else
			{
				nodes[current_build_event.node_index].aabb = aabb;
				nodes[current_build_event.node_index].data.is_leaf = 0;
				nodes[current_build_event.node_index].data.lst_chld_ofst = build_events.size() - 1;
				nodes[current_build_event.node_index].data.fst_chld_ind = nodes.size();

				for(uint i = 0; i < build_events.size(); ++i)
					build_events[i].node_index = nodes.size(), nodes.emplace_back();

				for(uint i = build_events.size() - 1; i < build_events.size(); --i)
					event_stack.push_back(build_events[i]);
			}
		}

		std::vector<float> costs(nodes.size());
		for(uint i = nodes.size() - 1; i < nodes.size(); --i)
		{
			costs[i] = AABB::cost();
			if(nodes[i].data.is_leaf)
			{
				for(uint j = 0; j <= nodes[i].data.lst_chld_ofst; ++j)
					costs[i] += build_objects[nodes[i].data.fst_chld_ind + j].cost;
 			}
			else
			{
				float rcp_sa = 1.0f / nodes[i].aabb.surface_area();
				for(uint j = 0; j <= nodes[i].data.lst_chld_ofst; ++j)
				{
					uint ci = nodes[i].data.fst_chld_ind + j;
					costs[i] += costs[ci] * nodes[ci].aabb.surface_area() * rcp_sa;
				}
			}
		}
		cost = costs[0];

		printf("BVH%d Built: %.2f \n", 1 << splits, cost);
	}
#endif
};

