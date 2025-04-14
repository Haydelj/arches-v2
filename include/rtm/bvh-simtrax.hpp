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

class BVHSIMTRAX
{
public:
	const static uint32_t VERSION = 2395794618; //random number used to validate the cache
	const static uint MAX_PRIMS = 8;
	const float BVH_C_isec = 1.f;
	const float BVH_C_trav = 1.f;

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
				uint32_t num_prims  : 3; //num prim - 1
				uint32_t prim_index : 28; //first prim
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

	struct BVHCostEval
	{
		float position;
		float cost;
		int   numLeft;
		int   numRight;
		int   event;
		int   axis;
	};

	struct BVHSAHEvent
	{
		float position;
		float cost;
		float leftArea;
		float rightArea;
		int   objId;
		int   numLeft;
		int   numRight;
	};

	struct CompareBVHSAHEvent
	{
		bool operator()(const BVHSAHEvent& x, const BVHSAHEvent& y)
		{
			// add obj_id sorting in here automatically?
			return x.position < y.position;
		}
	};

#ifndef __riscv
	float sah_cost{0.0f};
	std::vector<Node> nodes;
	Node* build_nodes{nullptr};
	std::vector<int>         triangles;
	std::vector<int> trianglesOrdered;
	std::vector<BuildObject>& objects;
	int      numIntNodes{ 0 };
	int      numLeafNodes{ 0 };
	int      maxReachedDepth{ 0 };

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
					if (start == 4 && end == 532 && (axis == 0) && (start + i >= 471))
						printf("i: %d, Right box: boxMin[0]: %f, boxMin[1]: %f, boxMin[2]: %f, boxMax[0]: %f, boxMax[1]: %f, boxMax[2]: %f\n", i, right_aabb.min[0], right_aabb.min[1], right_aabb.min[2], right_aabb.max[0], right_aabb.max[1], right_aabb.max[2]);
					cost_right[i] = right_cost_sum * right_aabb.surface_area() * inv_aabb_sa;
				}

				for(uint i = 1; i < size; ++i)
				{
					float cost;
					// if(i == 0) cost = left_cost_sum;
					cost = cost_left[i] + cost_right[i] + AABB::cost();
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
					if(start == 4 && end == 532 && (axis == 0) && (start + i == 447 || start + i == 471))
						printf("i = %d, cost = %f\n", start + i, cost);
				}
			}
			// if(axis == 3) axis = 2;
			
			if(start == 4 && end == 532)
				printf("Start: %d, end: %d, best_axis: %d, best cost: %f, cost 0: %f, cost 1: %f, cost 2: %f, split 0: %d, split 1: %d, split 2: %d\n", start, end, best_axis, best_spliting_cost, costs[0], costs[1], costs[2], splits[0], splits[1], splits[2]);

			if(best_spliting_cost >= size)
				return ~0u;

			if(best_axis != ~0u)
			{
				std::sort(build_objects + start, build_objects + end, [&](const BuildObject& a, const BuildObject& b)
				{
					return a.aabb.centroid()[best_axis] < b.aabb.centroid()[best_axis];
				});
				printf("Resort from %d to %d, based on axis: %d\n", start, end, best_axis);

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
	BVHSIMTRAX() = default;

	BVHSIMTRAX(std::vector<BuildObject>& build_objects, uint quality = 2) : objects(build_objects)
	{
		build(build_objects, quality);
	}

	BVHSIMTRAX(std::string cache, std::vector<BuildObject>& build_objects, uint quality = 2) : objects(build_objects)
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
		printf("Loading BVHSIMTRAX: %s\n", file_path.c_str());

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
				printf("Loaded BVHSIMTRAX: %s\n", file_path.c_str());
				printf("Quality: %d\n", header.quality);
				printf("Cost: %f\n", header.sah_cost);
				printf("Nodes: %d\n", header.num_nodes);
				printf("Objects: %d\n", header.num_build_objects);
			}
		}

		if(!succeeded)
			printf("Failed to load BVHSIMTRAX: %s\n", file_path.c_str());

		return succeeded;
	}

	bool BuildEvents(
		int         const  first,
		int         const  last,
		int         const  axis,
		BVHCostEval       &bestEval)
	{
		AABB overallBox;
	
		std::vector<BVHSAHEvent> events;
		events.reserve(last - first);
		for (int i = first; i < last; ++i)
		{
			BVHSAHEvent newEvent;
			AABB        triBox;
			triBox.add(objects[triangles[i]].aabb);
	
			newEvent.position = 0.5f * (triBox.min[axis] + triBox.max[axis]);
			newEvent.objId    = i;
			events.push_back(newEvent);
	
			overallBox.add(triBox);
		}
		std::sort(events.begin(), events.end(), CompareBVHSAHEvent());
	
		int const numEvents = static_cast<int>(events.size());
		AABB leftBox;
	
		int numLeft  = 0;
		int numRight = numEvents;
	
		for (size_t i = 0; i < events.size(); ++i)
		{
			events[i].numLeft  = numLeft;
			events[i].numRight = numRight;
			events[i].leftArea = leftBox.surface_area();
	
			AABB triBox;
			triBox.add(objects[triangles[events[i].objId]].aabb);
	
			leftBox.add(triBox);
	
			numLeft++;
			numRight--;
		}
	
		AABB rightBox;
	
		bestEval.cost  = std::numeric_limits<float>::max();
		bestEval.event = -1;
	
		float const overallBoxAreaInv = 1.f / overallBox.surface_area();
		for (int i = numEvents - 1; i >= 0; --i)
		{
			AABB triBox;
			triBox.add(objects[triangles[events[i].objId]].aabb);
	
			rightBox.add(triBox);
	
			if (events[i].numLeft  > 0 &&
				events[i].numRight > 0)
			{
				events[i].rightArea = rightBox.surface_area();
	
				float thisCost = (events[i].numLeft  * events[i].leftArea +
								  events[i].numRight * events[i].rightArea);
				thisCost      *= overallBoxAreaInv;
				thisCost      *= BVH_C_isec;
				thisCost      += BVH_C_trav;
	
				events[i].cost = thisCost;
				if (thisCost < bestEval.cost)
				{
					bestEval.cost     = thisCost;
					bestEval.position = events[i].position;
					bestEval.axis     = axis;
					bestEval.event    = i;
					bestEval.numLeft  = events[i].numLeft;
					bestEval.numRight = events[i].numRight;
				}
			}
		}
		return (bestEval.event != -1);
	}

	int PartitionSAH(int const objBegin,
		int const objEnd,
		int& outputAxis)
	{
		int const numObjects = objEnd - objBegin;
		if (numObjects == 1)
		{
			outputAxis = -1;
			return -1;
		}
	
		BVHCostEval bestCost;
		bestCost.event = 0;
		bestCost.axis  = -1;
	#ifdef TREE_ROTATIONS
		bestCost.cost  = std::numeric_limits<float>::max();
	#else
		bestCost.cost  = BVH_C_isec * numObjects;
	#endif    
	
		for (int axis = 0; axis < 3; ++axis)
		{
			BVHCostEval newCost;
			if (BuildEvents(objBegin, objEnd, axis, newCost))
			{
				if (newCost.cost < bestCost.cost)
				{
					bestCost = newCost;
				}
			}
		}
	
		outputAxis = bestCost.axis;
		if (outputAxis != -1)
		{
			// build the events and sort them
			std::vector<BVHSAHEvent> events;
			events.reserve(objEnd - objBegin);
			for (int i = objBegin; i < objEnd; ++i)
			{
				AABB triBox;
				triBox.add(objects[triangles[i]].aabb);
	
				BVHSAHEvent newEvent;
				newEvent.position = 0.5f * (triBox.min[outputAxis] + triBox.max[outputAxis]);
				newEvent.objId    = i;
				events.push_back(newEvent);
			}
			std::sort(events.begin(), events.end(), CompareBVHSAHEvent());
	
			std::vector<int> copiedTris;
			copiedTris.reserve(events.size());
			for (auto e : events)
			{
				copiedTris.push_back(triangles[e.objId]);
			}
			for (int i = objBegin; i < objEnd; ++i)
			{
				triangles[i] = copiedTris[i - objBegin];
			}
	
			int const result = objBegin + bestCost.event;
			if (result == objBegin ||
				result == objEnd)
			{
				if (numObjects < 8)
				{
					outputAxis = -1;
					return 0;
				}
				return (objBegin + numObjects / 2);
			}
			else
				return result;
		}
		else
		{
			return 0; // making a leaf anyway
		}
	}

	void BuildHelper(
		int const  nodeID,
		int const  triBeginId,
		int const  triEndId,
		int       &nextFreeNode,
		int const  depth)
	{
		// TODO: might be interesting to do this in parallel
		assert(triEndId > triBeginId);
	
		Node &node = build_nodes[nodeID];
	
		int       bestAxis = -1;
		int const numTris  = triEndId - triBeginId;
		//int split = partitionObjectMedian(tri_begin, tri_end, bestAxis);
		int const split    = PartitionSAH(triBeginId, triEndId, bestAxis);
	
		if (bestAxis == -1)
		{
			numLeafNodes++;
			/*if(numLeafNodes < 100)
				printf("Generate tri: %d, start: %d, end: %d, num tris: %d\n", numLeafNodes, triBeginId, triEndId, numTris);*/
	
			// make leaf
			int const firstInOrder = static_cast<int>(trianglesOrdered.size());
			for (int i = 0; i < numTris; ++i)
			{
				trianglesOrdered.emplace_back(triangles[triBeginId + i]);
				node.aabb.add(objects[triangles[triBeginId + i]].aabb);
			}

			node.data.is_leaf = 1;
			node.data.num_prims = numTris;
			node.data.prim_index = triBeginId;
		}
		else
		{
			/*if(numIntNodes < 100)
				printf("Generate node: %d, start: %d, end: %d, split: %d\n", numIntNodes, triBeginId, triEndId, split);*/
			numIntNodes++;
	
			// make internal node
			node.data.is_leaf = 0;
			node.data.child_index = nextFreeNode;
			nextFreeNode += 2;
	
			BuildHelper(node.data.child_index,     triBeginId, split,    nextFreeNode, depth + 1);
			BuildHelper(node.data.child_index + 1, split,      triEndId, nextFreeNode, depth + 1);
		}
	
		maxReachedDepth = std::max(maxReachedDepth, depth);
		if (depth > 63)
		{
			printf("  WARNING: BVH depth just hit %d\n", depth);
		}
	}

	void UpdateBounds(int const nodeID)
	{
		Node &node = build_nodes[nodeID];
		if (node.data.is_leaf)
		{
			// for (int i = 0; i < node.data.num_prims; ++i)
			// {
			// 	int const childId = node.data.prim_index + i;
			// 	node.aabb.add(objects[trianglesOrdered[childId]].aabb);
			// }
			// node.data.prim_index = trianglesOrdered[node.data.prim_index];
		}
		else
		{
			int const leftNodeId  = node.data.child_index;
			int const rightNodeId = node.data.child_index + 1;
			UpdateBounds(leftNodeId);
			UpdateBounds(rightNodeId);

			Node &leftNode  = build_nodes[leftNodeId];
			Node &rightNode = build_nodes[rightNodeId];

			node.aabb = AABB();
			node.aabb.add(leftNode.aabb);
			node.aabb.add(rightNode.aabb);
		}
	}

	float ComputeNodeSAHCost(int const nodeID)
	{
		Node *node = build_nodes + nodeID;

		// leaf node
		if (node->data.is_leaf)
		{
			return (node->data.num_prims * BVH_C_isec);
		}

		int   const leftID    = node->data.child_index;
		int   const rightID   = leftID + 1;
		float const leftCost  = ComputeNodeSAHCost(leftID);
		float const rightCost = ComputeNodeSAHCost(rightID);

		float const area      = node->aabb.surface_area();
		float const leftArea  = build_nodes[leftID].aabb.surface_area();
		float const rightArea = build_nodes[rightID].aabb.surface_area();

		float cost = BVH_C_trav + (leftArea * leftCost + rightArea * rightCost) / area;
		return cost;
		// TODO: this divide cancels out the multiply by areas on the next level up
		// can be taken out, but this isn't part of the simulation, 
		// so doesn't really matter
	}

	void build(std::vector<BuildObject>& build_objects, uint quality = 2)
	{
		trianglesOrdered.clear();
		int const numTris = build_objects.size();
		build_nodes = new Node[2 * numTris];

		// Allocate necessary space
		trianglesOrdered.reserve(numTris);
		triangles       .reserve(numTris);
		for (int i = 0; i < numTris; ++i)
		{
			triangles.push_back(i);
		}

		// Recursively build the BVH in place
		int nextFree = 1;
		numIntNodes     = 0;
		numLeafNodes    = 0;
		maxReachedDepth = 0;
		printf("Starting BVH build.\n");
		BuildHelper(0, 0, numTris, nextFree, 0);
		printf("BVH build complete with %d interior + %d leaf = %d total nodes, max depth %d.\n",
			numIntNodes,
			numLeafNodes,
			(numIntNodes + numLeafNodes), 
			maxReachedDepth);

		for(uint i = 0; i < numTris; ++i)
		{
			build_objects[i].index = trianglesOrdered[i];
		}

		// Update metrics, etc
		UpdateBounds      (0);
		float cost = ComputeNodeSAHCost(0);
		printf("BVH: before rotations, SAH cost = %f\n", cost);

		nodes.assign(build_nodes, build_nodes + numIntNodes);
	}
#endif
};

}