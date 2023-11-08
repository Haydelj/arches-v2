#pragma once
#include "stdafx.hpp"

#include "rtm.hpp"
#include "random-number-generator.hpp"
#include "sampling.hpp"
#include "bvh.hpp"
#include "camera.hpp"
#include "ray.hpp"
#include "intersect.hpp"
#include "intersect-tt1.hpp"
#include "intersect-tt4.hpp"

#define GLOBAL_DATA_ADDRESS 64ull

struct GlobalData
{
	uint32_t framebuffer_width;
	uint32_t framebuffer_height;
	uint32_t framebuffer_size;
	uint32_t* framebuffer;

	Camera camera;

	uint bounces; 

	uint config;

	union
	{
		BVH::Node* blas;
		BVH::CompressedNode4* cblas;
	};

	union
	{
		MeshPointers mesh;
		TesselationTree1Pointers tt1;
		TesselationTree4Pointers tt4;
	};
	
	glm::uvec3* ni;
	glm::vec3* normals;
};

bool intersect(const GlobalData& global_data, const Ray& ray, Hit& hit)
{
	switch(global_data.config)
	{
		case 0: return intersect(global_data.cblas, global_data.mesh, ray, hit); //BVHC
		case 1: return intersect(global_data.cblas, global_data.tt4, ray, hit); //TTC
		case 2: return intersect(global_data.blas, global_data.mesh, ray, hit); //BVH
		case 3: return intersect(global_data.blas, global_data.tt1, ray, hit); //TT
	}
	return false;
}

bool intersect(const GlobalData& global_data, uint last_patch_index, const Ray& last_ray, Ray& ray, Hit& hit)
{
	switch(global_data.config)
	{
		case 0: 
			return intersect(global_data.cblas, global_data.mesh, ray, hit); //BVHC
		case 1: 
		{

			TesselationTree4SecondaryRayData data4(global_data.tt4, last_patch_index, last_ray);
			return intersect(global_data.cblas, data4, ray, hit); //TTC
		}
		case 2: 
			return intersect(global_data.blas, global_data.mesh, ray, hit); //BVH
		case 3:
		{
			TesselationTree1SecondaryRayData data1(global_data.tt1, last_patch_index, last_ray);
 			return intersect(global_data.blas, data1, ray, hit); //TT
		}
	}
	return false;
}

