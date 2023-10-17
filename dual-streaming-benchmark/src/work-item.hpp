#pragma once
#include "stdafx.hpp"

struct BucketRay
{
	rtm::Ray ray;
	uint32_t id{~0u};
};

struct WorkItem
{
	BucketRay bray;
	uint32_t  segment;
};
