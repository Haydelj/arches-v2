#pragma once
#include "stdafx.hpp"

struct BucketRay
{
	rtm::Ray ray;
	uint32_t id{~0u};
};

#define INVALID_SEGMENT_ID 0x00ffffffu

struct WorkItem
{
	BucketRay bray;
	union
	{
		struct
		{
			uint32_t  segment_id : 24;
			uint32_t  order_hint : 8;
		};
		uint32_t _data;
	};
};
