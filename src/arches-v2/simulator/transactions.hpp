#pragma once

#include "stdafx.hpp"

#include "dual-streaming-kernel/work-item.hpp"


namespace Arches {

struct MemoryRange
{
	paddr_t paddr;
	const char* data_type = nullptr;
	bool operator < (const MemoryRange& other) const
	{
		return paddr < other.paddr;
	};
};
struct MemoryRequest
{
public:
	enum class Type : uint8_t
	{
		NA,

		LOAD,
		STORE,
		PREFETCH,

		AMO_ADD,
		AMO_XOR,
		AMO_OR,
		AMO_AND,
		AMO_MIN,
		AMO_MAX,
		AMO_MINU,
		AMO_MAXU,
	};

	const static uint MAX_SIZE = CACHE_BLOCK_SIZE;

	//meta data 
	Type     type;
	uint8_t  size;
	uint16_t flags;
	uint16_t dst;
	uint16_t port;

	const char* unit_name; // From which unit?

	union
	{
		paddr_t paddr;
		vaddr_t vaddr;
	};

	union
	{
		uint8_t  data[MAX_SIZE];
		uint8_t  data_u8;
		uint16_t data_u16;
		uint32_t data_u32;
		uint64_t data_u64;
	};

public:
	MemoryRequest() = default;

	MemoryRequest(const MemoryRequest& other)
	{
		_assert(other.size <= MAX_SIZE);
		*this = other;
	}

	MemoryRequest& operator=(const MemoryRequest& other)
	{
		_assert(other.size <= MAX_SIZE);
		type = other.type;
		size = other.size;
		flags = other.flags;
		dst = other.dst;
		port = other.port;
		paddr = other.paddr;
		std::memcpy(data, other.data, other.size);

		unit_name = other.unit_name;

		return *this;
	}
};

struct MemoryReturn
{
public:
	//meta data 
	uint8_t  size;
	uint16_t dst;
	uint16_t port;

	union
	{
		paddr_t paddr;
		vaddr_t vaddr;
	};

	union
	{
		uint8_t  data[MemoryRequest::MAX_SIZE];
		uint8_t  data_u8;
		uint16_t data_u16;
		uint32_t data_u32;
		uint64_t data_u64;
	};

public:
	MemoryReturn() = default;

	MemoryReturn(const MemoryReturn& other)
	{
		_assert(other.size <= MemoryRequest::MAX_SIZE);
		*this = other;
	}

	MemoryReturn(const MemoryRequest& request, const void* data) : size(request.size), dst(request.dst), port(request.port), paddr(request.paddr)
	{
		_assert(request.size <= MemoryRequest::MAX_SIZE);
		std::memcpy(this->data, data, request.size);
	}

	MemoryReturn& operator=(const MemoryReturn& other)
	{
		_assert(other.size <= MemoryRequest::MAX_SIZE);
		size = other.size;
		dst = other.dst;
		port = other.port;
		paddr = other.paddr;
		std::memcpy(data, other.data, size);
		return *this;
	}
};

struct StreamSchedulerRequest
{
	enum class Type : uint8_t
	{
		NA,
		STORE_WORKITEM,
		LOAD_BUCKET,
		BUCKET_COMPLETE,
	};

	Type     type{Type::NA};
	uint16_t port;

	union
	{
		WorkItem swi;
		struct
		{
			uint previous_segment_id;
		}lb;
		struct
		{
			uint segment_id;
		}bc;
	};

	StreamSchedulerRequest() {};

	StreamSchedulerRequest(const MemoryReturn& other)
	{
		*this = other;
	}

	StreamSchedulerRequest& operator=(const StreamSchedulerRequest& other)
	{
		type = other.type;
		port = other.port;
		if(type == StreamSchedulerRequest::Type::STORE_WORKITEM)
		{
			swi = other.swi;
		}
		else if(type == StreamSchedulerRequest::Type::LOAD_BUCKET)
		{
			lb = other.lb;
		}
		else if(type == StreamSchedulerRequest::Type::BUCKET_COMPLETE)
		{
			bc = other.bc;
		}
	}
};

struct SFURequest
{
	uint16_t dst;
	uint16_t port;
};

struct SceneBufferLoadRequest
{
	uint sink = ~0u;
	paddr_t paddr = ~0u;
	uint8_t size;
	uint16_t dst;
	uint16_t port;
};

}
