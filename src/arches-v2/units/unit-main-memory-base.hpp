#pragma once 
#include "stdafx.hpp"

#include "unit-memory-base.hpp"
#include "util/elf.hpp"
#include "util/stb_image_write.h"

namespace Arches { namespace Units {

class UnitMainMemoryBase : public UnitMemoryBase
{
public:
	size_t size_bytes;

	union 
	{
		uint8_t*  _data_u8;
		uint16_t* _data_u16;
		uint32_t* _data_u32;
		uint64_t* _data_u64;
	};

public:
	UnitMainMemoryBase(size_t size) : UnitMemoryBase()
	{
		size_bytes = size;
		_data_u64 = (uint64_t*)malloc(size);
	}

	virtual ~UnitMainMemoryBase()
	{
		free(_data_u64);
	}

	void clear()
	{
		memset(_data_u8, 0x00, size_bytes);
	}

	void direct_read(void* data, size_t size, paddr_t paddr) const
	{ 
		memcpy(data, _data_u8 + paddr, size);
	}

	void direct_write(const void* data, size_t size, paddr_t paddr)
	{
		memcpy(_data_u8 + paddr, data, size);
	}

	void dump_as_png_uint8(paddr_t from_paddr, size_t width, size_t height, std::string const& path)
	{
		uint8_t const* src = _data_u8 + from_paddr;
		stbi_flip_vertically_on_write(true);
		stbi_write_png(path.c_str(), static_cast<int>(width), static_cast<int>(height), 4, src, 0);
	}
};

}}