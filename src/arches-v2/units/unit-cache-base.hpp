#pragma once 
#include "stdafx.hpp"

#include "unit-main-memory-base.hpp"
#include "util/bit-manipulation.hpp"
#include "util/alignment-allocator.hpp"

namespace Arches { namespace Units {

class UnitCacheBase : public UnitMemoryBase
{
public:
	UnitCacheBase(size_t size, uint block_size, uint associativity, uint sector_size = 0);
	virtual ~UnitCacheBase();

	void serialize(std::string file_path);
	bool deserialize(std::string file_path, const UnitMainMemoryBase& main_mem);

protected:
	struct BlockMetaData
	{
		uint64_t tag     : 52;
		uint64_t lru     : 4;
		uint64_t dirty   : 4;
		uint64_t valid   : 4;
	};

	struct Victim
	{
		paddr_t addr;
		uint8_t* data;
		uint8_t dirty;
		uint8_t valid;
	};

	uint64_t _set_index_mask, _tag_mask, _block_offset_mask;
	uint _set_index_offset, _tag_offset;

	uint _associativity, _block_size, _sector_size;
	std::vector<BlockMetaData, AlignmentAllocator<BlockMetaData, 64>> _tag_array;
	std::vector<uint8_t, AlignmentAllocator<uint8_t, 64>> _data_array;

	uint8_t* _read_block(paddr_t block_addr);
	uint8_t* _read_block(paddr_t sector_addr, uint8_t& valid_mask);
	uint8_t* _write_sector(paddr_t sector_addr, const uint8_t* data, bool set_dirty = false);
	Victim _insert_block(paddr_t block_addr);

	paddr_t _get_sector_index(paddr_t paddr) { return _get_block_offset(paddr) / _sector_size;}
	paddr_t _get_block_offset(paddr_t paddr) { return (paddr >> 0) & _block_offset_mask; }
	paddr_t _get_block_addr(paddr_t paddr) { return paddr & ~_block_offset_mask; }
	paddr_t _get_block_addr(uint64_t tag, uint64_t set_index) { return (tag << _tag_offset) | (set_index << _set_index_offset); }
	paddr_t _get_set_index(paddr_t paddr) { return  (paddr >> _set_index_offset) & _set_index_mask; }
	paddr_t _get_tag(paddr_t paddr) { return (paddr >> _tag_offset) & _tag_mask; }
};

}}