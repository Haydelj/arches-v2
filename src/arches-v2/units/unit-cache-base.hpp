#pragma once 
#include "stdafx.hpp"

#include "unit-memory-base.hpp"
#include "util/bit-manipulation.hpp"

namespace Arches { namespace Units {

class UnitCacheBase : public UnitMemoryBase
{
public:
	UnitCacheBase(size_t size, uint associativity);
	virtual ~UnitCacheBase();

protected:
	struct BlockMetaData
	{
		uint64_t tag     : 59;
		uint64_t lru     : 4;
		uint64_t valid   : 1;

		BlockMetaData()
		{
			valid = 0;
		}
	};

	struct alignas(CACHE_BLOCK_SIZE) BlockData
	{
		uint8_t bytes[CACHE_BLOCK_SIZE];
	};

	uint64_t _set_index_mask, _tag_mask, _block_offset_mask;
	uint _set_index_offset, _tag_offset;

	uint _associativity;
	std::vector<BlockMetaData> _tag_array;
	std::vector<BlockData> _data_array;

	BlockData* _get_block(paddr_t paddr);
	BlockData* _insert_block(paddr_t paddr, const uint8_t* data);

	paddr_t _get_block_offset(paddr_t paddr) { return  (paddr >> 0) & _block_offset_mask; }
	paddr_t _get_block_addr(paddr_t paddr) { return paddr & ~_block_offset_mask; }
	paddr_t _get_set_index(paddr_t paddr) { return  (paddr >> _set_index_offset) & _set_index_mask; }
	paddr_t _get_tag(paddr_t paddr) { return (paddr >> _tag_offset) & _tag_mask; }
};

}}