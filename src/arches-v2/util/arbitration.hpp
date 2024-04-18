#pragma once 
#include "stdafx.hpp"
#include "bit-manipulation.hpp"

template<typename MASK_T>
class Arbiter
{
public:
	virtual uint size() = 0;
	virtual MASK_T get_mask() = 0;
	virtual uint num_pending() = 0;
	virtual void add(uint index) = 0;
	virtual void remove(uint index) = 0;
	virtual uint get_index() = 0;
};


//Uses a nbit integer and BM2 extension to implement a computational and stoarge efficent arbiter for up to 64 clients
template<typename MASK_T>
class RoundRobinArbiter : public Arbiter<MASK_T>
{
protected:
	MASK_T _pending{0};
	uint16_t _priority_index{0};
	uint16_t _size;

public:
	RoundRobinArbiter(uint size = sizeof(MASK_T) * 8) : _size(size) { _assert(size <= sizeof(MASK_T) * 8); }

	uint size() override { return _size; }

	MASK_T get_mask() override
	{
		return _pending;
	}

	uint num_pending() override
	{
		return popcnt(_pending);
	}

	void add(uint index) override
	{
		_pending |= (MASK_T)1 << index;
	}

	void remove(uint index) override
	{
		_pending &= ~((MASK_T)1 << index);

		//Advance the priority index on remove so the grant index is now lowest priority
		if(index == _priority_index)
			if(++_priority_index >= _size)
				_priority_index = 0;
	}

	uint get_index() override
	{
		if(!_pending) 
			return ~0u;

		MASK_T rot_mask = rotr(_pending, _priority_index); //rotate the mask so the last index flag is in the 0 bit
		uint offset = ctz(rot_mask); //count the number of 0s till the next 1
		uint grant_index = (_priority_index + offset) % (sizeof(MASK_T) * 8); //grant the next set bit
		_priority_index = grant_index; //make the grant bit the highest priority bit so that it will continue to be granted until removed
		return grant_index; 
	}
};