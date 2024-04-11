#pragma once
#include "stdafx.hpp"

#include "simulator/simulator.hpp"

namespace Arches { namespace Units {

//Assumes 64 byte cache lines. This prevents false sharing.
class alignas(64) UnitBase
{
public:
	Simulator* simulator{nullptr};
	uint64_t   unit_id{~0ull};

public:
	UnitBase() = default;
	virtual void clock_rise() = 0;
	virtual void clock_fall() = 0;
	virtual void reset() {};
};

}}