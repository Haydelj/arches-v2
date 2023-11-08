#pragma once
#include "stdafx.hpp"

#include "simulator/simulator.hpp"

namespace Arches { namespace Units {

class UnitBase
{
public:
	UnitBase() = default;

	Simulator* simulator{nullptr};
	uint64_t   unit_id{~0ull};
	virtual void clock_rise() = 0;
	virtual void clock_fall() = 0;
};

}}