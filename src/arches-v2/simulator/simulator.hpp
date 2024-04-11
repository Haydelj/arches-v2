#pragma once
#include "stdafx.hpp"

namespace Arches {

namespace Units
{
	class UnitBase;
}

class Simulator
{
private:
	struct UnitGroup
	{
		uint start;
		uint end;

		UnitGroup() = default;
		UnitGroup(uint start, uint end) : start(start), end(end) {}
	};

	std::vector<UnitGroup> _unit_groups;
	std::vector<Units::UnitBase*> _units;

public:
	std::atomic_uint units_executing{0};
	cycles_t current_cycle{0};

	Simulator() { _unit_groups.emplace_back(0u, 0u); }

	void register_unit(Units::UnitBase* unit);
	void new_unit_group();

	void _clock_rise();
	void _clock_fall();

	void execute(uint epsilon = 0, std::function<void()> interval_logger = nullptr);
};

}