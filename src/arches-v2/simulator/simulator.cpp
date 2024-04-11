#include "simulator.hpp"

#include "units/unit-base.hpp"

namespace Arches {

void Simulator::register_unit(Units::UnitBase * unit)
{
	unit->unit_id = _units.size();
	_units.push_back(unit);
	_unit_groups.back().end++;
	unit->simulator = this;
}

void Simulator::new_unit_group()
{
	_unit_groups.emplace_back(static_cast<uint>(_units.size()), static_cast<uint>(_units.size()));
}

#ifndef _DEBUG
#define USE_TBB
#endif

#ifdef USE_TBB
//tbb controled block ranges
#define UNIT_LOOP tbb::parallel_for(tbb::blocked_range<uint>(0, _units.size()), [&](tbb::blocked_range<uint> r) { for(uint i = r.begin(); i < r.end(); ++i) {
#define UNIT_LOOP_END }});

//custom block ranges
#define UNIT_LOOP tbb::parallel_for(tbb::blocked_range<uint>(0, _unit_groups.size()), [&](tbb::blocked_range<uint> r) { for(uint j = r.begin(); j < r.end(); ++j) { for(uint i = _unit_groups[j].start; i < _unit_groups[j].end; ++i) {
#define UNIT_LOOP_END }}});

#else
#define UNIT_LOOP for(uint i = 0; i < _units.size(); ++i) {
#define UNIT_LOOP_END }
#endif

void Simulator::_clock_rise()
{
	UNIT_LOOP
		_units[i]->clock_rise();
	UNIT_LOOP_END
}

void Simulator::_clock_fall()
{
	UNIT_LOOP
		_units[i]->clock_fall();
	UNIT_LOOP_END
}

void Simulator::execute(uint delta, std::function<void()> interval_logger)
{
	for(auto& unit : _units)
		unit->reset();

	do
	{
		_clock_rise();
		_clock_fall();

		current_cycle++;
		if(delta != 0 && current_cycle % delta == 0)
			interval_logger();
	}
	while(units_executing > 0);
}

}