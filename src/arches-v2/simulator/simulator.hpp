#pragma once
#include "stdafx.hpp"

#if 1
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

	void execute();
};

}
#else

#include <thread>
#include <barrier>

namespace Arches
{

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
			uint size;

			UnitGroup() = default;
			UnitGroup(uint start, uint size) : start(start), size(size) {}
		};

		std::vector<UnitGroup> _unit_groups;
		std::vector<Units::UnitBase*> _units;
		std::atomic_bool              _done{false};

		std::atomic_uint _rise_index{0};
		std::atomic_uint _fall_index{0};

		std::vector<std::thread*> _thread_pool;
		std::barrier<std::_No_completion_function> _barrier;

	public:
		cycles_t current_cycle{~0ull};

		Simulator(uint num_threads);

		void register_unit(Units::UnitBase* unit);
		void new_unit_group();

		void _clock_rise();
		void _clock_fall();

		void _thread_work(uint thread_id);

		void execute();

	};

}
#endif