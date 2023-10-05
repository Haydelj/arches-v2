#pragma once

#include "../stdafx.hpp"

#include "unit-base.hpp"
#include "../simulator/interconnects.hpp"
#include "../simulator/transactions.hpp"

namespace Arches { namespace Units {

class UnitMemoryBase : public UnitBase
{
protected:
	class MemoryRequestCrossBar : public CasscadedCrossBar<MemoryRequest>
	{
	private:
		uint64_t mask;

	public:
		MemoryRequestCrossBar(uint ports, uint banks, uint64_t bank_select_mask) : mask(bank_select_mask), CasscadedCrossBar<MemoryRequest>(ports, banks, banks) {}

		uint get_sink(const MemoryRequest& request) override
		{
			uint bank = pext(request.paddr, mask);
			assert(bank < num_sinks());
			return bank;
		}
	};

	class MemoryReturnCrossBar : public CasscadedCrossBar<MemoryReturn>
	{
	public:
		MemoryReturnCrossBar(uint ports, uint banks) : CasscadedCrossBar<MemoryReturn>(banks, ports, banks) {}

		uint get_sink(const MemoryReturn& ret) override
		{
			return ret.port;
		}
	};

public:
	UnitMemoryBase() = default;

	//Should only be used on clock fall
	virtual bool request_port_write_valid(uint port_index) = 0;
	virtual void write_request(const MemoryRequest& request, uint port_index) = 0;

	//Should only be used on clock rise
	virtual bool return_port_read_valid(uint port_index) = 0;
	virtual const MemoryReturn& peek_return(uint port_index) = 0;
	virtual const MemoryReturn read_return(uint port_index) = 0;
};

class MemoryMap
{
private:
	std::vector<std::pair<Arches::paddr_t, uint>> ranges;

public:
	struct MemoryMapping
	{
		Arches::Units::UnitMemoryBase* unit;
		uint16_t        port_index; //port index within unit
		uint16_t        num_ports; //number of ports reserved for this mapping
		uint16_t        port_id; //provides unqie ids for all ports

		bool operator==(const MemoryMapping& other) const
		{
			return unit == other.unit && port_index == other.port_index && num_ports == other.num_ports;
		}
	};

	std::vector<MemoryMapping> mappings;
	uint total_ports{0};

	void add_unit(Arches::paddr_t paddr, Arches::Units::UnitMemoryBase* unit, uint port_id, uint num_ports)
	{
		if(unit == nullptr)
		{
			num_ports = 0;
			port_id = 0;
		}

		uint i = 0;
		for(; i < ranges.size(); ++i)
			if(paddr < ranges[i].first) break;

		ranges.insert(ranges.begin() + i, {paddr, ~0u});

		if(unit == nullptr) return;

		MemoryMapping mapping = {unit, port_id, num_ports, 0};

		uint j;
		for(j = 0; j < mappings.size(); ++j)
			if(mapping == mappings[j]) break;

		if(j == mappings.size()) mappings.push_back(mapping);

		ranges[i].second = j;

		total_ports = 0;
		for(j = 0; j < mappings.size(); ++j)
		{
			mappings[j].port_id = total_ports;
			total_ports += mappings[j].num_ports;
		}
	}

	uint get_mapping_index(Arches::paddr_t paddr)
	{
		uint start = 0;
		uint end = ranges.size();
		while((start + 1) != end)
		{
			uint middle = (start + end) / 2;
			if(paddr >= ranges[middle].first) start = middle;
			else                              end = middle;
		}

		return ranges[start].second;
	}

	uint get_mapping_index_for_unique_port_index(uint unique_port_index)
	{
		uint start = 0;
		uint end = mappings.size();
		while((start + 1) != end)
		{
			uint middle = (start + end) / 2;
			if(unique_port_index >= mappings[middle].port_id) start = middle;
			else                                                     end = middle;
		}

		return start;
	}
};

}}