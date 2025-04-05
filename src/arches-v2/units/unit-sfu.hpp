#pragma once 
#include "stdafx.hpp"

#include "unit-base.hpp"
#include "isa/execution-base.hpp"

#include "util/arbitration.hpp"
#include "simulator/transactions.hpp"

namespace Arches { namespace Units {

class UnitSFU : public UnitBase
{
private:
	Cascade<SFURequest> request_crossbar;
	std::vector<LatencyFIFO<SFURequest>> piplines;
	class ReturnCascade : public Decascade<SFURequest>
	{
	public:
		ReturnCascade(uint banks, uint ports) : Decascade<SFURequest>(banks, ports) {}

		uint get_sink(const SFURequest& ret) override
		{
			return ret.port;
		}
	}
	return_crossbar;

public:
	UnitSFU(uint num_piplines, uint latency, uint cpi, uint num_clients) :
		request_crossbar(num_clients, num_piplines), return_crossbar(num_piplines, num_clients), piplines(num_piplines, {latency})
	{
	}

	//Should only be used on clock fall
	virtual bool request_port_write_valid(uint port_index)
	{
		return request_crossbar.is_write_valid(port_index);
	}

	virtual void write_request(const SFURequest& request)
	{
		request_crossbar.write(request, request.port);
	}

	//Should only be used on clock rise
	virtual bool return_port_read_valid(uint port_index)
	{
		return return_crossbar.is_read_valid(port_index);
	}

	virtual const SFURequest& peek_return(uint port_index)
	{
		return return_crossbar.peek(port_index);
	}

	virtual const SFURequest& read_return(uint port_index)
	{
		return return_crossbar.read(port_index);
	}

	void clock_rise() override
	{
		request_crossbar.clock();

		for(uint pipline_index = 0; pipline_index < piplines.size(); ++pipline_index)
		{
			if(request_crossbar.is_read_valid(pipline_index) && piplines[pipline_index].is_write_valid())
			{
				SFURequest req = request_crossbar.read(pipline_index);
				piplines[pipline_index].write(req);
			}

		}
	}

	void clock_fall() override
	{
		for(uint pipline_index = 0; pipline_index < piplines.size(); ++pipline_index)
		{
			if(piplines[pipline_index].is_read_valid() && return_crossbar.is_write_valid(pipline_index))
			{
				SFURequest ret = piplines[pipline_index].read();
				return_crossbar.write(ret, pipline_index);
			}
			piplines[pipline_index].clock();
		}

		return_crossbar.clock();
	}
};

}}