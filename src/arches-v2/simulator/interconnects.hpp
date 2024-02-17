#pragma once
#include "stdafx.hpp"

#include "util/arbitration.hpp"

namespace Arches {

template <typename T>
class Pipline
{
private:
	std::queue<T> _queue;
	std::vector<uint> _pipline_counters;

	uint _latency;
	uint _cpi;
	uint _last_stage_cpi;

public:
	Pipline(uint latency, uint cpi = 1)
	{
		uint pipline_stages = std::max((latency - 1) / cpi + 1, 1u);
		_pipline_counters.resize(pipline_stages, ~0u);

		_latency = latency;
		_cpi = cpi;

		_last_stage_cpi = (_latency - 1) % _cpi;
	}

	bool empty()
	{
		return _queue.empty();
	}

	uint lantecy()
	{
		return _latency;
	}

	void clock()
	{
		for(uint i = 0; i < (_pipline_counters.size() - 1); ++i)
			if(_pipline_counters[i] < _cpi)
				_pipline_counters[i]++;

		if(_pipline_counters.back() < _last_stage_cpi)
			_pipline_counters.back()++;

		for(uint i = (_pipline_counters.size() - 1); i != 0; --i)
		{
			if(_pipline_counters[i] == ~0u && _pipline_counters[i - 1] == _cpi)
			{
				_pipline_counters[i] = 0;
				_pipline_counters[i - 1] = ~0u;
			}
		}
	}

	bool is_write_valid()
	{
		return _pipline_counters.front() == ~0u;
	}

	void write(const T& entry)
	{
		assert(is_write_valid());
		_queue.push(entry);
		_pipline_counters.front() = 0;
	}

	bool is_read_valid()
	{
		return _pipline_counters.back() == _last_stage_cpi;
	}

	T& peek()
	{
		assert(is_read_valid());
		return _queue.front();
	}

	T read()
	{
		assert(is_read_valid());
		T ret = _queue.front();
		_queue.pop();
		_pipline_counters.back() = ~0u;
		return ret;
	}
};

template <typename T>
class FIFO
{
private:
	std::queue<T> _queue;
	size_t        _max_size;

public:
	FIFO(size_t max_size)
	{
		_max_size = max_size;
	}

	bool is_write_valid()
	{
		return _queue.size() < _max_size;
	}

	void write(const T& entry)
	{
		_queue.push(entry);
	}

	bool is_read_valid()
	{
		return !_queue.empty();
	}

	T& peek()
	{
		return _queue.front();
	}

	T read()
	{
		T ret = _queue.front();
		_queue.pop();
		return ret;
	}
};



template<typename T>
class InterconnectionNetwork
{
public:
	InterconnectionNetwork() {}

	//Owner interface
	virtual void clock() = 0;
	virtual uint num_sources() = 0;
	virtual uint num_sinks() = 0;

	//Sink Interface. Clock rise only. 
	virtual bool is_read_valid(uint sink_index) = 0;
	virtual const T& peek(uint sink_index) = 0;
	virtual const T read(uint sink_index) = 0;

	//Source Interface. Clock fall only.
	virtual bool is_write_valid(uint source_index) = 0;
	virtual void write(const T& transaction, uint source_index) = 0;
};



template<typename T>
class RegisterArray : public InterconnectionNetwork<T>
{
private:
	//Should be much more efficent since each line has one byte and they are alligne they will likely be on the same line
	//reads to the data only happen if pending flag is set and ack is done by clearing the pending bit
	std::vector<bool> _pending;
	std::vector<T>    _transactions;

public:
	RegisterArray(uint size) : _pending(size, false), _transactions(size) {}
	~RegisterArray() {}



	void clock()
	{

	}

	uint num_sources()
	{
		_pending.size();
	}

	uint num_sinks()
	{
		_pending.size();
	}



	bool is_read_valid(uint sink_index)
	{
		return _pending[sink_index];
	}

	const T& peek(uint sink_index)
	{
		return _transactions[sink_index];
	}

	const T read(uint sink_index)
	{
		_pending[sink_index] = false;
		return _transactions[sink_index];
	}



	bool is_write_valid(uint source_index)
	{
		return !_pending[source_index];
	}

	void write(const T& transaction, uint source_index)
	{
		_pending[source_index] = true;
		_transactions[source_index] = transaction;
	}
};

template<typename T>
class FIFOArray : public InterconnectionNetwork<T>
{
private:
	std::vector<uint8_t>       _sizes;
	std::vector<std::queue<T>> _fifos;
	uint8_t _max_size;

public:
	FIFOArray(uint size, uint depth = 8) : _sizes(size), _fifos(size), _max_size(depth) {}



	void clock() override
	{

	}

	uint num_sources() override
	{
		return _sizes.size();
	}

	uint num_sinks() override
	{
		return _sizes.size();
	}



	bool is_read_valid(uint sink_index) override
	{
		return _sizes[sink_index] > 0;
	}

	const T& peek(uint sink_index) override
	{
		assert(is_read_valid(sink_index));
		return _fifos[sink_index].front();
	}

	const T read(uint sink_index) override
	{
		const T t = peek(sink_index);
		_sizes[sink_index]--;
		_fifos[sink_index].pop();
		return t;
	}



	bool is_write_valid(uint source_index) override
	{
		return _sizes[source_index] < _max_size;
	}

	void write(const T& transaction, uint source_index) override
	{
		assert(is_write_valid(source_index));
		_sizes[source_index]++;
		_fifos[source_index].push(transaction);
	}
};


template<typename T>
class Casscade : public InterconnectionNetwork<T>
{
private:
	FIFOArray<T> _source_fifos;
	size_t                         _cascade_ratio;
	std::vector<RoundRobinArbiter> _arbiters;
	FIFOArray<T> _sink_fifos;

public:
	Casscade(uint sources, uint sinks, uint fifo_depth = 8) : 
		_source_fifos(sources, fifo_depth), 
		_cascade_ratio((sources + sinks - 1) / sinks),
		_arbiters(sinks, _cascade_ratio),
		_sink_fifos(sinks, fifo_depth) 
	{ 
		assert(sources >= sinks); 
	}

	void clock()
	{
		for(uint source_index = 0; source_index < _source_fifos.num_sinks(); ++source_index)
		{
			if(!_source_fifos.is_read_valid(source_index)) continue;

			uint cascade_index = source_index / _cascade_ratio;
			uint cascade_source_index = source_index % _cascade_ratio;

			_arbiters[cascade_index].add(cascade_source_index);
		}

		for(uint sink_index = 0; sink_index < _sink_fifos.num_sinks(); ++sink_index)
		{
			if(!_sink_fifos.is_write_valid(sink_index) || !_arbiters[sink_index].num_pending()) continue;

			uint cascade_source_index = _arbiters[sink_index].get_index();
			uint source_index = sink_index * _cascade_ratio + cascade_source_index;

			_sink_fifos.write(_source_fifos.read(source_index), sink_index);
			_arbiters[sink_index].remove(cascade_source_index);
		}
	}

	uint num_sources() override { return _source_fifos.num_sources(); }
	uint num_sinks() override { return _sink_fifos.num_sinks(); }

	bool is_read_valid(uint sink_index) override { return _sink_fifos.is_read_valid(sink_index); }
	const T& peek(uint sink_index) override { return _sink_fifos.peek(sink_index); }
	const T read(uint sink_index) override { return _sink_fifos.read(sink_index); }

	bool is_write_valid(uint source_index) override { return _source_fifos.is_write_valid(source_index); }
	void write(const T& transaction, uint source_index) override { _source_fifos.write(transaction, source_index); }
};

template<typename T>
class Decasscade : public InterconnectionNetwork<T>
{
private:
	FIFOArray<T> _source_fifos;
	FIFOArray<T> _sink_fifos;

public:
	Decasscade(uint sources, uint sinks) : 
		_source_fifos(sources), 
		_sink_fifos(sinks) 
	{ assert(source <= sinks); }

	virtual uint get_sink(const T& transaction) = 0;

	void clock()
	{
		for(uint source_index = 0; source_index < _source_fifos.num_sinks(); ++source_index)
		{
			if(!_source_fifos.is_read_valid(source_index)) continue;

			uint sink_index = get_sink(_source_fifos.peek(source_index), source_index);
			if(!_sink_fifos.is_write_valid(sink_index)) continue;

			_sink_fifos.write(_source_fifos.read(source_index), sink_index);
		}
	}

	uint num_sources() override { return _source_fifos.num_sources(); }
	uint num_sinks() override { return _sink_fifos.num_sinks(); }

	bool is_read_valid(uint sink_index) override { return _sink_fifos.is_read_valid(sink_index); }
	const T& peek(uint sink_index) override { return _sink_fifos.peek(sink_index); }
	const T read(uint sink_index) override { return _sink_fifos.read(sink_index); }

	bool is_write_valid(uint source_index) override { return _source_fifos.is_write_valid(source_index); }
	void write(const T& transaction, uint source_index) override { _source_fifos.write(transaction, source_index); }
};

template<typename T>
class CrossBar : public InterconnectionNetwork<T>
{
private:
	FIFOArray<T> _source_fifos;
	std::vector<RoundRobinArbiter> _arbiters;
	FIFOArray<T> _sink_fifos;
	std::vector<uint> already_arrange;

public:
	CrossBar(uint sources, uint sinks) : _source_fifos(sources), _arbiters(sinks, sources), _sink_fifos(sinks), already_arrange(sources, ~0u) {}

	virtual uint get_sink(const T& transaction) = 0;

	void clock()
	{
		for (uint source_index = 0; source_index < _source_fifos.num_sinks(); ++source_index)
		{
			if (!_source_fifos.is_read_valid(source_index) || already_arrange[source_index] == ~0u) continue;
			uint sink_index = get_sink(_source_fifos.peek(source_index));
			assert((_arbiters[sink_index]._pending & (1ull << source_index)) == 0);
			already_arrange[source_index] = sink_index;
			_arbiters[sink_index].add(source_index);
		}

		for (uint sink_index = 0; sink_index < _sink_fifos.num_sinks(); ++sink_index)
		{
			if (!_sink_fifos.is_write_valid(sink_index) || _arbiters[sink_index].num_pending() == 0) continue;
			uint source_index = _arbiters[sink_index].get_index();
			assert(already_arrange[source_index] == sink_index);
			_sink_fifos.write(_source_fifos.read(source_index), sink_index);
			already_arrange[source_index] = ~0u;
			_arbiters[sink_index].remove(source_index);
		}
	}

	uint num_sources() override { return _source_fifos.num_sources(); }
	uint num_sinks() override { return _sink_fifos.num_sinks(); }

	bool is_read_valid(uint sink_index) override { return _sink_fifos.is_read_valid(sink_index); }
	const T& peek(uint sink_index) override { return _sink_fifos.peek(sink_index); }
	const T read(uint sink_index) override { return _sink_fifos.read(sink_index); }

	bool is_write_valid(uint source_index) override { return _source_fifos.is_write_valid(source_index); }
	void write(const T& transaction, uint source_index) override { _source_fifos.write(transaction, source_index); }
};

template<typename T>
class CasscadedCrossBar : public InterconnectionNetwork<T>
{
private:
	FIFOArray<T> _source_fifos;
	size_t                         _input_cascade_ratio;
	std::vector<RoundRobinArbiter> _cascade_arbiters;
	std::vector<RoundRobinArbiter> _crossbar_arbiters;
	size_t                         _output_cascade_ratio;
	std::vector<uint>			   _source_to_sink;
	FIFOArray<T> _sink_fifos;

public:
	CasscadedCrossBar(uint sources, uint sinks, uint crossbar_width, uint source_fifo_depth = 16, uint sink_fifo_depth = 32) :
		_source_fifos(sources, source_fifo_depth),
		_input_cascade_ratio((sources + crossbar_width - 1) / crossbar_width),
		_cascade_arbiters(crossbar_width, _input_cascade_ratio),
		_crossbar_arbiters(crossbar_width, crossbar_width),
		_output_cascade_ratio((sinks + crossbar_width - 1) / crossbar_width),
		_sink_fifos(sinks, sink_fifo_depth),
		_source_to_sink(sources, ~0u)
	{}

	virtual uint get_sink(const T& transaction) = 0;

	void clock()
	{
		for (uint source_index = 0; source_index < _source_fifos.num_sinks(); ++source_index)
		{
			if (!_source_fifos.is_read_valid(source_index) || _source_to_sink[source_index] != ~0u) continue;

			uint sink_index = get_sink(_source_fifos.peek(source_index));
			_source_to_sink[source_index] = sink_index;

			uint cascade_index = source_index / _input_cascade_ratio;
			uint cascade_source_index = source_index % _input_cascade_ratio;

			_cascade_arbiters[cascade_index].add(cascade_source_index);
		}

		for (uint cascade_index = 0; cascade_index < _cascade_arbiters.size(); ++cascade_index)
		{
			if (!_cascade_arbiters[cascade_index].num_pending()) continue;

			uint cascade_source_index = _cascade_arbiters[cascade_index].get_index();
			uint source_index = cascade_index * _input_cascade_ratio + cascade_source_index;
			uint sink_index = _source_to_sink[source_index];
			uint crossbar_index = sink_index / _output_cascade_ratio;

			_crossbar_arbiters[crossbar_index].add(cascade_index);
		}

		for (uint crossbar_index = 0; crossbar_index < _crossbar_arbiters.size(); ++crossbar_index)
		{
			if (!_crossbar_arbiters[crossbar_index].num_pending()) continue;

			uint cascade_index = _crossbar_arbiters[crossbar_index].get_index();
			uint cascade_source_index = _cascade_arbiters[cascade_index].get_index();
			uint source_index = cascade_index * _input_cascade_ratio + cascade_source_index;

			if (!_sink_fifos.is_write_valid(crossbar_index)) continue;

			uint sink_index = get_sink(_source_fifos.peek(source_index));
			_source_to_sink[source_index] = ~0u;

			_crossbar_arbiters[crossbar_index].remove(cascade_index);
			_cascade_arbiters[cascade_index].remove(cascade_source_index);
			_sink_fifos.write(_source_fifos.read(source_index), sink_index);
		}
	}

	uint num_sources() override { return _source_fifos.num_sources(); }
	uint num_sinks() override { return _sink_fifos.num_sinks(); }

	bool is_read_valid(uint sink_index) override { return _sink_fifos.is_read_valid(sink_index); }
	const T& peek(uint sink_index) override { return _sink_fifos.peek(sink_index); }
	const T read(uint sink_index) override { return _sink_fifos.read(sink_index); }

	bool is_write_valid(uint source_index) override { return _source_fifos.is_write_valid(source_index); }
	void write(const T& transaction, uint source_index) override { _source_fifos.write(transaction, source_index); }
};

}