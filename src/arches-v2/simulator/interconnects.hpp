#pragma once
#include "stdafx.hpp"

#include "util/arbitration.hpp"
#include "util/alignment-allocator.hpp"
#include "util/bit-manipulation.hpp"

namespace Arches {

template <typename T>
class Pipline
{
private:
	std::queue<T> _queue;
	std::vector<uint64_t> _pipline_state;
	uint _latency;

public:
	Pipline(uint latency)
	{
		uint pipline_regs = latency;
		_pipline_state.resize((pipline_regs + 63) / 64);
		_latency = latency;
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
		_pipline_state[0] = (_pipline_state[0] >> 1) | generate_nbit_mask(ctz(~_pipline_state[0]));
		for(uint i = 1; i < _pipline_state.size(); ++i)
		{
			if(!(_pipline_state[i - 1] & (0x1ull << 63)) && _pipline_state[i] & 0x1)
			{
				_pipline_state[i - 1] |= (0x1ull << 63);
				_pipline_state[i] &= ~(0x1ull);
			}
			_pipline_state[i] = (_pipline_state[i] >> 1) | generate_nbit_mask(ctz(~_pipline_state[i]));
		}
	}

	bool is_write_valid()
	{
		return !((_pipline_state.back() >> ((_latency - 1) & 0x3f)) & 0x1);
	}

	void write(const T& entry)
	{
		_assert(is_write_valid());
		_pipline_state.back() |= (0x1ull << ((_latency - 1) & 0x3f));
		_queue.push(entry);
	}

	bool is_read_valid()
	{
		return _pipline_state[0] & 0x1;
	}

	T& peek()
	{
		_assert(is_read_valid());
		return _queue.front();
	}

	T read()
	{
		_assert(is_read_valid());
		_pipline_state[0] &= ~0x1ull;
		T ret = _queue.front();
		_queue.pop();
		return ret;
	}
};

template <typename T>
class FIFO
{
private:
	std::queue<T> _queue;
	uint          _max_size;

public:
	FIFO(uint max_size)
	{
		_max_size = max_size;
	}

	bool is_write_valid()
	{
		return _queue.size() < _max_size;
	}

	void write(const T& entry)
	{
		_assert(is_write_valid());
		_queue.push(entry);
	}

	bool is_read_valid()
	{
		return !_queue.empty();
	}

	T& peek()
	{
		_assert(is_read_valid());
		return _queue.front();
	}

	T read()
	{
		const T t = peek();
		_queue.pop();
		return t;
	}
};

template<typename T, uint MAX_SIZE = 64 * 4>
class alignas(64) Interconnect
{
protected:
	uint8_t _input_pending[MAX_SIZE];
	uint8_t _output_pending[MAX_SIZE];
	const uint _num_sources;
	const uint _num_sinks;

public:
	Interconnect(uint sources, uint sinks) : _num_sources(sources), _num_sinks(sinks)
	{
		_assert(sources <= MAX_SIZE);
		_assert(sinks <= MAX_SIZE);
		for(uint i = 0; i < MAX_SIZE; ++i)
		{
			_input_pending[i] = 0;
			_output_pending[i] = 0;
		}
	}

	uint num_sources() const { return _num_sources; }
	uint num_sinks() const { return _num_sinks; }

	//Owner interface
	virtual void clock() = 0;

	//Sink Interface. Clock rise only. 
	bool is_read_valid(uint sink_index) const
	{ 
		_assert(sink_index < _num_sinks);
		return _output_pending[sink_index]; 
	}
	virtual const T& peek(uint sink_index) = 0;
	virtual const T read(uint sink_index) = 0;

	//Source Interface. Clock fall only.
	bool is_write_valid(uint source_index) const
	{
		_assert(source_index < _num_sources);
		return !_input_pending[source_index];
	}
	virtual void write(const T& transaction, uint source_index) = 0;
};

template<typename T>
class RegisterArray : public Interconnect<T>
{
private:
	std::vector<T> _transactions;

public:
	RegisterArray(uint size) : Interconnect<T>(size, size), _transactions(size) {}

	void clock() override
	{
		//copy states from input to output
		for(uint i = 0; i < _transactions.size(); ++i)
			Interconnect<T>::_output_pending[i] = Interconnect<T>::_input_pending[i];
	}

	const T& peek(uint sink_index) override
	{
		_assert(Interconnect<T>::is_read_valid(sink_index));
		return _transactions[sink_index];
	}

	const T read(uint sink_index) override
	{
		_assert(Interconnect<T>::is_read_valid(sink_index));
		Interconnect<T>::_output_pending[sink_index] = 0;
		Interconnect<T>::_input_pending[sink_index] = 0;
		return _transactions[sink_index];
	}

	void write(const T& transaction, uint source_index) override
	{
		_assert(Interconnect<T>::is_write_valid(source_index));
		Interconnect<T>::_input_pending[source_index] = 1;
		_transactions[source_index] = transaction;
	}
};

constexpr uint default_fifo_depth = 1;

template<typename T>
class FIFOArray : public Interconnect<T>
{
private:
	std::vector<std::queue<T>> _fifos;
	uint _fifo_depth;

public:
	FIFOArray(uint size, uint fifo_depth = default_fifo_depth) : Interconnect<T>(size, size), _fifos(size), _fifo_depth(fifo_depth)
	{
	}

	void clock() override
	{
		//copy states from input to output
		for(uint i = 0; i < _fifos.size(); ++i)
		{
			Interconnect<T>::_input_pending[i] = _fifos[i].size() >= _fifo_depth;
			Interconnect<T>::_output_pending[i] = _fifos[i].size() > 0;
		}
	}

	const T& peek(uint sink_index) override
	{
		_assert(Interconnect<T>::is_read_valid(sink_index));
		return _fifos[sink_index].front();
	}

	const T read(uint sink_index) override
	{
		const T t = peek(sink_index);
		Interconnect<T>::_output_pending[sink_index] = 0;
		_fifos[sink_index].pop();
		return t;
	}

	void write(const T& transaction, uint source_index) override
	{
		_assert(Interconnect<T>::is_write_valid(source_index));
		Interconnect<T>::_input_pending[source_index] = 1;
		_fifos[source_index].push(transaction);
	}
};

template <typename T>
using I = Interconnect<T>;

template<typename T>
class BufferedInterconnect : public I<T>
{
protected:
	std::vector<std::queue<T>> _source_fifos;
	std::vector<std::queue<T>> _sink_fifos;
	const uint _source_fifo_depth;
	const uint _sink_fifo_depth;

public:
	BufferedInterconnect(uint sources, uint sinks, uint source_fifo_depth = default_fifo_depth, uint sink_fifo_depth = default_fifo_depth) : 
		I<T>(sources, sinks), _source_fifos(sources), _sink_fifos(sinks), _source_fifo_depth(source_fifo_depth), _sink_fifo_depth(sink_fifo_depth) {}

	virtual void clock() override
	{
		for(uint i = 0; i < _source_fifos.size(); ++i)
			I<T>::_input_pending[i] = _source_fifos[i].size() >= _source_fifo_depth;

		for(uint i = 0; i < _sink_fifos.size(); ++i)
			I<T>::_output_pending[i] = _sink_fifos[i].size() > 0;
	}

	const T& peek(uint sink_index) override
	{
		_assert(I<T>::is_read_valid(sink_index));
		return _sink_fifos[sink_index].front();
	}

	const T read(uint sink_index) override
	{
		const T t = peek(sink_index);
		I<T>::_output_pending[sink_index] = 0;
		_sink_fifos[sink_index].pop();
		return t;
	}

	void write(const T& transaction, uint source_index) override
	{
		_assert(Interconnect<T>::is_write_valid(source_index));
		I<T>::_input_pending[source_index] = 1;
		_source_fifos[source_index].push(transaction);
	}
};

template <typename T>
using BI = BufferedInterconnect<T>;

template<typename T, typename ARB = RoundRobinArbiter<uint128_t>>
class Cascade : public BI<T>
{
private:
	size_t _cascade_ratio;
	std::vector<ARB> _arbiters;

public:
	Cascade(uint sources, uint sinks, uint source_fifo_depth = default_fifo_depth, uint sink_fifo_depth = default_fifo_depth) :
		BI<T>(sources, sinks, source_fifo_depth, sink_fifo_depth),
		_cascade_ratio((sources + sinks - 1) / sinks),
		_arbiters(sinks, _cascade_ratio)
	{
		_assert(sources >= sinks);
	}

	void clock() override
	{
		for(uint source_index = 0; source_index < I<T>::num_sources(); ++source_index)
		{
			if(BI<T>::_source_fifos[source_index].empty()) continue;

			uint cascade_index = source_index / _cascade_ratio;
			uint cascade_source_index = source_index % _cascade_ratio;

			_arbiters[cascade_index].add(cascade_source_index);
		}

		for(uint sink_index = 0; sink_index < I<T>::num_sinks(); ++sink_index)
		{
			if((BI<T>::_sink_fifos[sink_index].size() >= BI<T>::_sink_fifo_depth) || !_arbiters[sink_index].num_pending()) continue;

			uint cascade_source_index = _arbiters[sink_index].get_index();
			uint source_index = sink_index * _cascade_ratio + cascade_source_index;

			BI<T>::_sink_fifos[sink_index].push(BI<T>::_source_fifos[source_index].front());
			BI<T>::_source_fifos[source_index].pop();
			_arbiters[sink_index].remove(cascade_source_index);
		}

		BI<T>::clock();
	}
};

template<typename T>
class Decascade : public BI<T>
{
private:
	size_t _cascade_ratio;

public:
	Decascade(uint sources, uint sinks, uint source_fifo_depth = default_fifo_depth, uint sink_fifo_depth = default_fifo_depth) :
		BI<T>(sources, sinks, source_fifo_depth, sink_fifo_depth),
		_cascade_ratio((sinks + sources - 1) / sources)
	{
		_assert(sources <= sinks);
	}

	virtual uint get_sink(const T& transaction) = 0;

	void clock() override
	{
		for(uint source_index = 0; source_index < I<T>::num_sources(); ++source_index)
		{
			if(BI<T>::_source_fifos[source_index].empty()) continue;

			uint sink_index = get_sink(BI<T>::_source_fifos[source_index].front());
			_assert(sink_index / _cascade_ratio == source_index);

			if(BI<T>::_sink_fifos[sink_index].size() >= BI<T>::_sink_fifo_depth) continue;
			BI<T>::_sink_fifos[sink_index].push(BI<T>::_source_fifos[source_index].front());
			BI<T>::_source_fifos[source_index].pop();
		}

		BI<T>::clock();
	}
};

template<typename T, typename ARB = RoundRobinArbiter<uint64_t>>
class CrossBar : public BI<T>
{
private:
	std::vector<ARB> _arbiters;

public:
	CrossBar(uint sources, uint sinks, uint source_fifo_depth = default_fifo_depth, uint sink_fifo_depth = default_fifo_depth) :
		BI<T>(sources, sinks, source_fifo_depth, sink_fifo_depth),
		_arbiters(sinks, sources)
	{
	}

	virtual uint get_sink(const T& transaction) = 0;

	void clock() override
	{
		for(uint source_index = 0; source_index < Interconnect<T>::num_sources(); ++source_index)
		{
			if(BI<T>::_source_fifos[source_index].empty()) continue;

			uint sink_index = get_sink(BI<T>::_source_fifos[source_index].front());
			_arbiters[sink_index].add(source_index);
		}

		for(uint sink_index = 0; sink_index < Interconnect<T>::num_sinks(); ++sink_index)
		{
			if((BI<T>::_sink_fifos[sink_index].size() >= BI<T>::_sink_fifo_depth) || !_arbiters[sink_index].num_pending()) continue;

			uint source_index = _arbiters[sink_index].get_index();

			_arbiters[sink_index].remove(source_index);
			BI<T>::_sink_fifos[sink_index].push(BI<T>::_source_fifos[source_index].front());
			BI<T>::_source_fifos[source_index].pop();
		}

		BI<T>::clock();
	}
};


template<typename T, typename ARB = RoundRobinArbiter<uint64_t>>
class CasscadedCrossBar : public BI<T>
{
protected:
	uint _source_crossbar_width, _sink_crossbar_width;
	size_t _input_cascade_ratio;
	std::vector<RoundRobinArbiter<uint64_t>> _cascade_arbiters;
	std::vector<ARB> _crossbar_arbiters;
	size_t _output_cascade_ratio;

public:
	CasscadedCrossBar(uint sources, uint sinks, uint source_crossbar_width = 64, uint sink_crossbar_width = 64, uint source_fifo_depth = default_fifo_depth, uint sink_fifo_depth = default_fifo_depth) :
		BI<T>(sources, sinks, source_fifo_depth, sink_fifo_depth),
		_source_crossbar_width(std::min(source_crossbar_width, sources)),
		_sink_crossbar_width(std::min(sink_crossbar_width, sinks)),
		_input_cascade_ratio((sources + _source_crossbar_width - 1) / _source_crossbar_width),
		_cascade_arbiters(_source_crossbar_width, _input_cascade_ratio),
		_crossbar_arbiters(_sink_crossbar_width, _source_crossbar_width),
		_output_cascade_ratio((sinks + _sink_crossbar_width - 1) / _sink_crossbar_width)
	{
		_assert(sources >= _source_crossbar_width);
		_assert(sinks >= _sink_crossbar_width);
	}

	virtual uint get_sink(const T& transaction) = 0;

	void clock() override
	{
		for(uint source_index = 0; source_index < Interconnect<T>::num_sources(); ++source_index)
		{
			if(BI<T>::_source_fifos[source_index].empty()) continue;

			uint cascade_index = source_index / _input_cascade_ratio;
			uint cascade_source_index = source_index % _input_cascade_ratio;

			_cascade_arbiters[cascade_index].add(cascade_source_index);
		}

		for(uint cascade_index = 0; cascade_index < _cascade_arbiters.size(); ++cascade_index)
		{
			if(!_cascade_arbiters[cascade_index].num_pending()) continue;

			uint cascade_source_index = _cascade_arbiters[cascade_index].get_index();
			uint source_index = cascade_index * _input_cascade_ratio + cascade_source_index;
			uint sink_index = get_sink(BI<T>::_source_fifos[source_index].front());
			uint crossbar_index = sink_index / _output_cascade_ratio;

			_crossbar_arbiters[crossbar_index].add(cascade_index);
		}

		for(uint crossbar_index = 0; crossbar_index < _crossbar_arbiters.size(); ++crossbar_index)
		{
			if(!_crossbar_arbiters[crossbar_index].num_pending()) continue;

			uint cascade_index = _crossbar_arbiters[crossbar_index].get_index();
			uint cascade_source_index = _cascade_arbiters[cascade_index].get_index();
			uint source_index = cascade_index * _input_cascade_ratio + cascade_source_index;
			uint sink_index = get_sink(BI<T>::_source_fifos[source_index].front());

			if(BI<T>::_sink_fifos[sink_index].size() >= BI<T>::_sink_fifo_depth) continue;

			_crossbar_arbiters[crossbar_index].remove(cascade_index);
			_cascade_arbiters[cascade_index].remove(cascade_source_index);
			BI<T>::_sink_fifos[sink_index].push(BI<T>::_source_fifos[source_index].front());
			BI<T>::_source_fifos[source_index].pop();
		}

		BI<T>::clock();
	}
};

}