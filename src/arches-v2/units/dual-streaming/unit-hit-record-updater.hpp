#pragma once

#include "units/unit-base.hpp"
#include "simulator/transactions.hpp"
#include "simulator/interconnects.hpp"
#include "units/unit-dram.hpp"



namespace Arches { namespace Units { namespace DualStreaming {

struct HitInfo {
	rtm::Hit hit;
	paddr_t hit_address;
};


struct HitRecordUpdaterRequest
{
	enum TYPE{
		STORE = 0,
		LOAD
	};
	TYPE type; //load or conditional store
	HitInfo hit_info;
	uint port;
};

/*!
* \brief The hit record updater receives hit records from TMs and update the record in the DRAM
*
* The hit record updater is divided into multiple channels (eg. number of channels in DRAM). It uses a casscadeCrossBar to process requests.
* Each channel maintains a local hit record cache. When a new hit record is requesting data from DRAM, we use the cache to store the record. When the cache is full, the TPs are blocked.
* Each channel processes one request from TPs in one cycle. Channels issue write request first (if available), then read request.
*/
class UnitHitRecordUpdater : public UnitBase {
public:
	struct Configuration
	{
		paddr_t hit_record_start;

		uint cache_size;
		uint associativity;

		uint num_tms;

		UnitMainMemoryBase* main_mem;
		uint                main_mem_port_offset{ 0 };
		uint                main_mem_port_stride{ 1 };
	};


	/*
	* Hit record cache in the updater. It is a set-associative cache used for storing requesting updates.
	*/
	class HitRecordCache {
	public:
		enum class State : uint8_t
		{
			CLOSEST_WRITE,
			CLOSEST_DRAM,
			LOAD_FROM_DRAM,
			EMPTY,
		};
	private:
		struct CacheElement {
			State state;
			HitInfo hit_info;
			uint lru = 0;
		};
		std::vector<CacheElement> cache;
		uint cache_size; // How many hit records
		uint associativity;
		uint num_set;
	public:
		HitRecordCache(uint _cache_size, uint _associativity): cache_size(_cache_size), associativity(_associativity), num_set(_cache_size / _associativity){
			cache.resize(cache_size, { State::EMPTY });
		}
		
		uint look_up(HitInfo hit_info) {
			uint set_id = hit_info.hit_address % num_set;
			uint start_index = set_id * associativity, end_index = start_index + associativity;
			assert(end_index <= cache_size);
			uint found_index = ~0;
			uint counter = 0;
			for (int i = start_index; i < end_index; i++) {
				if (cache[i].state != State::EMPTY && cache[i].hit_info.hit_address == hit_info.hit_address) {
					found_index = i;
					counter += 1;
				}
			}
			assert(counter < 2);
			if (!counter) return ~0;
			for (int i = start_index; i < end_index; i++) {
				if (cache[i].lru < cache[found_index].lru) cache[i].lru++;
			}
			cache[found_index].lru = 0;
			return found_index;
		}

		void compare_replace(HitInfo hit_info) {
			// This must come from a write request
			uint set_id = hit_info.hit_address % num_set;
			uint start_index = set_id * associativity, end_index = start_index + associativity;
			assert(end_index <= cache_size);
			uint found_index = ~0;
			for (int i = start_index; i < end_index; i++) {
				if (cache[i].state != State::EMPTY && cache[i].hit_info.hit_address == hit_info.hit_address) {
					found_index = i;
					if (hit_info.hit.t < cache[i].hit_info.hit.t) {
						if(cache[i].state == State::CLOSEST_DRAM) cache[i].state = State::CLOSEST_WRITE;
						cache[i].hit_info = hit_info;
					}
				}
			}
			assert(found_index != ~0);
		}

		void direct_replace(HitInfo hit_info, uint cache_index) {
			if (cache_index != ~0) {
				cache[cache_index].state = State::LOAD_FROM_DRAM;
				cache[cache_index].lru = 0;
				cache[cache_index].hit_info = hit_info;
			}
		}

		/*!
		* \brief Hit record from DRAM.
		*/
		uint process_dram_hit(HitInfo hit_info) {
			uint set_id = hit_info.hit_address % num_set;
			uint start_index = set_id * associativity, end_index = start_index + associativity;
			assert(end_index <= cache_size);
			uint found_index = ~0;
			for (int i = start_index; i < end_index; i++) {
				if (cache[i].state == State::LOAD_FROM_DRAM && cache[i].hit_info.hit_address == hit_info.hit_address) {
					found_index = i;
					if (cache[i].hit_info.hit.t < hit_info.hit.t) { // The record in cache is closer
						cache[i].state = State::CLOSEST_WRITE;
					}
					else {
						cache[i].hit_info = hit_info;
						cache[i].state = State::CLOSEST_DRAM;
					}
				}
			}
			assert(found_index != ~0);
			return found_index;
		}

		uint try_insert(HitInfo hit_info) {
			uint set_id = hit_info.hit_address % num_set;
			uint start_index = set_id * associativity, end_index = start_index + associativity;
			assert(end_index <= cache_size);
			uint found_index = ~0;
			for (int i = start_index; i < end_index; i++) {
				if (cache[i].state == State::EMPTY) {
					found_index = i;
					break;
				}
			}
			if (found_index == ~0) {
				uint lru = 0;
				for (int i = start_index; i < end_index; i++) {
					if ((cache[i].state == State::CLOSEST_DRAM || cache[i].state == State::CLOSEST_WRITE) && cache[i].lru >= lru) {
						found_index = i;
						lru = cache[i].lru;
					}
				}
			}
			return found_index;
		}

		HitInfo fetch_hit(uint cache_index) {
			return cache[cache_index].hit_info;
		}

		State get_state(uint cache_index) {
			return cache[cache_index].state;
		}

		void update_state(uint cache_index, State state) {
			cache[cache_index].state = state;
		}

		void check_cache() {
			std::map<int, int> counter;
			for (int i = 0; i < cache_size; i++) {
				if(cache[i].state != State::EMPTY) counter[cache[i].hit_info.hit_address] += 1;
			}
			for (auto [key, value] : counter) assert(value == 1);
		}

	};

	class HitRecordUpdaterRequestCrossBar : public CasscadedCrossBar<HitRecordUpdaterRequest> {
	public:
		HitRecordUpdaterRequestCrossBar(uint ports, uint channels, paddr_t hit_record_start_address) : CasscadedCrossBar<HitRecordUpdaterRequest>(ports, channels, channels), hit_record_start_address(hit_record_start_address){}
		uint get_sink(const HitRecordUpdaterRequest& request) override {
			paddr_t hit_record_address = request.hit_info.hit_address;
			int channel_id = calcDramAddr(hit_record_address).channel;
			return calcDramAddr(hit_record_address).channel;
		}
		paddr_t hit_record_start_address;
	};

	class ReturnCrossBar : public CasscadedCrossBar<MemoryReturn>
	{
	public:
		ReturnCrossBar(uint ports, uint banks) : CasscadedCrossBar<MemoryReturn>(banks, ports, banks) {}

		uint get_sink(const MemoryReturn& ret) override
		{
			return ret.port;
		}
	};

	struct Channel {
		HitRecordCache hit_record_cache; //128 records
		std::queue<MemoryRequest> read_queue; //128 * 7 = 1024 bit
		std::queue<MemoryRequest> write_queue;
		std::queue<MemoryReturn> return_queue;
		std::map<paddr_t, uint64_t> rsb_load_queue; // If the number of TMs is smaller than 64, otherwise we should replace UINT64 with std::vector
		std::map<std::pair<paddr_t, uint>, uint> rsb_counter;
	};

private:
	paddr_t hit_record_start_address;
	HitRecordUpdaterRequestCrossBar request_network;
	FIFOArray<MemoryReturn> return_network;

	std::vector<Channel> channels;
	UnitMemoryBase* main_memory;
	uint                main_mem_port_offset{ 0 };
	uint                main_mem_port_stride{ 1 };

	uint busy = 0;

	
private:
	void process_requests(uint channel_index);

	void process_returns(uint channel_index);

	void issue_requests(uint channel_index);

	void issue_returns(uint channel_index);

public:
	UnitHitRecordUpdater(Configuration config) : request_network(config.num_tms, NUM_DRAM_CHANNELS, config.hit_record_start), main_memory(config.main_mem), return_network(config.num_tms), main_mem_port_offset(config.main_mem_port_offset), main_mem_port_stride(config.main_mem_port_stride), hit_record_start_address(config.hit_record_start){
		for (int i = 0; i < NUM_DRAM_CHANNELS; i++) {
			channels.push_back({ HitRecordCache(config.cache_size, config.associativity) });

		}
		busy = 0;
	}


	bool request_port_write_valid(uint port_index)
	{
		return request_network.is_write_valid(port_index);
	}

	void write_request(const HitRecordUpdaterRequest& request, uint port_index)
	{
		request_network.write(request, port_index);
	}

	bool return_port_read_valid(uint port_index)
	{
		return return_network.is_read_valid(port_index);
	}

	const MemoryReturn& peek_return(uint port_index)
	{
		return return_network.peek(port_index);
	}

	const MemoryReturn read_return(uint port_index)
	{
		return return_network.read(port_index);
	}

	// Process requests from TMs and returns from DRAM
	void clock_rise() override;

	void clock_fall() override;
};

}
}
}