#pragma once 
#include "unit-hit-record-updater.hpp"

namespace Arches {
namespace Units {
namespace DualStreaming {

void UnitHitRecordUpdater::clock_rise() {
	request_network.clock();
	for (int i = 0; i < channels.size(); i++) {
		process_requests(i);
		process_returns(i);
		if (!channels[i].read_queue.empty() || !channels[i].return_queue.empty() || !channels[i].write_queue.empty()) {
			if (busy == 0) simulator->units_executing++;
			busy |= (1 << i);
		}
	}
}

void UnitHitRecordUpdater::clock_fall() {
	for (int i = 0; i < channels.size(); i++) {
		issue_requests(i);
		issue_returns(i);
		if (channels[i].read_queue.empty() && channels[i].return_queue.empty() && channels[i].write_queue.empty()) {
			if (busy == (1 << i)) simulator->units_executing--;
			busy &= ~(1 << i);
		}
	}
	return_network.clock();
}

void UnitHitRecordUpdater::process_requests(uint channel_index) {
	if (!request_network.is_read_valid(channel_index)) return;
	Channel& channel = channels[channel_index];
	const HitRecordUpdaterRequest rsb_req = request_network.peek(channel_index);

	uint cache_index = channel.hit_record_cache.look_up(rsb_req.hit_info);
	HitRecordCache::State state = HitRecordCache::State::EMPTY;
	if (cache_index != ~0) state = channel.hit_record_cache.get_state(cache_index);

	if (rsb_req.type == HitRecordUpdaterRequest::TYPE::LOAD) {
		assert(fabs(rsb_req.hit_info.hit.t - T_MAX) < 1e-6); // For read request, hit.t = T_MAX
		if (state == HitRecordCache::State::CLOSEST_DRAM || state == HitRecordCache::State::CLOSEST_WRITE) {
			// We do not need to visit DRAM because we already have the closest hit
			HitInfo hit_info = channel.hit_record_cache.fetch_hit(cache_index);
			MemoryReturn ret;
			ret.port = rsb_req.port;
			ret.size = sizeof(rtm::Hit);
			std::memcpy(ret.data, &hit_info.hit, sizeof(rtm::Hit));
			ret.paddr = hit_info.hit_address;
			channel.return_queue.push(ret);
			request_network.read(channel_index);
		}
		else {
			if (cache_index != ~0) {
				// There is already a load request for this hit
				assert(rsb_req.port < 64);
				channel.rsb_load_queue[rsb_req.hit_info.hit_address] |= (1ull << rsb_req.port);
				channel.rsb_counter[{rsb_req.hit_info.hit_address, rsb_req.port}]++;
				request_network.read(channel_index);
			}
			else {
				// We need to insert the request to the cache
				uint replace_index = channel.hit_record_cache.try_insert(rsb_req.hit_info);
				if (replace_index != ~0) {
					HitInfo hit_info_replaced = channel.hit_record_cache.fetch_hit(replace_index);
					HitRecordCache::State state_replaced = channel.hit_record_cache.get_state(replace_index);

					if (state_replaced == HitRecordCache::State::CLOSEST_WRITE) {
						// We need to write this data to DRAM
						MemoryRequest write_req;
						write_req.type = MemoryRequest::Type::STORE;
						write_req.size = sizeof(rtm::Hit);
						write_req.write_mask = generate_nbit_mask(sizeof(rtm::Hit));
						write_req.paddr = hit_info_replaced.hit_address;
						std::memcpy(write_req.data, &hit_info_replaced.hit, sizeof(rtm::Hit));
						channel.write_queue.push(write_req);
					}
					channel.hit_record_cache.direct_replace(rsb_req.hit_info, replace_index);

					MemoryRequest load_req;
					load_req.type = MemoryRequest::Type::LOAD;
					load_req.size = sizeof(rtm::Hit);
					load_req.paddr = rsb_req.hit_info.hit_address;
					channel.read_queue.push(load_req);
					
					channel.rsb_load_queue[rsb_req.hit_info.hit_address] |= (1ull << rsb_req.port);
					channel.rsb_counter[{rsb_req.hit_info.hit_address, rsb_req.port}]++;
					request_network.read(channel_index);
				}
			}

		}
	}
	else if (rsb_req.type == HitRecordUpdaterRequest::TYPE::STORE) {
		if (cache_index != ~0) {
			// There is a record in the cache
			channel.hit_record_cache.compare_replace(rsb_req.hit_info);
			request_network.read(channel_index);
		}
		else {
			// It is a new record
			uint replace_index = channel.hit_record_cache.try_insert(rsb_req.hit_info);
			if (replace_index != ~0) {
				HitInfo hit_info_replaced = channel.hit_record_cache.fetch_hit(replace_index);
				HitRecordCache::State state_replaced = channel.hit_record_cache.get_state(replace_index);

				if (state_replaced == HitRecordCache::State::CLOSEST_WRITE) {
					// We need to write this data to DRAM
					MemoryRequest write_req;
					write_req.type = MemoryRequest::Type::STORE;
					write_req.size = sizeof(rtm::Hit);
					write_req.write_mask = generate_nbit_mask(sizeof(rtm::Hit));
					write_req.paddr = hit_info_replaced.hit_address;
					std::memcpy(write_req.data, &hit_info_replaced.hit, sizeof(rtm::Hit));
					channel.write_queue.push(write_req);
				}
				channel.hit_record_cache.direct_replace(rsb_req.hit_info, replace_index);

				MemoryRequest load_req;
				load_req.type = MemoryRequest::Type::LOAD;
				load_req.size = sizeof(rtm::Hit);
				load_req.paddr = rsb_req.hit_info.hit_address;
				channel.read_queue.push(load_req);

				request_network.read(channel_index);
			}
		}
	}
	else assert(false);
}

void UnitHitRecordUpdater::process_returns(uint channel_index) {
	uint port_in_main_memory = channel_index * main_mem_port_stride + main_mem_port_offset;
	if (!main_memory->return_port_read_valid(port_in_main_memory)) return;

	Channel& channel = channels[channel_index];
	MemoryReturn ret_from_dram = main_memory->read_return(port_in_main_memory);
	paddr_t hit_address = ret_from_dram.paddr;
	rtm::Hit hit_in_dram;
	std::memcpy(&hit_in_dram, ret_from_dram.data, sizeof(rtm::Hit));
	HitInfo hit_info = { hit_in_dram, hit_address };

	uint cache_index = channel.hit_record_cache.process_dram_hit(hit_info);
	if (cache_index != ~0) {
		HitInfo cloest_hit_info = channel.hit_record_cache.fetch_hit(cache_index);
		// If there are load requests from TP
		if (channel.rsb_load_queue.count(hit_address)) {
			uint64_t rsb_set = channel.rsb_load_queue[hit_address];
			for (uint64_t set = rsb_set, rsb_index; set != 0; set ^= (1ull << rsb_index)) {
				rsb_index = ctz(set);
				assert(channel.rsb_counter.count({ hit_address, rsb_index }));
				MemoryReturn ret_to_rsb;
				ret_to_rsb.paddr = hit_address;
				ret_to_rsb.port = rsb_index;
				ret_to_rsb.size = sizeof(rtm::Hit);
				std::memcpy(ret_to_rsb.data, &hit_info.hit, sizeof(rtm::Hit));
				uint counter = channel.rsb_counter[{hit_address, rsb_index}];
				while(counter--) channel.return_queue.push(ret_to_rsb);
				channel.rsb_counter.erase({ hit_address, rsb_index });
			}
			channel.rsb_load_queue.erase(hit_address);
		}
	}
	else assert(false); // there must be a record in the cache
}

void UnitHitRecordUpdater::issue_requests(uint channel_index) {
	uint port_in_main_memory = channel_index * main_mem_port_stride + main_mem_port_offset;
	if (!main_memory->request_port_write_valid(port_in_main_memory)) return;
	Channel& channel = channels[channel_index];

	bool requested = false;

	// Write first
	if (!channel.write_queue.empty()) {
		MemoryRequest req = channel.write_queue.front();
		channel.write_queue.pop();
		req.port = port_in_main_memory;
		main_memory->write_request(req, port_in_main_memory);
		requested = true;
	}
	// Read
	if (!requested && !channel.read_queue.empty()) {
		MemoryRequest req = channel.read_queue.front();
		channel.read_queue.pop();
		req.port = port_in_main_memory;
		main_memory->write_request(req, port_in_main_memory);
		requested = true;
	}

	
}

void UnitHitRecordUpdater::issue_returns(uint channel_index) {
	Channel& channel = channels[channel_index];
	auto& queue = channel.return_queue;

	if (!queue.empty()) {
		MemoryReturn ret = queue.front();
		if (return_network.is_write_valid(ret.port)) {
			return_network.write(ret, ret.port);
			assert(ret.size == sizeof(rtm::Hit));
			queue.pop();
		}
	}
}


}
}
}