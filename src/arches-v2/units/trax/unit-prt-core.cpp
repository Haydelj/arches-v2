#include "unit-prt-core.hpp"

namespace Arches { namespace Units { namespace TRaX {

UnitPRTCore::UnitPRTCore(const Configuration& config) :
	_max_rays(config.max_rays), _num_tp(config.num_tp), _node_base_addr(config.node_base_addr), _tri_base_addr(config.tri_base_addr),
	_cache(config.cache), _request_network(config.num_tp, 1), _return_network(1, config.num_tp),
	_box_pipline(3, 1), _tri_pipline(22, 1)
{
	_tri_staging_buffers.resize(config.max_rays);
	_ray_states.resize(config.max_rays);
	for(uint i = 0; i < _ray_states.size(); ++i)
		_free_ray_ids.insert(i);
}

bool UnitPRTCore::_try_queue_node(uint ray_id, uint node_id)
{
	paddr_t start = _node_base_addr + node_id * sizeof(rtm::PackedBVH2::NodePack);
	_fetch_queue.push({start, (uint8_t)(sizeof(rtm::PackedBVH2::NodePack)), (uint16_t)ray_id});
	return true;
}

bool UnitPRTCore::_try_queue_tri(uint ray_id, uint tri_id)
{
	paddr_t start = _tri_base_addr + tri_id * sizeof(rtm::Triangle);
	paddr_t end = start + sizeof(rtm::Triangle);

	_tri_staging_buffers[ray_id].tri_id = tri_id;
	_tri_staging_buffers[ray_id].bytes_filled = 0;

	//split request at cache boundries
	//queue the requests to fill the buffer
	paddr_t addr = start;
	while(addr < end)
	{
		paddr_t next_boundry = std::min(end, _block_address(addr + CACHE_BLOCK_SIZE));
		uint8_t size = next_boundry - addr;
		_fetch_queue.push({addr, size, (uint16_t)(ray_id | 0x8000u)});
		addr += size;
	}

	return true;
}

void UnitPRTCore::_read_requests()
{

}

void UnitPRTCore::_read_returns()
{

}

void UnitPRTCore::_schedule_ray()
{

}

void UnitPRTCore::_simualte_intersectors()
{

}

void UnitPRTCore::_issue_requests()
{

}

void UnitPRTCore::_issue_returns()
{

}

}}}