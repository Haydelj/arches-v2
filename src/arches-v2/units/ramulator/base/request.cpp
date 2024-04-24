#include "units/ramulator/base/request.h"

namespace Ramulator {

Request::Request(Addr_t addr, int type): addr(addr), type_id(type) {};

Request::Request(AddrVec_t addr_vec, int type): addr_vec(addr_vec), type_id(type) {};

Request::Request(Addr_t addr, int type, int source_id, std::function<void(Request&)> callback):
addr(addr), type_id(type), source_id(source_id), callback(callback) {};

Request::Request(Addr_t addr, int type, int source_id, unsigned int return_id, std::function<void(Request&)> callback) :
	addr(addr), type_id(type), source_id(source_id), return_id(return_id), callback(callback) {};

Request::Request(Addr_t addr, int type, int source_id, unsigned int return_id, unsigned int arches_channel, std::function<void(Request&)> callback) :
	addr(addr), type_id(type), source_id(source_id), return_id(return_id), arches_channel(arches_channel), callback(callback) {};
}        // namespace Ramulator

