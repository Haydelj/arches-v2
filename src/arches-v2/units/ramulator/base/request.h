#ifndef     RAMULATOR_BASE_REQUEST_H
#define     RAMULATOR_BASE_REQUEST_H

#include <vector>
#include <list>
#include <string>

#include "units/ramulator/base/base.h"

namespace Ramulator {

struct Request { 
  Addr_t    addr = -1;
  AddrVec_t addr_vec {};

  // Basic request id convention
  // 0 = Read, 1 = Write. The device spec defines all others
  struct Type {
    enum : int {
      Read = 0, 
      Write,
    };
  };
  
  int type_id = -1;    // An identifier for the type of the request
  int source_id = -1;  // An identifier for where the request is coming from (e.g., which core)
  unsigned int return_id = ~0u;     // An identifier for Arches-v2 wrapper
  unsigned int arches_channel = ~0u;    // Channel index from Arches-v2

  int command = -1;          // The command that need to be issued to progress the request
  int final_command = -1;    // The final command that is needed to finish the request
  bool is_first_command = true;     // whether it is the first command of this request
  bool is_write_merge = false;

  Clk_t arrive = -1;   // Clock cycle when the request arrive at the memory controller
  Clk_t depart = -1;   // Clock cycle when the request depart the memory controller

  std::function<void(Request&)> callback;

  Request(Addr_t addr, int type);
  Request(AddrVec_t addr_vec, int type);
  Request(Addr_t addr, int type, int source_id, std::function<void(Request&)> callback);
  Request(Addr_t addr, int type, int source_id, unsigned int return_id, std::function<void(Request&)> callback);
  Request(Addr_t addr, int type, int source_id, unsigned int return_id, unsigned int arches_channel, std::function<void(Request&)> callback);
};


struct ReqBuffer {
  std::list<Request> buffer;
  size_t max_size = 64;

  using iterator = std::list<Request>::iterator;
  iterator begin() { return buffer.begin(); };
  iterator end() { return buffer.end(); };


  size_t size() const { return buffer.size(); }

  bool enqueue(Request& request) {
      // write merge
      if (request.type_id == Request::Type::Write)
      {
          for (auto itr = buffer.begin(); itr != buffer.end(); ++itr)
          {
              if (request.addr == itr->addr)
              {
                  request.is_write_merge = true;
                  return true;
              }
          }
      }

    if (buffer.size() <= max_size) {
      buffer.push_back(request);
      return true;
    } else {
        //std::cout << "Enqueue Unsuccess!!! Buffer size = " << buffer.size() << ", channel = " << request.arches_channel << ", type = " << request.type_id << ", addr = " << request.addr << std::endl;
      return false;
    }
  }

  void remove(iterator it) {
    buffer.erase(it);
  }
};

}        // namespace Ramulator


#endif   // RAMULATOR_BASE_REQUEST_H