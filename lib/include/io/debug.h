#pragma once

#include <array>
#include <cstdint>
#include "mmu/mmu.h"

namespace gameboy::io {
class Debug
{
   public:
    Debug() : debug_ring_(1024, 0), ring_size_(0)
    {
    }

    void update(memory::MMU& memory)
    {
        if (memory.read(0xFF02) == 0x81) {
            char c = memory.read(0xFF01);
            debug_ring_[ring_size_++] = c;
            memory.write(0xFF02, 0);
        }

        print();
    }

   private:
    void print()
    {
        if (debug_ring_[0]) {
            // auto& logger = logger::Logger::instance();
            // logger.log("DBG: {}", debug_ring_);
        }
    }
    std::string debug_ring_;
    uint32_t ring_size_{};
};
}  // namespace gameboy::io
