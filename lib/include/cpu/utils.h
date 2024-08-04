#pragma once

#include <cstddef>
#include "cpu/cpucontext.h"
#include "mmu/mmu.h"
namespace gameboy::cpu {

inline void update_cycles(size_t cycles, CPUContext& context, memory::MMU& memory)
{
    for (auto i = 0ul; i < cycles; ++i) {
        // make it at compile tiome
        for (int x = 0; x < 4; ++x) {
            context.ticks++;
            // timer_tick
            context.timer.tick(context.interrupt_flags);
            memory.ppu_tick(context);
        }

        memory.dma_tick();
    }
}
}  // namespace gameboy::cpu