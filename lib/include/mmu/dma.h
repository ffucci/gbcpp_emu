#pragma once

#include <cstdint>
#include <memory>

#include "ppu/ppu.h"

namespace gameboy::mmu {

struct DMAContext
{
    bool active{true};
    uint8_t bt{};
    uint8_t value{};
    uint8_t start_delay{2};
};

struct DMA
{
   public:
    DMA() = default;

    void start(uint8_t start)
    {
        dma_context_.active = true;
        dma_context_.bt = 0;
        dma_context_.start_delay = 2;
        dma_context_.value = start;
    }

    inline bool transfering() const noexcept
    {
        return dma_context_.active;
    }

    DMAContext dma_context_;
};
};  // namespace gameboy::mmu