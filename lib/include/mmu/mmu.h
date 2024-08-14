#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
#include "cartridge/cartridge.h"
#include "cartridge/cartridge_metadata.h"
#include "cpu/cpucontext.h"
#include "io/gamepad.h"
#include "io/lcd.h"
#include "mmu/dma.h"
#include "mmu/ram.h"
#include "ppu/ppu.h"
#include "utils/logger.h"

#include "io/device.h"

namespace gameboy::memory {

// 0x0000 - 0x3FFF : ROM Bank 0
// 0x4000 - 0x7FFF : ROM Bank 1 - Switchable
// 0x8000 - 0x97FF : CHR RAM
// 0x9800 - 0x9BFF : BG Map 1
// 0x9C00 - 0x9FFF : BG Map 2
// 0xA000 - 0xBFFF : Cartridge RAM
// 0xC000 - 0xCFFF : RAM Bank 0
// 0xD000 - 0xDFFF : RAM Bank 1-7 - switchable - Color only
// 0xE000 - 0xFDFF : Reserved - Echo RAM
// 0xFE00 - 0xFE9F : Object Attribute Memory
// 0xFEA0 - 0xFEFF : Reserved - Unusable
// 0xFF00 - 0xFF7F : I/O Registers
// 0xFF80 - 0xFFFE : Zero Page

class MMU
{
   public:
    MMU() = delete;
    explicit MMU(cartridge::Cartridge& cartridge, cpu::CPUContext& context) : cartridge_(cartridge), device_(context)
    {
    }

    auto read(uint16_t address) const -> uint8_t;

    auto read16(uint16_t address) const -> uint16_t;

    auto write(uint16_t address, uint8_t value) -> void;

    auto write16(uint16_t address, uint16_t value) -> void;

    auto ie_register() -> uint8_t;

    auto ppu() const noexcept -> const ppu::PPU&
    {
        return ppu_;
    }

    void ppu_tick(cpu::CPUContext& context)
    {
        auto read_handler = [this](uint16_t address) -> uint8_t { return read(address); };
        ppu_.tick(lcd_, context, read_handler);
    }

    auto dma() -> mmu::DMA&
    {
        return dma_;
    }

    // NEEDS TO BE RESTRUCTURED
    inline void dma_tick()
    {
        if (dma_.dma_context_.active == 0) {
            return;
        }

        if (dma_.dma_context_.start_delay != 0) {
            dma_.dma_context_.start_delay--;
            return;
        }
        const auto result_value = read((dma_.dma_context_.value * 0x100) + dma_.dma_context_.bt);
        ppu_.write<ppu::PPUWriteType::OAM>(dma_.dma_context_.bt, result_value);
        dma_.dma_context_.bt++;
        dma_.dma_context_.active = dma_.dma_context_.bt < 0xA0;  // up until 9F
    }

    auto gamepad() -> io::GamePad&
    {
        return device_.gamepad();
    }

    ~MMU() = default;

   private:
    cartridge::Cartridge& cartridge_;
    RAM ram_{};
    ppu::PPU ppu_{};
    io::Device device_;
    mmu::DMA dma_;
    lcd::LCD lcd_;

    uint8_t int_enable_register_{0};

    static constexpr uint16_t HRAM_LIMIT{0x8000};
    static constexpr uint16_t CHR_RAM_LIMIT{0xA000};

    static constexpr auto is_lcd_space = [](uint16_t address) { return (address >= 0xFF40) && (address <= 0xFF4B); };
    static constexpr bool DEBUG{false};
};

}  // namespace gameboy::memory
