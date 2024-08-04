#pragma once

#include <cstdint>
#include "cpu/cpucontext.h"
#include "io/lcd.h"
#include "ppu/ppu_context.h"
#include "ppu/ppu_fsm.h"
namespace gameboy::ppu {

enum class PPUWriteType : int8_t
{
    OAM,
    VRAM
};

class PPU
{
   public:
    PPU()
    {
        ppu_context_.video_buffer.resize(PPUContext::XRES * PPUContext::YRES);
    }

    template <PPUWriteType WriteType>
    auto read(uint16_t address) const noexcept -> uint8_t
    {
        if constexpr (WriteType == PPUWriteType::OAM) {
            if (address >= 0xFE00) {
                address -= 0xFE00;
            }

            return std::bit_cast<const uint8_t*>(ppu_context_.oam_ram.data())[address];
        } else {
            return std::bit_cast<const uint8_t*>(ppu_context_.vram.data())[address - 0x8000];
        }
    }

    template <PPUWriteType WriteType>
    void write(uint16_t address, uint8_t value)
    {
        if constexpr (WriteType == PPUWriteType::OAM) {
            if (address >= 0xFE00) {
                address -= 0xFE00;
            }

            auto* container = std::bit_cast<uint8_t*>(ppu_context_.oam_ram.data());
            container[address] = value;
        } else {
            auto* container = std::bit_cast<uint8_t*>(ppu_context_.vram.data());
            container[address - 0x8000] = value;
        }
    }

    void tick(lcd::LCD& lcd, cpu::CPUContext& context)
    {
        ppu_context_.line_ticks++;
        ppu_fsm_.handle(lcd, context);
    }

    auto context() const -> const PPUContext&
    {
        return ppu_context_;
    }

   private:
    PPUContext ppu_context_;
    PPUFSM ppu_fsm_{ppu_context_};
};

}  // namespace gameboy::ppu