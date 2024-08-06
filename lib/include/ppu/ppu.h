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
        ppu_context_.pfc.fetch_state = PPUFetchState::Tile;
        ppu_context_.video_buffer = std::vector<uint32_t>(PPUContext::XRES * PPUContext::YRES, 0);
    }

    template <PPUWriteType WriteType>
    auto read(uint16_t address) const noexcept -> uint8_t
    {
        if constexpr (WriteType == PPUWriteType::OAM) {
            if (address >= 0xFE00) {
                address -= 0xFE00;
            }

            return std::bit_cast<const uint8_t*>(ppu_context_.oam_ram.data())[address];
        }

        return std::bit_cast<const uint8_t*>(ppu_context_.vram.data())[address - 0x8000];
    }

    template <PPUWriteType WriteType>
    void write(uint16_t address, uint8_t value)
    {
        if constexpr (WriteType == PPUWriteType::OAM) {
            if (address >= 0xFE00) {
                address -= 0xFE00;
            }

            uint8_t* container = std::bit_cast<uint8_t*>(ppu_context_.oam_ram.data());
            container[address] = value;
        } else {
            uint8_t* container = std::bit_cast<uint8_t*>(ppu_context_.vram.data());
            container[address - 0x8000] = value;
        }
    }

    void tick(lcd::LCD& lcd, cpu::CPUContext& context, std::function<uint8_t(uint16_t)>&& on_read)
    {
        ppu_context_.line_ticks++;
        ppu_fsm_.handle(lcd, context, std::move(on_read));
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