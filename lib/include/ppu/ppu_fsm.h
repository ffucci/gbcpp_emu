#pragma once

#include <SDL2/SDL_timer.h>
#include <array>
#include <cstdint>
#include <functional>
#include <utility>

#include "cpu/cpucontext.h"
#include "cpu/interrupt_type.h"
#include "io/lcd.h"
#include "ppu/ppu_context.h"
#include "utils/logger.h"

namespace gameboy::ppu {

class PPUFSM
{
   public:
    PPUFSM() = delete;
    explicit PPUFSM(PPUContext& context) : ppu_context_(context) /*  */
    {
    }

    void handle(lcd::LCD& lcd, cpu::CPUContext& cpu_context, const std::function<uint8_t(uint16_t)>& on_read)
    {
        const auto mode = lcd.context().lcds_mode();
        PPUHandler handler = ppu_handlers_.at(std::to_underlying(mode));
        std::invoke(handler, this, lcd, cpu_context, on_read);  // (this->*handler)(lcd, cpu_context);
    }

   private:
    void mode_vblank(lcd::LCD& lcd, cpu::CPUContext& cpu_context, const std::function<uint8_t(uint16_t)>&)
    {
        auto& context = lcd.context();
        if (ppu_context_.line_ticks >= TICKS_PER_LINE) {
            context.inc_ly(ppu_context_.window_line, cpu_context);
            if (context.ly >= LINES_PER_FRAME) {
                context.set_lcds_mode(lcd::LCDMode::Oam);
                context.ly = 0;  // When the frame is over we can cleanup
                ppu_context_.window_line = 0;
            }

            ppu_context_.line_ticks = 0;
        }
    }

    void mode_oam(lcd::LCD& lcd, cpu::CPUContext& cpu_context, const std::function<uint8_t(uint16_t)>&)
    {
        if (ppu_context_.line_ticks >= 80) {
            lcd.context().set_lcds_mode(lcd::LCDMode::Transfer);

            auto& pq_context = ppu_context_.pfc;
            pq_context.line_reset();
        }

        if (ppu_context_.line_ticks == 1) {
            ppu_context_.oam_freelist.reset();
            ppu_context_.load_line_sprites(lcd);
        }
    }

    void mode_transfer(lcd::LCD& lcd, cpu::CPUContext& cpu_context, const std::function<uint8_t(uint16_t)>& on_read)
    {
        auto& context = lcd.context();
        ppu_context_.process(lcd, on_read);
        if (ppu_context_.pfc.pushed_x >= PPUContext::XRES) {
            // fifo reset
            ppu_context_.fifo_reset();
            context.set_lcds_mode(lcd::LCDMode::HBlank);

            if (lcd.context().status_interrupt_mode(lcd::LCDStat::HBlank)) {
                cpu_context.request_interrupt(cpu::InterruptType::LCD_STAT);
            }
        }
    }

    void mode_hblank(lcd::LCD& lcd, cpu::CPUContext& cpu_context, const std::function<uint8_t(uint16_t)>& on_read)
    {
        auto& lcd_context = lcd.context();

        if (ppu_context_.line_ticks >= TICKS_PER_LINE) {
            lcd_context.inc_ly(ppu_context_.window_line, cpu_context);
            if (lcd_context.ly >= PPUContext::YRES) {
                lcd_context.set_lcds_mode(lcd::LCDMode::VBlank);
                cpu_context.request_interrupt(cpu::InterruptType::VBLANK);

                // If the interrupt is in vblank
                if (lcd_context.status_interrupt_mode(lcd::LCDStat::VBlank)) {
                    cpu_context.request_interrupt(cpu::InterruptType::LCD_STAT);
                }

                ppu_context_.current_frame++;

                // update frames
                uint32_t end_time = SDL_GetTicks();
                auto frame_time = end_time - prev_frame_time_;

                if (frame_time < target_frame_time) {
                    SDL_Delay((target_frame_time - frame_time));  // waiting to finish the target frame time
                }

                if ((end_time - start_timer_) >= 1000) {
                    uint32_t fps = frame_count_;
                    start_timer_ = end_time;
                    frame_count_ = 0;
                    auto& logger = logger::Logger::instance();
                    logger.log("FPS :{}", fps);
                }

                frame_count_++;
                prev_frame_time_ = SDL_GetTicks();
            } else {
                lcd_context.set_lcds_mode(lcd::LCDMode::Oam);
            }

            ppu_context_.line_ticks = 0;
        }
    }

    using PPUHandler = void (PPUFSM::*)(lcd::LCD&, cpu::CPUContext&, const std::function<uint8_t(uint16_t)>&);

    static constexpr uint16_t TICKS_PER_LINE{456};
    static constexpr uint16_t LINES_PER_FRAME{154};
    static constexpr uint32_t target_frame_time{1000 / 60};

    std::array<PPUHandler, std::to_underlying(lcd::LCDMode::Transfer) + 1> ppu_handlers_ = {
        &PPUFSM::mode_hblank, &PPUFSM::mode_vblank, &PPUFSM::mode_oam, &PPUFSM::mode_transfer};

    uint32_t prev_frame_time_ = 0;
    uint32_t start_timer_ = 0;
    uint32_t frame_count_ = 0;
    PPUContext& ppu_context_;
};

}  // namespace gameboy::ppu