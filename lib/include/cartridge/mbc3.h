#pragma once

#include <chrono>

#include "cartridge/cartridge.h"

namespace gameboy::cartridge {

class MBC3 : public Cartridge
{
   public:
    explicit MBC3(std::string filename, RomBank rom_data, CartridgeMetadata metadata);

    uint8_t read(uint16_t address) const override;

    void write(uint16_t address, uint8_t value) override;

   private:
    void latch_rtc();

    void write_rtc(uint8_t value);

    bool rtc_enabled_{false};

    // Keeps track of rising edge
    uint8_t rising_edge_{0x0};

    // Keeps track of the selected RTC register
    uint8_t rtc_select_{0x00};

    uint8_t rtc_[5]{};
    std::chrono::time_point<std::chrono::system_clock> start_time_{};
    bool halt_timer_{false};
};

}  // namespace gameboy::cartridge
