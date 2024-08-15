#pragma once

#include "cartridge/cartridge.h"

namespace gameboy::cartridge {

class MBC3 : public Cartridge
{
   public:
    explicit MBC3(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
    {
        // Starting from idx 1
        rom_idx_ = 1;
    }

    uint8_t read(uint16_t address) const override
    {
        if (address < 0x4000) {
            return rom_data_[address];
        }

        // Rom reading
        if (address >= 0x4000 && address < 0x8000) {
            uint16_t addr_within_bank = address - 0x4000;
            uint32_t bank_offset = 0x4000 * rom_idx_;
            return rom_data_[bank_offset + addr_within_bank];
        }

        // The last space is reserved to RAM
        if (address_in_range(address, 0xA000, 0xC000)) {
            if (!ram_enabled_) {
                return 0xFF;
            }

            if (!rtc_enabled_) {
                uint16_t addr_within_bank = address - 0xA000;
                uint32_t bank_offset = 0x2000 * ram_idx_;
                return ram_data_[bank_offset + addr_within_bank];
            }

            return rtc_[rtc_select_];
        }

        return 0x0;
    }

    void write(uint16_t address, uint8_t value) override
    {
        uint16_t masked_address = (address & 0xE000);  // 1110 0000
        if (address < 0x2000) {
            ram_enabled_ = ((value & 0xF) == 0xA);
        }

        // In this range we select the corresponding ROM bank
        if (address_in_range(address, 0x2000, 0x4000)) {
            value = (value == 0) ? 1 : value;
            rom_idx_ = value & 0x7F;
            return;
        }

        if (address_in_range(address, 0x4000, 0x6000)) {
            if (value <= 0x03) {
                rtc_enabled_ = false;
                ram_idx_ = value;
                return;
            }

            if (value >= 0x08 && value <= 0x0C) {
                rtc_enabled_ = true;
                rtc_select_ = value & 0x07;
            }
            return;
        }

        if (address_in_range(address, 0x6000, 0x8000)) {
            // On a rising edge 0x00->0x01, the current
            // time is latched into the RTC register.
            if (value == 0x01 && rising_edge_ == 0) {
                // If the timer is not halted then add the time elapsed
                if (!halt_timer_) {
                    // Get time elapsed since last write
                    auto ElapsedTime = std::chrono::system_clock::now() - start_time_;

                    // Convert time stored into chrono object
                    std::chrono::seconds RTCTime(rtc_[0] * 1 + rtc_[1] * 60 + rtc_[2] * 3600 + rtc_[3] * 86400);

                    // Add time elapsed to time in RTC
                    auto UpdatedTime = RTCTime + ElapsedTime;

                    rtc_[0] =
                        std::chrono::duration_cast<std::chrono::seconds>(UpdatedTime).count() % 60;  // Seconds 0-59
                    rtc_[1] =
                        std::chrono::duration_cast<std::chrono::minutes>(UpdatedTime).count() % 60;  // Minutes 0-59
                    rtc_[2] = std::chrono::duration_cast<std::chrono::hours>(UpdatedTime).count();   // Hours 0-23
                    rtc_[2] %= 24;
                    rtc_[3] /= 24;  // days 0x00-0xFF

                    // Since I will not implement save functionality, I won't worry about
                    // keeping track of the number of days played.
                    rtc_[3] = 0x00;  // Days low 0x00 - 0xFF;
                    rtc_[4] = halt_timer_ << 6;

                    start_time_ = std::chrono::system_clock::now();
                }
            }

            rising_edge_ = 0x01;
            return;
        }

        // The last space is reserved to RAM
        if (address_in_range(address, 0xA000, 0xC000)) {
            if (!ram_enabled_) {
                return;
            }

            if (rtc_enabled_) {
                uint16_t addr_within_bank = address - 0xA000;
                uint32_t bank_offset = 0x2000 * ram_idx_;
                ram_data_[bank_offset + addr_within_bank] = value;
                return;
            }

            start_time_ = std::chrono::system_clock::now();

            rtc_[rtc_select_] = value;

            // Ensure values written are within the correct ranges
            switch (rtc_select_) {
                case 0:
                case 1:  // Seconds or minutes
                    if (value > 0x3B) {
                        rtc_[rtc_select_] = 0x3B;
                    }
                    break;
                case 2:  // Hours
                    if (value > 0x17) {
                        rtc_[rtc_select_] = 0x17;
                    }
                    break;
                case 4:  // Day high
                    // We will ignores the highest bit of day and
                    // the correspond carry

                    // If the timer was halted and is no longer, then
                    // update the StartTime variable.
                    if (halt_timer_) {
                        halt_timer_ = (value >> 6) == 1;
                        if (halt_timer_ == 0) {
                            start_time_ = std::chrono::system_clock::now();
                        }
                    } else {
                        halt_timer_ = (value >> 6) == 1;
                    }

                    rtc_[rtc_select_] = value & 0x40;

                    break;
            }
        }
    }

    // battery
    bool rtc_enabled_{false};

    // Keeps track of rising edge
    uint8_t rising_edge_{0x0};

    // Keeps track of the selected RTC register
    uint8_t rtc_select_{0x00};

    uint8_t rtc_[5];
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    bool halt_timer_{false};
};

}  // namespace gameboy::cartridge