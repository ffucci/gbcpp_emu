#pragma once

#include <sys/types.h>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "cartridge/cartridge_metadata.h"
#include "cartridge/mbc1_cart.h"
#include "utils/logger.h"

namespace gameboy::cartridge {

inline bool address_in_range(uint16_t address, uint16_t left, uint16_t right)
{
    return address >= left && address < right;
}

class Cartridge
{
   public:
    explicit Cartridge(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : filename_(std::move(filename)), rom_data_(std::move(rom_data)), header_(std::move(metadata))
    {
        setup_banks(header_.ram_size);
    }

    virtual auto read(uint16_t address) const -> uint8_t = 0;
    virtual void write(uint16_t address, uint8_t value) = 0;

    Cartridge() = delete;

    virtual ~Cartridge() = default;

   protected:
    void setup_banks(uint8_t ram_size)
    {
        std::cout << "Setup banks... ram_size: " << ram_size << std::endl;
        switch (ram_size) {
            case 2: {
                ram_data_.resize(RAM_BANK_SIZE);
                break;
            };

            case 3: {
                ram_data_.resize(4 * RAM_BANK_SIZE);
                break;
            }

            case 4: {
                ram_data_.resize(16 * RAM_BANK_SIZE);
                break;
            }

            case 5: {
                ram_data_.resize(8 * RAM_BANK_SIZE);
                break;
            }
        }
    }

    std::string filename_;
    RomBank rom_data_;
    RamBank ram_data_{};

    CartridgeMetadata header_;

    bool ram_enabled_{false};
    uint16_t rom_idx_{0};
    uint16_t ram_idx_{0};

    static constexpr uint16_t RAM_BANK_SIZE{0x2000};
    static constexpr uint16_t ROM_BASE_ADDRESS{0x4000};
};

class NoMBC : public Cartridge
{
   public:
    explicit NoMBC(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
    {
    }

    uint8_t read(uint16_t address) const override
    {
        return rom_data_[address];
    }

    void write(uint16_t address, uint8_t) override
    {
    }
};

class MBC1 : public Cartridge
{
   public:
    explicit MBC1(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
    {
        rom_idx_ = 1;
        std::cout << "Size of ram: " << ram_data_.size() << std::endl;
    }

    uint8_t read(uint16_t address) const override
    {
        if (address < 0x4000) {
            return rom_data_[address];
        }

        uint16_t masked_address = (address & 0xE000);  // 1110 0000

        // The last space is reserved to RAM
        if (masked_address == 0xA000) {
            if (!ram_enabled_) {
                return 0xFF;
            }
            uint16_t addr_within_bank = address - 0xA000;
            uint16_t bank_offset = 0x2000 * ram_idx_;
            return ram_data_[bank_offset + addr_within_bank];
        }

        uint16_t addr_within_bank = address - ROM_BASE_ADDRESS;
        uint32_t bank_offset = ROM_BASE_ADDRESS * rom_idx_;
        return rom_data_[bank_offset + addr_within_bank];
    }

    void write(uint16_t address, uint8_t value) override
    {
        uint16_t masked_address = (address & 0xE000);  // 1110 0000
        if (address < 0x2000) {
            ram_enabled_ = ((value & 0xF) == 0xA);
        }

        if (masked_address == 0x2000) {
            value = (value == 0) ? 1 : value;
            value &= 0b11111;
            rom_idx_ = value;
            return;
        }

        if (masked_address == 0x4000) {
            value &= 0b11;
            if (ram_banking) {
                ram_idx_ = value;
            }
            return;
        }

        // The last space is reserved to RAM
        if (masked_address == 0x6000) {
            ram_banking = value & 1;
        }

        // The last space is reserved to RAM
        if (masked_address == 0xA000) {
            if (!ram_enabled_) {
                return;
            }
            uint16_t addr_within_bank = address - 0xA000;
            uint32_t bank_offset = 0x2000 * ram_idx_;
            ram_data_[bank_offset + addr_within_bank] = value;
        }
    }

    // battery
    bool battery{false};
    bool need_save{false};  // should save data
    uint8_t ram_banking{0x0};
};

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
    bool battery{false};
    bool need_save{false};  // should save data
    bool rtc_enabled_{false};

    // Keeps track of rising edge
    uint8_t rising_edge_{0x0};

    // Keeps track of the selected RTC register
    uint8_t rtc_select_{0x00};

    uint8_t rtc_[5];
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    bool halt_timer_{false};
};

inline std::unique_ptr<Cartridge> get_cartridge(std::string filename)
{
    auto rom_data = gameboy::utils::read_rom(filename);
    auto header = gameboy::cartridge::from_rom(rom_data);

    if (header.is_mbc1()) {
        return std::make_unique<MBC1>(std::move(filename), rom_data, header);
    }

    if (header.is_mbc3()) {
        return std::make_unique<MBC3>(std::move(filename), rom_data, header);
    }

    return std::make_unique<NoMBC>(std::move(filename), rom_data, header);
}

}  // namespace gameboy::cartridge