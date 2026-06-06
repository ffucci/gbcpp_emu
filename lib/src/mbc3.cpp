#include "cartridge/mbc3.h"

#include <utility>

namespace gameboy::cartridge {
namespace {

constexpr uint16_t ROM_BANK_0_END{0x4000};
constexpr uint16_t SWITCHABLE_ROM_BANK_START{0x4000};
constexpr uint16_t SWITCHABLE_ROM_BANK_END{0x8000};
constexpr uint16_t RAM_ENABLE_END{0x2000};
constexpr uint16_t ROM_BANK_SELECT_START{0x2000};
constexpr uint16_t ROM_BANK_SELECT_END{0x4000};
constexpr uint16_t RAM_OR_RTC_SELECT_START{0x4000};
constexpr uint16_t RAM_OR_RTC_SELECT_END{0x6000};
constexpr uint16_t RTC_LATCH_START{0x6000};
constexpr uint16_t RTC_LATCH_END{0x8000};
constexpr uint16_t EXTERNAL_RAM_START{0xA000};
constexpr uint16_t EXTERNAL_RAM_END{0xC000};

constexpr uint8_t RAM_ENABLE_VALUE{0x0A};
constexpr uint8_t LOW_NIBBLE_MASK{0x0F};
constexpr uint8_t ROM_BANK_MASK{0x7F};
constexpr uint8_t MAX_RAM_BANK{0x03};
constexpr uint8_t RTC_SECONDS_REGISTER{0x08};
constexpr uint8_t RTC_DAY_HIGH_REGISTER{0x0C};
constexpr uint8_t RTC_REGISTER_MASK{0x07};
constexpr uint8_t RTC_LATCH_VALUE{0x01};
constexpr uint8_t MAX_SECONDS_OR_MINUTES{0x3B};
constexpr uint8_t MAX_HOURS{0x17};
constexpr uint8_t RTC_HALT_BIT{0x40};
constexpr uint16_t FIRST_SWITCHABLE_ROM_BANK{1};

}  // namespace

MBC3::MBC3(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
    : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
{
    rom_idx_ = FIRST_SWITCHABLE_ROM_BANK;
}

uint8_t MBC3::read(uint16_t address) const
{
    if (address < ROM_BANK_0_END) {
        return rom_data_[address];
    }

    if (address_in_range(address, SWITCHABLE_ROM_BANK_START, SWITCHABLE_ROM_BANK_END)) {
        const uint16_t addr_within_bank = address - SWITCHABLE_ROM_BANK_START;
        const uint32_t bank_offset = ROM_BASE_ADDRESS * rom_idx_;
        return rom_data_[bank_offset + addr_within_bank];
    }

    if (address_in_range(address, EXTERNAL_RAM_START, EXTERNAL_RAM_END)) {
        if (!ram_enabled_) {
            return 0xFF;
        }

        if (!rtc_enabled_) {
            const uint16_t addr_within_bank = address - EXTERNAL_RAM_START;
            const uint32_t bank_offset = RAM_BANK_SIZE * ram_idx_;
            return ram_data_[bank_offset + addr_within_bank];
        }

        return rtc_[rtc_select_];
    }

    return 0x0;
}

void MBC3::write(uint16_t address, uint8_t value)
{
    if (address < RAM_ENABLE_END) {
        ram_enabled_ = ((value & LOW_NIBBLE_MASK) == RAM_ENABLE_VALUE);
    }

    if (address_in_range(address, ROM_BANK_SELECT_START, ROM_BANK_SELECT_END)) {
        value = (value == 0) ? FIRST_SWITCHABLE_ROM_BANK : value;
        rom_idx_ = value & ROM_BANK_MASK;
        return;
    }

    if (address_in_range(address, RAM_OR_RTC_SELECT_START, RAM_OR_RTC_SELECT_END)) {
        if (value <= MAX_RAM_BANK) {
            rtc_enabled_ = false;
            need_save_ = true;
            ram_idx_ = value;
            return;
        }

        if (value >= RTC_SECONDS_REGISTER && value <= RTC_DAY_HIGH_REGISTER) {
            rtc_enabled_ = true;
            rtc_select_ = value & RTC_REGISTER_MASK;
        }
        return;
    }

    if (address_in_range(address, RTC_LATCH_START, RTC_LATCH_END)) {
        if (value == RTC_LATCH_VALUE && rising_edge_ == 0) {
            latch_rtc();
        }

        rising_edge_ = RTC_LATCH_VALUE;
        return;
    }

    if (address_in_range(address, EXTERNAL_RAM_START, EXTERNAL_RAM_END)) {
        if (!ram_enabled_) {
            return;
        }

        if (!rtc_enabled_) {
            const uint16_t addr_within_bank = address - EXTERNAL_RAM_START;
            const uint32_t bank_offset = RAM_BANK_SIZE * ram_idx_;
            ram_data_[bank_offset + addr_within_bank] = value;
            if (battery_saver) {
                battery_saver->save(filename_, ram_data_.data() + bank_offset);
            }
            return;
        }

        write_rtc(value);
    }
}

void MBC3::latch_rtc()
{
    if (halt_timer_) {
        return;
    }

    const auto elapsed_time = std::chrono::system_clock::now() - start_time_;
    const std::chrono::seconds rtc_time(rtc_[0] + rtc_[1] * 60 + rtc_[2] * 3600 + rtc_[3] * 86400);
    const auto updated_time = rtc_time + elapsed_time;

    rtc_[0] = std::chrono::duration_cast<std::chrono::seconds>(updated_time).count() % 60;
    rtc_[1] = std::chrono::duration_cast<std::chrono::minutes>(updated_time).count() % 60;
    rtc_[2] = std::chrono::duration_cast<std::chrono::hours>(updated_time).count() % 24;
    rtc_[3] = 0x00;
    rtc_[4] = halt_timer_ << 6;

    start_time_ = std::chrono::system_clock::now();
}

void MBC3::write_rtc(uint8_t value)
{
    start_time_ = std::chrono::system_clock::now();
    rtc_[rtc_select_] = value;

    switch (rtc_select_) {
        case 0:
        case 1:
            if (value > MAX_SECONDS_OR_MINUTES) {
                rtc_[rtc_select_] = MAX_SECONDS_OR_MINUTES;
            }
            break;
        case 2:
            if (value > MAX_HOURS) {
                rtc_[rtc_select_] = MAX_HOURS;
            }
            break;
        case 4:
            if (halt_timer_) {
                halt_timer_ = (value & RTC_HALT_BIT) != 0;
                if (!halt_timer_) {
                    start_time_ = std::chrono::system_clock::now();
                }
            } else {
                halt_timer_ = (value & RTC_HALT_BIT) != 0;
            }

            rtc_[rtc_select_] = value & RTC_HALT_BIT;
            break;
        default:
            break;
    }
}

}  // namespace gameboy::cartridge
