#include "../include/utils/emulator_session.h"

#include <exception>

#include <SDL_log.h>

#include "cartridge/cartridge_factory.h"
#include "cpu/cpucontext.h"

namespace gameboy::app {

bool EmulatorSession::load_rom(const std::string& path)
{
    stop();

    try {
        auto cartridge = cartridge::get_cartridge(path);
        auto cpu = std::make_unique<cpu::CPU>(cpu::get_initial_state(), *cartridge);
        auto* cpu_ptr = cpu.get();

        {
            std::scoped_lock lock(mutex_);
            cartridge_ = std::move(cartridge);
            cpu_ = std::move(cpu);
            current_rom_path_ = path;
        }

        cpu_thread_ = std::jthread([cpu_ptr](std::stop_token token) { cpu_ptr->run(token); });
        return true;
    } catch (const std::exception& e) {
        SDL_Log("Could not load ROM '%s': %s", path.c_str(), e.what());
    } catch (...) {
        SDL_Log("Could not load ROM '%s': unknown error", path.c_str());
    }

    return false;
}

void EmulatorSession::stop()
{
    if (cpu_thread_.joinable()) {
        cpu_thread_.request_stop();
        cpu_thread_.join();
    }

    std::scoped_lock lock(mutex_);
    cpu_.reset();
    cartridge_.reset();
    current_rom_path_.clear();
}

}  // namespace gameboy::app
