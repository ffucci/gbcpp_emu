#pragma once

#include <cstdint>
namespace gameboy::io {
struct GamePadState
{
    bool start{false};
    bool select{false};
    bool a{false};
    bool b{false};
    // Direction part
    bool up{false};
    bool down{false};
    bool left{false};
    bool right{false};
};

struct GamePadContext
{
    bool button_select;
    bool direction_select;
    GamePadState state;
};

class GamePad
{
   public:
    auto button_selected() const noexcept -> bool
    {
        return ctx_.button_select;
    }

    auto direction_selected() const noexcept -> bool
    {
        return ctx_.direction_select;
    }

    auto state() noexcept -> GamePadState&
    {
        return ctx_.state;
    }

    void set_selection(uint8_t value)
    {
        ctx_.button_select = value & 0x20;
        ctx_.direction_select = value & 0x10;
    }

    auto get_output() const noexcept -> uint8_t
    {
        uint8_t output = 0xCF;
        auto button_set = !button_selected();
        if (button_set) {
            // if start is pressed
            if (ctx_.state.start) {
                output &= ~(1 << 3);
            } else if (ctx_.state.select) {
                output &= ~(1 << 2);
            } else if (ctx_.state.a) {
                output &= ~(1 << 1);
            } else if (ctx_.state.b) {
                output &= ~(1 << 0);
            }
        }

        auto dir_set = !direction_selected();
        if (dir_set) {
            // if start is pressed
            if (ctx_.state.left) {
                output &= ~(1 << 1);
            } else if (ctx_.state.right) {
                output &= ~(1 << 0);
            } else if (ctx_.state.up) {
                output &= ~(1 << 2);
            } else if (ctx_.state.down) {
                output &= ~(1 << 3);
            }
        }
        return output;
    }

   private:
    GamePadContext ctx_;
};

}  // namespace gameboy::io