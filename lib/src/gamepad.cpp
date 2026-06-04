#include "io/gamepad.h"

namespace gameboy::io {

void GamePad::set_pressed(Button button, bool pressed) noexcept
{
    const auto mask = button_mask(button);
    if (pressed) {
        context_ |= mask;
        return;
    }

    context_ &= ~mask;
}

void GamePad::set_selection(uint8_t value)
{
    set_bit(BUTTON_SELECT_MASK, value & 0x20);
    set_bit(DIRECTION_SELECT_MASK, value & 0x10);
}

auto GamePad::get_output() const noexcept -> uint8_t
{
    uint8_t output = 0xC0;
    if (button_selected()) {
        output |= 0x20;
    }

    if (direction_selected()) {
        output |= 0x10;
    }

    uint8_t input = 0x0F;
    auto button_set = !button_selected();
    if (button_set) {
        if (pressed(Button::Start)) {
            input &= ~(1 << 3);
        }

        if (pressed(Button::Select)) {
            input &= ~(1 << 2);
        }

        if (pressed(Button::A)) {
            input &= ~(1 << 1);
        }

        if (pressed(Button::B)) {
            input &= ~(1 << 0);
        }
    }

    auto dir_set = !direction_selected();
    if (dir_set) {
        if (pressed(Button::Left)) {
            input &= ~(1 << 1);
        }

        if (pressed(Button::Right)) {
            input &= ~(1 << 0);
        }

        if (pressed(Button::Up)) {
            input &= ~(1 << 2);
        }

        if (pressed(Button::Down)) {
            input &= ~(1 << 3);
        }
    }
    return output | input;
}

void GamePad::set_bit(uint16_t mask, bool enabled) noexcept
{
    if (enabled) {
        context_ |= mask;
        return;
    }

    context_ &= ~mask;
}

}  // namespace gameboy::io
