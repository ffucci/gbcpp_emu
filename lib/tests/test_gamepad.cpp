#include <gtest/gtest.h>
#include "io/gamepad.h"

namespace gameboy::tests {

TEST(GamePadTest, GIVEN_no_pressed_buttons_WHEN_no_group_is_selected_THEN_selection_bits_are_preserved)
{
    io::GamePad gamepad;

    gamepad.set_selection(0x30);

    EXPECT_EQ(gamepad.get_output(), 0xFF);
}

TEST(GamePadTest, GIVEN_pressed_a_WHEN_button_group_is_selected_THEN_a_is_active_low)
{
    io::GamePad gamepad;

    gamepad.set_selection(0x10);
    gamepad.set_pressed(io::GamePad::Button::A, true);

    EXPECT_EQ(gamepad.get_output(), 0xDD);
}

TEST(GamePadTest, GIVEN_pressed_right_WHEN_direction_group_is_selected_THEN_right_is_active_low)
{
    io::GamePad gamepad;

    gamepad.set_selection(0x20);
    gamepad.set_pressed(io::GamePad::Button::Right, true);

    EXPECT_EQ(gamepad.get_output(), 0xEE);
}

}  // namespace gameboy::tests
