#include <gtest/gtest.h>
#include "cpu/cpucontext.h"
#include "cpu/instruction_handlers.h"

namespace gameboy::tests {

class CPUContextFixture : public ::testing::Test
{
};

TEST_F(CPUContextFixture, GIVEN_CPUContext_WHEN_setting_flag_THEN_flags_are_set_correctly)
{
    cpu::CPUContext context{};

    cpu::cpu_set_flag(context.registers.f, 1, 0, 0, 0);
    std::cout << static_cast<int>(context.registers.f) << std::endl;
    std::cout << std::format("flags {:07B}", context.registers.f) << std::endl;
    std::cout << context.registers.get_flags_string() << std::endl;

    EXPECT_EQ(context.registers.f, cpu::ZERO);

    cpu::cpu_set_flag(context.registers.f, 0, 1, 0, 0);
    EXPECT_EQ(context.registers.get_flags_string(), "-N--");
    EXPECT_EQ(context.registers.f, cpu::N_SUB);

    cpu::cpu_set_flag(context.registers.f, 0, 0, 1, 0);
    EXPECT_EQ(context.registers.get_flags_string(), "--H-");
    EXPECT_EQ(context.registers.f, cpu::HALF);
}

TEST_F(CPUContextFixture, GIVEN_CPUContext_WHEN_setting_flag_THEN_half_flag_works)
{
    cpu::CPUContext context{};
    const auto val = 0xC010;
    cpu::cpu_set_flag(context.registers.f, 0, 0, (0xF & val) == 0, 1);
    std::cout << static_cast<int>(context.registers.f) << std::endl;
    std::cout << std::format("flags {:08B}", context.registers.f) << std::endl;
    std::cout << context.registers.get_flags_string() << std::endl;

    EXPECT_EQ(context.registers.f, cpu::HALF | cpu::CARRY);

    cpu::cpu_set_flag(context.registers.f, 0, 0, 0xFF, 0);
    std::cout << std::format("flags {:08B}", context.registers.f) << std::endl;
    EXPECT_EQ(context.registers.f, cpu::HALF);

    cpu::cpu_set_flag(context.registers.f, 1, 0, 0, 0);
    std::cout << std::format("flags {:08B}", context.registers.f) << std::endl;
    EXPECT_EQ(context.registers.f, cpu::ZERO);
}

};  // namespace gameboy::tests