#include "cpu/instruction_handlers.h"

namespace gameboy::cpu {

void sbc_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto cpu_flag_c = ((ctx.registers.f & CARRY) != 0);
    uint16_t val = ctx.fetched_data + cpu_flag_c;

    const auto r1_val = ctx.cpu_read_reg(ctx.instruction.r1);
    int z = (r1_val - val == 0);
    int h =
        (static_cast<int>(r1_val) & 0xF) - (static_cast<int>(ctx.fetched_data) & 0xF) - (static_cast<int>(cpu_flag_c)) <
        0;
    int c = static_cast<int>(r1_val) - static_cast<int>(ctx.fetched_data) - (static_cast<int>(cpu_flag_c)) < 0;

    ctx.cpu_set_reg(ctx.instruction.r1, r1_val - val);
    cpu_set_flag(ctx.registers.f, z, 1, h, c);
}

void sub_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint16_t val = ctx.cpu_read_reg(ctx.instruction.r1) - ctx.fetched_data;
    int z = (val == 0);
    int h =
        (static_cast<int>(ctx.cpu_read_reg(ctx.instruction.r1)) & 0xF) - (static_cast<int>(ctx.fetched_data) & 0xF) < 0;
    int c = static_cast<int>(ctx.cpu_read_reg(ctx.instruction.r1)) - static_cast<int>(ctx.fetched_data) < 0;

    ctx.cpu_set_reg(ctx.instruction.r1, val);
    cpu_set_flag(ctx.registers.f, z, 1, h, c);
}
void adc_handler(CPUContext& ctx, memory::MMU& mmu)
{
    uint16_t u = ctx.fetched_data;
    uint16_t a = ctx.registers.a;
    uint16_t c = (ctx.registers.f & CARRY) != 0;

    ctx.registers.a = (u + a + c) & 0xFF;

    auto h = (a & 0xF) + (u & 0xF) + c > 0xF;
    auto carry = (a + u + c) > 0xFF;

    cpu_set_flag(ctx.registers.f, ctx.registers.a == 0, 0, h, carry);
}
void add_handler(CPUContext& ctx, memory::MMU& mmu)
{
    uint32_t val = ctx.cpu_read_reg(ctx.instruction.r1) + ctx.fetched_data;

    [[maybe_unused]] bool is_16_bit = CPUContext::is_16_bit(ctx.instruction.r1);

    // if(is_16_bit)
    // {
    //     emu_cycles(1);
    // }

    if (ctx.instruction.r1 == RegisterType::SP) {
        val = ctx.cpu_read_reg(ctx.instruction.r1) + static_cast<char>(ctx.fetched_data);
    }

    int z = (val & 0xFF) == 0;
    int h = (ctx.cpu_read_reg(ctx.instruction.r1) & 0xF) + (ctx.fetched_data & 0xF) >= 0x10;
    int c = (ctx.cpu_read_reg(ctx.instruction.r1) & 0xFF) + (ctx.fetched_data & 0xFF) >= 0x100;

    if (is_16_bit) {
        z = 0xFF;
        h = (ctx.cpu_read_reg(ctx.instruction.r1) & 0xFFF) + (ctx.fetched_data & 0xFFF) >= 0x1000;
        uint32_t n =
            static_cast<uint32_t>(ctx.cpu_read_reg(ctx.instruction.r1)) + static_cast<uint32_t>(ctx.fetched_data);
        c = n >= 0x10000;
    }

    if (ctx.instruction.r1 == RegisterType::SP) {
        z = 0;
        h = (ctx.cpu_read_reg(ctx.instruction.r1) & 0xF) + (ctx.fetched_data & 0xF) >= 0x10;
        c = (ctx.cpu_read_reg(ctx.instruction.r1) & 0xFF) + (ctx.fetched_data & 0xFF) >= 0x100;
    }

    ctx.cpu_set_reg(ctx.instruction.r1, val & 0xFFFF);
    cpu_set_flag(ctx.registers.f, z, 0, h, c);
}
void dec_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto val = ctx.cpu_read_reg(ctx.instruction.r1) - 1;

    if (CPUContext::is_16_bit(ctx.instruction.r2)) {
        // emu_cycles(1); add one cycle
    }

    auto& instruction = ctx.instruction;
    if (instruction.r1 == RegisterType::HL && instruction.mode == AddressingMode::MR) {
        auto address = ctx.cpu_read_reg(RegisterType::HL);
        auto val = memory.read(address) - 1;
        val &= 0xFF;
        memory.write(address, val);
    } else {
        ctx.cpu_set_reg(ctx.instruction.r1, val);
    }

    if ((ctx.current_opcode & 0x0B) == 0x0B) {
        return;
    }

    // TODO: Correct CPU Set flags (which is not totally correct)
    cpu_set_flag(ctx.registers.f, val == 0, 1, (val & 0x0F) == 0x0F, 0xFF);
    return;
}
void inc_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto val = ctx.cpu_read_reg(ctx.instruction.r1) + 1;

    if (CPUContext::is_16_bit(ctx.instruction.r2)) {
        // emu_cycles(1); add one cycle
    }

    auto& instruction = ctx.instruction;
    if (instruction.r1 == RegisterType::HL && instruction.mode == AddressingMode::MR) {
        auto address = ctx.cpu_read_reg(RegisterType::HL);
        auto val = memory.read(address) + 1;
        val &= 0xFF;
        memory.write(address, val);
    } else {
        ctx.cpu_set_reg(ctx.instruction.r1, val);
    }

    if ((ctx.current_opcode & 0x03) == 0x03) {
        return;
    }

    cpu_set_flag(ctx.registers.f, val == 0, 0, (val & 0xF) == 0, 0xFF);
    return;
}
void rst_handler(CPUContext& ctx, memory::MMU& memory)
{
    jump_to_addr(ctx, memory, ctx.instruction.parameter, true);
}
void reti_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.interrupt_masked = true;
    ret_handler(ctx, memory);
}
void ret_handler(CPUContext& ctx, memory::MMU& memory)
{
    // if (ctx.instruction.condition != ConditionType::None) {
    //     emu_cycles(1);
    // }

    if (check_cond(ctx)) {
        const uint16_t lo = stack_pop(ctx, memory);
        // emu_cycles(1);
        const uint16_t hi = stack_pop(ctx, memory);
        // emu_cycles(1);

        const uint16_t ret_address = lo | (hi << 8);
        ctx.registers.pc = ret_address;
        std::cout << std::format("{:04X}", ret_address) << std::endl;
        // emu_cycles(1);
    }
}
void call_handler(CPUContext& ctx, memory::MMU& memory)
{
    jump_to_addr(ctx, memory, ctx.fetched_data, true);
}
void jr_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto offset = static_cast<char>(ctx.fetched_data & 0xFF);
    const auto address = ctx.registers.pc + offset;
    jump_to_addr(ctx, memory, address, false);
}
void jp_handler(CPUContext& ctx, memory::MMU& memory)
{
    jump_to_addr(ctx, memory, ctx.fetched_data, false);
}
void jump_to_addr(CPUContext& ctx, memory::MMU& memory, uint16_t addr, bool push_pc)
{
    if (check_cond(ctx)) {
        if (push_pc) {
            // emu_cycles(2);
            stack_push16(ctx, memory, ctx.registers.pc);
        }

        ctx.registers.pc = addr;
        // emu_cycles(1)
    }
}
void pop_handler(CPUContext& ctx, memory::MMU& memory)
{
    const auto lo = stack_pop(ctx, memory);
    // emu_cycles(1);
    const auto hi = stack_pop(ctx, memory);
    // emu_cycles(1);

    const auto value = lo | (hi << 8);
    const auto reg1 = ctx.instruction.r1;

    if (reg1 == RegisterType::AF) {
        ctx.cpu_set_reg(reg1, value & 0xFFF0);  // Takes the value from memory and puts into reg
        return;
    }

    ctx.cpu_set_reg(reg1, value);  // Takes the value from memory and puts into reg
}
void push_handler(CPUContext& ctx, memory::MMU& memory)
{
    const auto value_to_push = ctx.cpu_read_reg(ctx.instruction.r1);
    auto hi = (value_to_push >> 8) & 0xFF;
    // emu_cycles(1);
    stack_push(ctx, memory, hi);

    auto lo = (ctx.cpu_read_reg(ctx.instruction.r1)) & 0xFF;
    // emu_cycles(1);
    stack_push(ctx, memory, lo);

    // emu_cycles(1);
}
void ldh_handler(CPUContext& ctx, memory::MMU& memory)
{
    if (ctx.instruction.r1 == RegisterType::A) {
        ctx.cpu_set_reg(RegisterType::A, memory.read(0xFF00 | ctx.fetched_data));
    } else {
        memory.write(ctx.memory_destination, ctx.registers.a);
    }

    // emu_cycles(1);
}
void di_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.interrupt_masked = true;
}
void ld_handler(CPUContext& ctx, memory::MMU& memory)
{
    if (ctx.destination_is_mem) {
        if (CPUContext::is_16_bit(ctx.instruction.r2)) {
            // emu_cycles(1)
            memory.write16(ctx.memory_destination, ctx.fetched_data);
        } else {
            memory.write(ctx.memory_destination, ctx.fetched_data);
        }

        // emu_cycles(1);
        return;
    }

    if (ctx.instruction.mode == AddressingMode::HL_SPR) {
        const auto reg2_val = ctx.cpu_read_reg(ctx.instruction.r2);

        auto hflag = (reg2_val & 0xF) + (ctx.fetched_data & 0xF) >= 0x10;
        auto cflag = (reg2_val & 0xFF) + (ctx.fetched_data & 0xFF) >= 0x100;

        cpu_set_flag(ctx.registers.f, 0, 0, hflag, cflag);
        ctx.cpu_set_reg(ctx.instruction.r1, reg2_val + (char)(ctx.fetched_data));  // TODO: understand better.
        return;
    }

    auto& logger = logger::Logger::instance();
    // In other cases just take the fetched data and move to register 1
    ctx.cpu_set_reg(ctx.instruction.r1, ctx.fetched_data);

    if constexpr (DEBUG) {
        logger.log("---------> LD {:02X} reg, {:02X} data", std::to_underlying(ctx.instruction.r1), ctx.fetched_data);
        logger.log("---------> Reg data, {:02X}", ctx.cpu_read_reg(ctx.instruction.r1));
        logger.log("---------> Reg data2, {:X}", ctx.registers.l);
        logger.log(
            "{:04X}: {} ({:02X}, {:02X}, {:02X}), A: {:02X}, BC:{:02X}{:02X}, DE:{:02X}{:02X}, HL:{:02X}{:02X}, "
            "F:{:b}",
            ctx.registers.pc, get_instruction_name(ctx.instruction.type), ctx.current_opcode,
            memory.read(ctx.registers.pc + 1), memory.read(ctx.registers.pc + 2), ctx.registers.a, ctx.registers.b,
            ctx.registers.c, ctx.registers.d, ctx.registers.e, ctx.registers.h, ctx.registers.l, ctx.registers.f);
    }
}
bool check_cond(CPUContext& context)
{
    bool z = (context.registers.f & ZERO) != 0;
    bool c = (context.registers.f & CARRY) != 0;
    switch (context.instruction.condition) {
        case ConditionType::None:
            return true;
        case ConditionType::C:
            return c;
        case ConditionType::NC:
            return !c;
        case ConditionType::Z:
            return z;
        case ConditionType::NZ:
            return !z;
    }

    return false;
}
void xor_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto& regs = ctx.registers;
    regs.a ^= ctx.fetched_data;
    cpu_set_flag(regs.f, regs.a == 0, 0, 0, 0);
}
}  // namespace gameboy::cpu