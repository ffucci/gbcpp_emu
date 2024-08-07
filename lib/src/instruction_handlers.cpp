#include "cpu/instruction_handlers.h"
#include <cstdint>
#include <utility>
#include "cpu/utils.h"
#include "utils/logger.h"

namespace gameboy::cpu {

void sbc_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto cpu_flag_c = ctx.registers.c_flag();
    uint8_t val = ctx.fetched_data + cpu_flag_c;

    const uint16_t r1_val = ctx.read_reg(ctx.instruction.r1);
    int z = (r1_val - val == 0);
    int h = ((static_cast<int>(r1_val) & 0xF) - (static_cast<int>(ctx.fetched_data) & 0xF) -
             (static_cast<int>(cpu_flag_c))) < 0;
    int c = (static_cast<int>(r1_val) - static_cast<int>(ctx.fetched_data) - (static_cast<int>(cpu_flag_c))) < 0;

    ctx.set_reg(ctx.instruction.r1, r1_val - val);
    cpu_set_flag(ctx.registers.f, z, 1, h, c);
}

void sub_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint16_t val = ctx.read_reg(ctx.instruction.r1) - ctx.fetched_data;
    int z = (val == 0);
    int h = (static_cast<int>(ctx.read_reg(ctx.instruction.r1)) & 0xF) - (static_cast<int>(ctx.fetched_data) & 0xF) < 0;
    int c = static_cast<int>(ctx.read_reg(ctx.instruction.r1)) - static_cast<int>(ctx.fetched_data) < 0;

    ctx.set_reg(ctx.instruction.r1, val);
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
    uint32_t val = ctx.read_reg(ctx.instruction.r1) + ctx.fetched_data;

    [[maybe_unused]] bool is_16_bit = CPUContext::is_16_bit(ctx.instruction.r1);

    if (is_16_bit) {
        update_cycles(1, ctx, mmu);
    }

    if (ctx.instruction.r1 == RegisterType::SP) {
        val = ctx.read_reg(ctx.instruction.r1) + static_cast<char>(ctx.fetched_data);
    }

    int z = ((val & 0xFF) == 0);
    int h = (ctx.read_reg(ctx.instruction.r1) & 0xF) + (ctx.fetched_data & 0xF) >= 0x10;
    int c = (ctx.read_reg(ctx.instruction.r1) & 0xFF) + (ctx.fetched_data & 0xFF) >= 0x100;

    if (is_16_bit) {
        z = -1;
        h = (ctx.read_reg(ctx.instruction.r1) & 0xFFF) + (ctx.fetched_data & 0xFFF) >= 0x1000;
        uint32_t n = static_cast<uint32_t>(ctx.read_reg(ctx.instruction.r1)) + static_cast<uint32_t>(ctx.fetched_data);
        c = n >= 0x10000;
    }

    if (ctx.instruction.r1 == RegisterType::SP) {
        z = 0;
        h = (ctx.read_reg(ctx.instruction.r1) & 0xF) + (ctx.fetched_data & 0xF) >= 0x10;
        c =
            (static_cast<int>(ctx.read_reg(ctx.instruction.r1) & 0xFF) + static_cast<int>(ctx.fetched_data & 0xFF) >=
             0x100);
    }

    ctx.set_reg(ctx.instruction.r1, val & 0xFFFF);
    cpu_set_flag(ctx.registers.f, z, 0, h, c);
}
void dec_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint16_t val = ctx.read_reg(ctx.instruction.r1) - 1;

    if (CPUContext::is_16_bit(ctx.instruction.r1)) {
        update_cycles(1, ctx, memory);
    }

    auto& instruction = ctx.instruction;
    if (instruction.r1 == RegisterType::HL && instruction.mode == AddressingMode::MR) {
        auto address = ctx.read_reg(RegisterType::HL);
        val = memory.read(address) - 1;
        memory.write(address, val);
    } else {
        ctx.set_reg(instruction.r1, val);
        val = ctx.read_reg(instruction.r1);
    }

    if ((ctx.current_opcode & 0x0B) == 0x0B) {
        return;
    }

    cpu_set_flag(ctx.registers.f, val == 0, 1, ((val & 0x0F) == 0x0F), -1);
    return;
}
void inc_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto val = ctx.read_reg(ctx.instruction.r1) + 1;

    if (CPUContext::is_16_bit(ctx.instruction.r1)) {
        update_cycles(1, ctx, memory);
    }

    auto& instruction = ctx.instruction;
    if (instruction.r1 == RegisterType::HL && instruction.mode == AddressingMode::MR) {
        auto address = ctx.read_reg(RegisterType::HL);
        val = memory.read(address) + 1;
        val &= 0xFF;
        memory.write(address, val);
    } else {
        ctx.set_reg(ctx.instruction.r1, val);  // We set and read because the variable is saved with a mask
        val &= 0xFF;
    }

    if ((ctx.current_opcode & 0x03) == 0x03) {
        return;
    }

    const bool has_half = ((val & 0x0F) == 0);
    cpu_set_flag(ctx.registers.f, val == 0, 0, has_half, 0xFF);
    return;
}
void rst_handler(CPUContext& ctx, memory::MMU& memory)
{
    jump_to_addr(ctx, memory, ctx.instruction.parameter, true);
}
void reti_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.master_interrupt_enabled = true;
    ret_handler(ctx, memory);
}
void ret_handler(CPUContext& ctx, memory::MMU& memory)
{
    if (ctx.instruction.condition != ConditionType::None) {
        update_cycles(1, ctx, memory);
    }

    if (check_cond(ctx)) {
        const uint16_t lo = stack_pop(ctx, memory);
        update_cycles(1, ctx, memory);
        const uint16_t hi = stack_pop(ctx, memory);
        update_cycles(1, ctx, memory);

        const uint16_t ret_address = lo | (hi << 8);
        ctx.registers.pc = ret_address;
        update_cycles(1, ctx, memory);
    }
}
void call_handler(CPUContext& ctx, memory::MMU& memory)
{
    jump_to_addr(ctx, memory, ctx.fetched_data, true);
}
void jr_handler(CPUContext& ctx, memory::MMU& memory)
{
    int8_t offset = static_cast<int8_t>(ctx.fetched_data & 0xFF);
    const uint16_t address = ctx.registers.pc + offset;
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
            update_cycles(2, ctx, memory);
            stack_push16(ctx, memory, ctx.registers.pc);
        }

        ctx.registers.pc = addr;
        update_cycles(1, ctx, memory);
    }
}
void pop_handler(CPUContext& ctx, memory::MMU& memory)
{
    const auto lo = stack_pop(ctx, memory);
    update_cycles(1, ctx, memory);
    const auto hi = stack_pop(ctx, memory);
    update_cycles(1, ctx, memory);

    const auto value = lo | (hi << 8);
    const auto reg1 = ctx.instruction.r1;

    if (reg1 == RegisterType::AF) {
        ctx.set_reg(reg1, value & 0xFFF0);  // Takes the value from memory and puts into reg
        return;
    }

    ctx.set_reg(reg1, value);  // Takes the value from memory and puts into reg
}
void push_handler(CPUContext& ctx, memory::MMU& memory)
{
    const uint16_t value_to_push = ctx.read_reg(ctx.instruction.r1);
    uint16_t hi = (value_to_push >> 8) & 0xFF;
    update_cycles(1, ctx, memory);
    stack_push(ctx, memory, hi);

    uint16_t lo = (ctx.read_reg(ctx.instruction.r1)) & 0xFF;
    update_cycles(1, ctx, memory);
    stack_push(ctx, memory, lo);

    update_cycles(1, ctx, memory);
}
void ldh_handler(CPUContext& ctx, memory::MMU& memory)
{
    if (ctx.instruction.r1 == RegisterType::A) {
        ctx.set_reg(RegisterType::A, memory.read(0xFF00 | ctx.fetched_data));
    } else {
        memory.write(ctx.memory_destination, ctx.registers.a);
    }

    update_cycles(1, ctx, memory);
}
void di_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.master_interrupt_enabled = false;
}
void ld_handler(CPUContext& ctx, memory::MMU& memory)
{
    if (ctx.destination_is_mem) {
        if (CPUContext::is_16_bit(ctx.instruction.r2)) {
            update_cycles(1, ctx, memory);
            memory.write16(ctx.memory_destination, ctx.fetched_data);
        } else {
            memory.write(ctx.memory_destination, ctx.fetched_data);
        }

        update_cycles(1, ctx, memory);
        return;
    }

    if (ctx.instruction.mode == AddressingMode::HL_SPR) {
        // const auto reg2_val = ctx.read_reg(ctx.instruction.r2);

        uint8_t hflag = (ctx.read_reg(ctx.instruction.r2) & 0xF) + (ctx.fetched_data & 0xF) >= 0x10;
        uint8_t cflag = (ctx.read_reg(ctx.instruction.r2) & 0xFF) + (ctx.fetched_data & 0xFF) >= 0x100;

        cpu_set_flag(ctx.registers.f, 0, 0, hflag, cflag);
        ctx.set_reg(
            ctx.instruction.r1,
            ctx.read_reg(ctx.instruction.r2) + (char)(ctx.fetched_data));  // TODO: understand better.
        return;
    }

    // In other cases just take the fetched data and move to register 1
    ctx.set_reg(ctx.instruction.r1, ctx.fetched_data);

    if constexpr (DEBUG) {
        auto& logger = logger::Logger::instance();

        logger.log("Fetched data {:04X}", ctx.fetched_data);
    }
}
bool check_cond(CPUContext& context)
{
    bool z = context.registers.z_flag();
    bool c = context.registers.c_flag();
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
    regs.a ^= ctx.fetched_data & 0xFF;
    cpu_set_flag(regs.f, regs.a == 0, 0, 0, 0);
}
void and_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto& regs = ctx.registers;
    regs.a &= ctx.fetched_data;
    cpu_set_flag(regs.f, regs.a == 0, 0, 1, 0);
}
void or_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto& regs = ctx.registers;
    regs.a |= ctx.fetched_data;
    cpu_set_flag(regs.f, regs.a == 0, 0, 0, 0);
}
void cp_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto& regs = ctx.registers;
    auto n = static_cast<int>(regs.a) - static_cast<int>(ctx.fetched_data);
    auto h = (static_cast<int>(regs.a) & 0xF) - (static_cast<int>(ctx.fetched_data) & 0xF);

    cpu_set_flag(regs.f, n == 0, 1, h < 0, n < 0);
}
void cb_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto set_reg_internal = [&ctx, &memory](RegisterType reg, uint8_t value) mutable {
        if (reg == RegisterType::HL) {
            memory.write(ctx.read_reg(RegisterType::HL), value);
            return;
        }
        ctx.set_reg8(reg, value);
    };
    // Fetched data contains the register that we want to read
    auto op = ctx.fetched_data;
    auto reg = decode_reg(op & 0b111);
    uint8_t bit = (op >> 3) & 0b111;
    uint8_t bit_op = (op >> 6) & 0b11;  // bit operation that we perform

    auto value = ctx.read_reg8(reg);
    update_cycles(1, ctx, memory);

    if (reg == RegisterType::HL) {
        update_cycles(2, ctx, memory);
        value = memory.read(ctx.read_reg(RegisterType::HL));
    }

    switch (bit_op) {
        case 1:
            // bit
            // Z - Set if bit b of register r is 0.
            // N - Reset.
            // H - Set.
            // C - Not affected.
            cpu_set_flag(ctx.registers.f, !(value & (1 << bit)), 0, 1, 0xFF);
            return;
        case 2: {
            // RESET
            value &= ~(1 << bit);
            set_reg_internal(reg, value);
            return;
        }
        case 3:
            // SET
            value |= (1 << bit);
            set_reg_internal(reg, value);
            return;
    }

    bool flag_c = (ctx.registers.f & CARRY) != 0;

    switch (bit) {
        case 0: {
            // RLC (Rotate Left)
            bool set_c = false;
            uint8_t result = (value << 1) & 0xFF;  // Rotate left
            if ((value & ZERO) != 0) {
                result |= 1;
                set_c = true;
            }

            set_reg_internal(reg, result);
            cpu_set_flag(ctx.registers.f, result == 0, 0, 0, set_c);
        }
            return;
        case 1: {
            // RRC
            auto old = value;
            value >>= 1;
            value |= (old << 7);
            set_reg_internal(reg, value);
            cpu_set_flag(ctx.registers.f, !value, 0, 0, old & 0x1);
        }
            return;
        case 2: {
            // RL
            auto old = value;
            value <<= 1;
            value |= flag_c;
            set_reg_internal(reg, value);
            // TODO: understand this --> !!(old & 0x80)
            cpu_set_flag(ctx.registers.f, !value, 0, 0, !!(old & 0x80));
        }
            return;
        case 3: {
            // RR
            auto old = value;
            value >>= 1;
            value |= (flag_c << 7);
            set_reg_internal(reg, value);
            cpu_set_flag(ctx.registers.f, !value, 0, 0, (old & 0x1));
        }
            return;
        case 4: {
            // SLA
            uint8_t old = value;
            value <<= 1;
            set_reg_internal(reg, value);
            cpu_set_flag(ctx.registers.f, !value, 0, 0, !!(old & 0x80));
        }
            return;
        case 5: {
            // SRA
            uint8_t u = static_cast<int8_t>(value) >> 1;
            set_reg_internal(reg, u);
            cpu_set_flag(ctx.registers.f, !u, 0, 0, value & 0x1);
        }
            return;
        case 6: {
            // SWAP
            value = ((value & 0xF0) >> 4) | ((value & 0x0F) << 4);
            set_reg_internal(reg, value);
            cpu_set_flag(ctx.registers.f, value == 0, 0, 0, 0);
        }
            return;
        case 7: {
            // SRL
            uint8_t u = value >> 1;
            set_reg_internal(reg, u);
            cpu_set_flag(ctx.registers.f, !u, 0, 0, value & 0x1);
        }
            return;
        default:
            return;
    }

    throw std::runtime_error("Invalid CB instruction...");
    return;
}
void stack_push(CPUContext& context, memory::MMU& memory, uint8_t value)
{
    context.registers.sp--;
    memory.write(context.registers.sp, value);
}
void stack_push16(CPUContext& context, memory::MMU& memory, uint16_t value)
{
    stack_push(context, memory, (value >> 8) & 0xFF);
    stack_push(context, memory, value & 0xFF);
}
auto stack_pop(CPUContext& context, memory::MMU& memory) -> uint8_t
{
    return memory.read(context.registers.sp++);
}
auto stack_pop16(CPUContext& context, memory::MMU& memory) -> uint8_t
{
    const auto lo = stack_pop(context, memory);
    const auto hi = stack_pop(context, memory);
    return lo | (hi << 8);
}
void none_handler(CPUContext& ctx, memory::MMU& memory)
{
    throw std::runtime_error("CANNOT EXECUTE");
}
void nop_handler(CPUContext& ctx, memory::MMU& memory)
{
}
void rlca_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint8_t u = ctx.registers.a;
    bool c = (u >> 7) & 1;
    u = (u << 1) | c;  // 00000111 -> (00001110) | 0;
    ctx.registers.a = u;
    cpu_set_flag(ctx.registers.f, 0, 0, 0, c);
}

void rrca_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint8_t bit = ctx.registers.a & 1;
    ctx.registers.a >>= 1;
    ctx.registers.a |= (bit << 7);  // to wrap around

    cpu_set_flag(ctx.registers.f, 0, 0, 0, bit);
}
void rla_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint8_t u = ctx.registers.a;
    uint8_t cf = ctx.registers.c_flag();
    uint8_t c = (u >> 7) & 1;
    ctx.registers.a = (u << 1) | cf;
    cpu_set_flag(ctx.registers.f, 0, 0, 0, c);
}
void rra_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint8_t cf = ctx.registers.c_flag();
    uint8_t new_carry = ctx.registers.a & 0x1;
    ctx.registers.a >>= 1;
    ctx.registers.a |= (cf << 7);

    cpu_set_flag(ctx.registers.f, 0, 0, 0, new_carry);
}
void stop_handler(CPUContext& ctx, memory::MMU& memory)
{
    throw std::runtime_error("STOPPING....");
}
void daa_handler(CPUContext& ctx, memory::MMU& memory)
{
    int8_t u = 0;
    int fc = 0;

    auto& regs = ctx.registers;
    if (regs.h_flag() || (!regs.n_flag() && (regs.a & 0xF) > 9)) {
        u = 6;
    }

    if (regs.c_flag() || (!regs.n_flag() && regs.a > 0x99)) {
        u |= 0x60;
        fc = 1;
    }

    regs.a += regs.n_flag() ? -u : u;

    cpu_set_flag(ctx.registers.f, regs.a == 0, -1, 0, fc);
}
void cpl_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.registers.a = ~ctx.registers.a;
    cpu_set_flag(ctx.registers.f, -1, 1, 1, -1);
}
void scf_handler(CPUContext& ctx, memory::MMU& memory)
{
    cpu_set_flag(ctx.registers.f, -1, 0, 0, 1);
}
void ccf_handler(CPUContext& ctx, memory::MMU& memory)
{
    cpu_set_flag(ctx.registers.f, -1, 0, 0, ctx.registers.c_flag() ^ 1);
}
void halt_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.state = CPUState::HALT;
}
void ei_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.enabling_ime = true;
}
}  // namespace gameboy::cpu