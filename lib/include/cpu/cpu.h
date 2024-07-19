#pragma once

#include <sys/types.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <utility>

#include "cartridge/cartridge.h"
#include "cpu/executor.h"
#include "instructions.h"
#include "mmu/mmu.h"
#include "utils/logger.h"

namespace gameboy::cpu {

static constexpr uint8_t ZERO = {1 << 7};
static constexpr uint8_t N_SUB = {1 << 6};
static constexpr uint8_t HALF = {1 << 5};
static constexpr uint8_t CARRY = {1 << 4};

static constexpr bool DEBUG{false};

struct CPURegisters
{
    uint8_t a;

    // Highest 4 bytes are the Z, N, H, C flags
    uint8_t f;
    uint8_t b;
    uint8_t c;
    uint8_t d;
    uint8_t e;
    uint8_t h;
    uint8_t l;
    uint16_t pc;  // Program Counter
    uint16_t sp;  // Stack pointer

    [[nodiscard]] std::string get_flags_string() const noexcept
    {
        std::string ret(4, '-');

        if (f & CARRY) {
            ret[3] = 'C';
        }

        if (f & HALF) {
            ret[2] = 'H';
        }

        if (f & N_SUB) {
            ret[1] = 'N';
        }

        if (f & ZERO) {
            ret[0] = 'S';
        }

        return ret;
    }
};

inline auto get_initial_state() -> CPURegisters
{
    // DMG
    // return {0x1, 0, 0x0, 0x13, 0x00, 0xD8, 0x01, 0x4d, 0x100, 0xFFFE};
    return {0x1, 0, 0x0, 0x0, 0x00, 0x0, 0x0, 0x0, 0x100, 0x0};
}

enum class CPUState
{
    HALT,
    RUNNING
};

constexpr uint16_t reverse(uint16_t n)
{
    return ((n & 0xFF00) >> 8) | ((n & 0x00FF) << 8);
}

struct CPUContext
{
    CPURegisters registers;
    uint16_t fetched_data;
    uint16_t memory_destination;
    uint8_t current_opcode;

    Instruction instruction;
    CPUState state;

    bool interrupt_masked{false};
    bool destination_is_mem{false};

    // Read registry
    auto cpu_read_reg(RegisterType rt) -> uint16_t
    {
        using gameboy::cpu::RegisterType;
        switch (rt) {
            case RegisterType::A:
                return registers.a;
            case RegisterType::F:
                return registers.f;
            case RegisterType::B:
                return registers.b;
            case RegisterType::C:
                return registers.c;
            case RegisterType::D:
                return registers.d;
            case RegisterType::E:
                return registers.e;
            case RegisterType::H:
                return registers.h;
            case RegisterType::L:
                return registers.l;

            case RegisterType::AF:
                return reverse(*((uint16_t*)&registers.a));
            case RegisterType::BC:
                return reverse(*((uint16_t*)&registers.b));
            case RegisterType::DE:
                return reverse(*((uint16_t*)&registers.d));
            case RegisterType::HL:
                return reverse(*((uint16_t*)&registers.h));
            case RegisterType::PC:
                return registers.pc;
            case RegisterType::SP:
                return registers.sp;
            default:
                return 0;
        }
    }

    void cpu_set_reg(RegisterType rt, uint16_t val)
    {
        switch (rt) {
            case RegisterType::A:
                registers.a = val & 0xFF;
                break;
            case RegisterType::F:
                registers.f = val & 0xFF;
                break;
            case RegisterType::B:
                registers.b = val & 0xFF;
                break;
            case RegisterType::C: {
                registers.c = val & 0xFF;
            } break;
            case RegisterType::D:
                registers.d = val & 0xFF;
                break;
            case RegisterType::E:
                registers.e = val & 0xFF;
                break;
            case RegisterType::H:
                registers.h = val & 0xFF;
                break;
            case RegisterType::L:
                registers.l = val & 0xFF;
                break;

            case RegisterType::AF:
                *((uint16_t*)&registers.a) = reverse(val);
                break;
            case RegisterType::BC:
                *((uint16_t*)&registers.b) = reverse(val);
                break;
            case RegisterType::DE:
                *((uint16_t*)&registers.d) = reverse(val);
                break;
            case RegisterType::HL: {
                *((uint16_t*)&registers.h) = reverse(val);
                break;
            }

            case RegisterType::PC:
                registers.pc = val;
                break;
            case RegisterType::SP:
                registers.sp = val;
                break;
            case RegisterType::None:
                break;

            default:
                throw std::runtime_error("Invalid register type...");
        }
    }

    static auto is_16_bit(RegisterType rt)
    {
        return rt >= RegisterType::AF;
    }
};

inline void stack_push(CPUContext& context, memory::MMU& memory, uint8_t value)
{
    context.registers.sp--;
    memory.write(context.registers.sp, value);
}

inline void stack_push16(CPUContext& context, memory::MMU& memory, uint16_t value)
{
    stack_push(context, memory, (value >> 8) & 0xFF);
    stack_push(context, memory, value & 0xFF);
}

inline auto stack_pop(CPUContext& context, memory::MMU& memory) -> uint8_t
{
    return memory.read(context.registers.sp++);
}

inline auto stack_pop16(CPUContext& context, memory::MMU& memory) -> uint8_t
{
    const auto lo = stack_pop(context, memory);
    const auto hi = stack_pop(context, memory);
    return lo | (hi << 8);
}

using Handler = void (*)(CPUContext& context, memory::MMU&);
using ExecutorsTable = std::array<Handler, NUM_INSTRUCTION_TYPES>;

inline void none_handler(CPUContext& ctx, memory::MMU& memory)
{
    throw std::runtime_error("CANNOT EXECUTE");
}

inline void nop_handler(CPUContext& ctx, memory::MMU& memory)
{
}

inline void cpu_set_flag(uint8_t& flags, uint8_t zero, uint8_t n, uint8_t half, uint8_t carry)
{
    const auto bz = (zero << 7);
    const auto bn = (n << 6);
    const auto bh = (half << 5);
    const auto bc = (carry << 4);

    flags = (flags & ~bz) | (-(zero != 0xFF) & bz);
    flags = (flags & ~bn) | (-(n != 0xFF) & bn);
    flags = (flags & ~bh) | (-(half != 0xFF) & bh);
    flags = (flags & ~bc) | (-(carry != 0xFF) & bc);
}

inline void xor_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto& regs = ctx.registers;
    regs.a ^= ctx.fetched_data;
    cpu_set_flag(regs.f, regs.a == 0, 0, 0, 0);
}

inline bool check_cond(CPUContext& context)
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

inline void ld_handler(CPUContext& ctx, memory::MMU& memory)
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

inline void di_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.interrupt_masked = true;
}

inline void ldh_handler(CPUContext& ctx, memory::MMU& memory)
{
    if (ctx.instruction.r1 == RegisterType::A) {
        ctx.cpu_set_reg(RegisterType::A, memory.read(0xFF00 | ctx.fetched_data));
    } else {
        memory.write(ctx.memory_destination, ctx.registers.a);
    }

    // emu_cycles(1);
}

inline void push_handler(CPUContext& ctx, memory::MMU& memory)
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

inline void pop_handler(CPUContext& ctx, memory::MMU& memory)
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

inline void jump_to_addr(CPUContext& ctx, memory::MMU& memory, uint16_t addr, bool push_pc)
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

inline void jp_handler(CPUContext& ctx, memory::MMU& memory)
{
    jump_to_addr(ctx, memory, ctx.fetched_data, false);
}

inline void jr_handler(CPUContext& ctx, memory::MMU& memory)
{
    auto offset = static_cast<char>(ctx.fetched_data & 0xFF);
    const auto address = ctx.registers.pc + offset;
    jump_to_addr(ctx, memory, address, false);
}

inline void call_handler(CPUContext& ctx, memory::MMU& memory)
{
    jump_to_addr(ctx, memory, ctx.fetched_data, true);
}

inline void ret_handler(CPUContext& ctx, memory::MMU& memory)
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

// returning from interrupt
inline void reti_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.interrupt_masked = true;
    ret_handler(ctx, memory);
}

inline void rst_handler(CPUContext& ctx, memory::MMU& memory)
{
    jump_to_addr(ctx, memory, ctx.instruction.parameter, true);
}

inline void inc_handler(CPUContext& ctx, memory::MMU& memory)
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

inline void dec_handler(CPUContext& ctx, memory::MMU& memory)
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

inline void add_handler(CPUContext& ctx, memory::MMU& mmu)
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

inline void adc_handler(CPUContext& ctx, memory::MMU& mmu)
{
    uint16_t u = ctx.fetched_data;
    uint16_t a = ctx.registers.a;
    uint16_t c = (ctx.registers.f & CARRY) != 0;

    ctx.registers.a = (u + a + c) & 0xFF;

    auto h = (a & 0xF) + (u & 0xF) + c > 0xF;
    auto carry = (a + u + c) > 0xFF;

    cpu_set_flag(ctx.registers.f, ctx.registers.a == 0, 0, h, carry);
}

inline void sub_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint16_t val = ctx.cpu_read_reg(ctx.instruction.r1) - ctx.fetched_data;
    int z = (val == 0);
    int h =
        (static_cast<int>(ctx.cpu_read_reg(ctx.instruction.r1)) & 0xF) - (static_cast<int>(ctx.fetched_data) & 0xF) < 0;
    int c = static_cast<int>(ctx.cpu_read_reg(ctx.instruction.r1)) - static_cast<int>(ctx.fetched_data) < 0;

    ctx.cpu_set_reg(ctx.instruction.r1, val);
    cpu_set_flag(ctx.registers.f, z, 1, h, c);
}

inline void sbc_handler(CPUContext& ctx, memory::MMU& memory)
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

static constexpr auto make_executors_table() -> ExecutorsTable
{
    ExecutorsTable table_{};
    std::fill(begin(table_), end(table_), none_handler);
    table_[std::to_underlying(InstructionType::NOP)] = nop_handler;
    table_[std::to_underlying(InstructionType::XOR)] = xor_handler;
    table_[std::to_underlying(InstructionType::JP)] = jp_handler;
    table_[std::to_underlying(InstructionType::JR)] = jr_handler;
    table_[std::to_underlying(InstructionType::INC)] = inc_handler;
    table_[std::to_underlying(InstructionType::DEC)] = dec_handler;

    // The details of these instructions need to be understood better
    table_[std::to_underlying(InstructionType::ADD)] = add_handler;
    table_[std::to_underlying(InstructionType::ADC)] = adc_handler;
    table_[std::to_underlying(InstructionType::SUB)] = sub_handler;
    table_[std::to_underlying(InstructionType::SBC)] = sbc_handler;

    // Going to subroutines opcodes (CALL, RET, ...)
    table_[std::to_underlying(InstructionType::CALL)] = call_handler;
    table_[std::to_underlying(InstructionType::RET)] = ret_handler;
    table_[std::to_underlying(InstructionType::RETI)] = reti_handler;
    table_[std::to_underlying(InstructionType::RST)] = rst_handler;
    table_[std::to_underlying(InstructionType::LD)] = ld_handler;
    table_[std::to_underlying(InstructionType::LDH)] = ldh_handler;
    table_[std::to_underlying(InstructionType::PUSH)] = push_handler;
    table_[std::to_underlying(InstructionType::POP)] = pop_handler;
    table_[std::to_underlying(InstructionType::DI)] = di_handler;
    return table_;
}

class CPU
{
   public:
    CPU(CPURegisters initial_state, cartridge::Cartridge& cartridge)
        : context_{.registers = std::move(initial_state)}, memory_(cartridge)
    {
    }

    void run()
    {
        context_.state = CPUState::RUNNING;
        auto& regs = context_.registers;

        while (context_.state != CPUState::HALT) {
            auto pc = regs.pc;
            context_.instruction = fetch_instruction();
            fetch_data(context_.instruction);
            auto& logger = logger::Logger::instance();

            logger.log(
                "{:08X} - {:04X}: {} ({:02X}, {:02X}, {:02X}) \tA: {:02X}, BC:{:02X}{:02X}, DE:{:02X}{:02X}, "
                "HL:{:02X}{:02X}, "
                "F:{}",
                ticks_, pc, get_instruction_name(context_.instruction.type), context_.current_opcode,
                memory_.read(pc + 1), memory_.read(pc + 2), regs.a, regs.b, regs.c, regs.d, regs.e, regs.h, regs.l,
                regs.get_flags_string());

            if (context_.instruction.type == InstructionType::NONE) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                throw std::runtime_error("Instruction is not valid or not yet added.");
            }

            execute(context_.instruction);
            ticks_++;
        }
    }

   private:
    auto fetch_instruction() noexcept -> const Instruction&
    {
        auto& registers = context_.registers;
        context_.current_opcode = memory_.read(registers.pc++);
        return instruction_set_[context_.current_opcode];
    }

    auto fetch_data(const Instruction& instruction) -> void
    {
        context_.memory_destination = 0;
        context_.destination_is_mem = false;

        const auto rd8_handler = [this]() {
            auto& regs = context_.registers;
            context_.fetched_data = memory_.read(regs.pc);
            // auto& logger = logger::Logger::instance();
            // logger.log("Fetched data... {:02X}", context_.fetched_data);
            regs.pc++;
            return;
        };

        const auto d_16_mode_handler = [this]() {
            auto& regs = context_.registers;

            auto lo = memory_.read(regs.pc);
            // emu_cycles(1)
            auto hi = memory_.read(regs.pc + 1);
            // emu_cycles(1)

            context_.fetched_data = (lo | (hi << 8));
            regs.pc += 2;
            return;
        };

        switch (instruction.mode) {
            case AddressingMode::IMP: {
                return;
            }
            case AddressingMode::R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r1);
                return;
            };
            case AddressingMode::R_R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r2);
                return;
            }

            case AddressingMode::R_MR: {
                auto addr = context_.cpu_read_reg(instruction.r2);
                if (instruction.r1 == RegisterType::C) {
                    addr |= 0xFF00;
                }

                context_.fetched_data = memory_.read(addr);
                // emu_cycles(1);
                return;
            }
            // memory register to registers
            case AddressingMode::MR_R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r2);
                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;  // understand how to optimize

                if (instruction.r1 == RegisterType::C) {
                    // If the memory destination is contained in register C then we need
                    // to set the higher bits to 1.
                    context_.memory_destination |= 0xFF00;
                }

                return;
            }
            case AddressingMode::R_D8: {
                rd8_handler();
                return;
            }
            case AddressingMode::R_HLI: {
                context_.fetched_data = memory_.read(context_.cpu_read_reg(instruction.r2));
                // emu_cycles(1)
                const auto new_val = context_.cpu_read_reg(RegisterType::HL) + 1;
                context_.cpu_set_reg(RegisterType::HL, new_val);
                return;
            }
            case AddressingMode::R_HLD: {
                context_.fetched_data = memory_.read(context_.cpu_read_reg(instruction.r2));
                // emu_cycles(1)
                const auto new_val = context_.cpu_read_reg(RegisterType::HL) - 1;
                context_.cpu_set_reg(RegisterType::HL, new_val);
                return;
            }

            case AddressingMode::HLI_R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r2);  // What to increment read from registry
                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;
                const auto new_val = context_.cpu_read_reg(RegisterType::HL) + 1;
                context_.cpu_set_reg(RegisterType::HL, new_val);
                return;
            }
            case AddressingMode::HLD_R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r2);  // What to increment read from registry
                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;
                const auto new_val = context_.cpu_read_reg(RegisterType::HL) - 1;
                context_.cpu_set_reg(RegisterType::HL, new_val);
                return;
            }

            case AddressingMode::D8:
            case AddressingMode::R_A8: {
                context_.fetched_data = memory_.read(context_.registers.pc);
                // emu_cycles(1)
                context_.registers.pc++;
                return;
            }
            case AddressingMode::A8_R: {
                context_.memory_destination = memory_.read(context_.registers.pc) | 0xFF00;
                context_.destination_is_mem = true;
                // emu_cycles(1)

                context_.registers.pc++;
                return;
            }
            case AddressingMode::HL_SPR: {
                // load stack pointer
                context_.fetched_data = memory_.read(context_.registers.pc);
                // emu_cycles(1)
                context_.registers.pc++;
                return;
            }
            case AddressingMode::R_D16:
                [[fallthrough]];
            case AddressingMode::D16: {
                d_16_mode_handler();
                break;
            }

            case AddressingMode::A16_R:
                [[fallthrough]];
            case AddressingMode::D16_R: {
                auto& regs = context_.registers;

                auto lo = memory_.read(regs.pc);
                // emu_cycles(1)
                auto hi = memory_.read(regs.pc + 1);
                // emu_cycles(1)

                context_.memory_destination = (lo | (hi << 8));
                context_.destination_is_mem = true;
                regs.pc += 2;
                context_.fetched_data = memory_.read(context_.cpu_read_reg(instruction.r2));
                return;
            }
            case AddressingMode::MR_D8: {
                auto& regs = context_.registers;

                context_.fetched_data = memory_.read(regs.pc);
                // emu_cycles(1)
                regs.pc++;
                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;
                return;
            }
            case AddressingMode::MR: {
                auto& regs = context_.registers;

                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;
                context_.fetched_data = memory_.read(context_.memory_destination);
                // emu_cycles(1)
                return;
            }
            case AddressingMode::R_A16: {
                auto& regs = context_.registers;

                auto lo = memory_.read(regs.pc);
                // emu_cycles(1)
                auto hi = memory_.read(regs.pc + 1);
                // emu_cycles(1)

                auto address = (lo | (hi << 8));
                regs.pc += 2;
                context_.fetched_data = memory_.read(address);
                // emu_cycles(1)
                return;
            }
            default:
                throw std::runtime_error("Unknown addressing mode");
        }
    }

    void execute(const Instruction& instruction)
    {
        if constexpr (DEBUG) {
            auto& logger = logger::Logger::instance();
            logger.log("OPC: {:#x} , PC: {:#x} ", context_.current_opcode, context_.registers.pc);
        }
        executors_[std::to_underlying(instruction.type)](context_, memory_);
    }

    CPUContext context_;
    memory::MMU memory_;

    size_t ticks_{0};
    static constexpr auto instruction_set_ = initialize_instruction_set();
    static constexpr auto executors_ = make_executors_table();
};
}  // namespace gameboy::cpu
