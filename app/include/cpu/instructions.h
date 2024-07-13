#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <utility>

namespace gameboy::cpu {
enum class RegisterType : uint8_t
{
    None,
    A,
    F,
    B,
    C,
    D,
    E,
    H,
    L,
    AF,
    BC,
    DE,
    HL,
    SP,
    PC
};

enum class ConditionType : uint8_t
{
    None,
    NZ,  // Non Zero
    Z,
    NC,
    C
};

enum class AddressingMode : uint8_t
{
    IMP,
    R_D16,
    R_R,
    MR_R,
    R,
    R_D8,
    R_MR,
    R_HLI,
    R_HLD,
    HLI_R,
    HLD_R,
    R_A8,
    A8_R,
    HL_SPR,
    D16,
    D8,
    D16_R,
    MR_D8,
    MR,
    A16_R,
    R_A16
};

enum class InstructionType : uint8_t
{
    NONE,
    NOP,
    LD,
    INC,
    DEC,
    RLCA,
    ADD,
    RRCA,
    STOP,
    RLA,
    JR,
    RRA,
    DAA,
    CPL,
    SCF,
    CCF,
    HALT,
    ADC,
    SUB,
    SBC,
    AND,
    XOR,
    OR,
    CP,
    POP,
    JP,
    PUSH,
    RET,
    CB,
    CALL,
    RETI,
    LDH,
    JPHL,
    DI,
    EI,
    RST,
    ERR,
    // CB Prefix Instruction
    RLC,
    RRC,
    RL,
    RR,
    SLA,
    SRA,
    SWAP,
    SRL,
    BIT,
    RES,
    SET
};

struct Instruction
{
    InstructionType type;
    AddressingMode mode;
    RegisterType r1;
    RegisterType r2;
    ConditionType condition;
    uint8_t parameter;
};

constexpr std::array<Instruction, 0x100> initialize_instruction_set()
{
    std::array<Instruction, 0x100> all_instructions{};
    all_instructions[0x00] = {InstructionType::NOP, AddressingMode::IMP};
    all_instructions[0x05] = {InstructionType::DEC, AddressingMode::R, RegisterType::B};
    all_instructions[0x0E] = {InstructionType::LD, AddressingMode::R_D8, RegisterType::C};

    // XOR instruction
    all_instructions[0xAF] = {InstructionType::XOR, AddressingMode::R, RegisterType::A};
    all_instructions[0xC3] = {InstructionType::XOR, AddressingMode::D16};
    all_instructions[0xF3] = {InstructionType::DI};

    return all_instructions;
}

static constexpr std::array<std::string_view, 48> instruction_names = {
    "<NONE>", "NOP",    "LD",     "INC",     "DEC",    "RLCA",   "ADD",    "RRCA",   "STOP",   "RLA",
    "JR",     "RRA",    "DAA",    "CPL",     "SCF",    "CCF",    "HALT",   "ADC",    "SUB",    "SBC",
    "AND",    "XOR",    "OR",     "CP",      "POP",    "JP",     "PUSH",   "RET",    "CB",     "CALL",
    "RETI",   "LDH",    "JPHL",   "DI",      "EI",     "RST",    "IN_ERR", "IN_RLC", "IN_RRC", "IN_RL",
    "IN_RR",  "IN_SLA", "IN_SRA", "IN_SWAP", "IN_SRL", "IN_BIT", "IN_RES", "IN_SET"};

// This returns the instruction name
constexpr auto get_instruction_name(InstructionType instr_type)
{
    return instruction_names[std::to_underlying(instr_type)];
}

};  // namespace gameboy::cpu