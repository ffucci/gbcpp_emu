#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <utility>

namespace gameboy::cpu {

static constexpr uint16_t NUM_INSTRUCTION_TYPES = 48;

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

    // LD 0x
    all_instructions[0x01] = {InstructionType::LD, AddressingMode::R_D16, RegisterType::BC};
    all_instructions[0x02] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::BC, RegisterType::A};
    all_instructions[0x05] = {InstructionType::DEC, AddressingMode::R, RegisterType::B};

    all_instructions[0x06] = {InstructionType::LD, AddressingMode::R_D8, RegisterType::B};
    all_instructions[0x08] = {InstructionType::LD, AddressingMode::A16_R, RegisterType::None, RegisterType::SP};
    all_instructions[0xA] = {InstructionType::LD, AddressingMode::R_MR, RegisterType::A, RegisterType::BC};
    all_instructions[0x0E] = {InstructionType::LD, AddressingMode::R_D8, RegisterType::C};

    // 0x1x
    all_instructions[0x11] = {InstructionType::LD, AddressingMode::R_D16, RegisterType::DE};
    all_instructions[0x12] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::DE, RegisterType::A};
    all_instructions[0x15] = {InstructionType::DEC, AddressingMode::R, RegisterType::D};
    all_instructions[0x16] = {InstructionType::LD, AddressingMode::R_D8, RegisterType::B};
    all_instructions[0x18] = {InstructionType::JR, AddressingMode::D8};

    all_instructions[0x1A] = {InstructionType::LD, AddressingMode::R_MR, RegisterType::A, RegisterType::DE};
    all_instructions[0x1E] = {InstructionType::LD, AddressingMode::R_D8, RegisterType::E};

    // 0x2x
    all_instructions[0x20] = {
        InstructionType::JR, AddressingMode::D8, RegisterType::None, RegisterType::None, ConditionType::NZ};

    all_instructions[0x21] = {InstructionType::LD, AddressingMode::R_D16, RegisterType::HL};
    all_instructions[0x22] = {InstructionType::LD, AddressingMode::HLI_R, RegisterType::HL, RegisterType::A};
    all_instructions[0x25] = {InstructionType::DEC, AddressingMode::R, RegisterType::H};
    all_instructions[0x26] = {InstructionType::LD, AddressingMode::R_D8, RegisterType::H};

    // (0x28) -> JR Z, r8
    all_instructions[0x28] = {
        InstructionType::JR, AddressingMode::D8, RegisterType::None, RegisterType::None, ConditionType::Z};

    all_instructions[0x2A] = {InstructionType::LD, AddressingMode::R_HLI, RegisterType::A, RegisterType::HL};
    all_instructions[0x2E] = {InstructionType::LD, AddressingMode::R_D8, RegisterType::L};

    // 0x3x
    // -------------------------------------------------------------------------------------------------------------------------------------
    //
    // (0x30) -> JR NC, r8
    all_instructions[0x30] = {
        InstructionType::JR, AddressingMode::D8, RegisterType::None, RegisterType::None, ConditionType::NC};

    all_instructions[0x31] = {InstructionType::LD, AddressingMode::R_D16, RegisterType::SP};
    all_instructions[0x32] = {InstructionType::LD, AddressingMode::HLD_R, RegisterType::HL, RegisterType::A};
    all_instructions[0x35] = {InstructionType::DEC, AddressingMode::R, RegisterType::HL};
    all_instructions[0x36] = {InstructionType::LD, AddressingMode::MR_D8, RegisterType::HL};

    // JR C, r8
    all_instructions[0x38] = {
        InstructionType::JR, AddressingMode::D8, RegisterType::None, RegisterType::None, ConditionType::C};

    all_instructions[0x3A] = {InstructionType::LD, AddressingMode::R_HLD, RegisterType::A, RegisterType::HL};
    all_instructions[0x3E] = {InstructionType::LD, AddressingMode::R_D8, RegisterType::A};

    // 0x5
    all_instructions[0x50] = {InstructionType::LD, AddressingMode::R_R, RegisterType::D, RegisterType::B};
    all_instructions[0x51] = {InstructionType::LD, AddressingMode::R_R, RegisterType::D, RegisterType::C};
    all_instructions[0x52] = {InstructionType::LD, AddressingMode::R_R, RegisterType::D, RegisterType::D};
    all_instructions[0x53] = {InstructionType::LD, AddressingMode::R_R, RegisterType::D, RegisterType::E};
    all_instructions[0x54] = {InstructionType::LD, AddressingMode::R_R, RegisterType::D, RegisterType::H};
    all_instructions[0x55] = {InstructionType::LD, AddressingMode::R_R, RegisterType::D, RegisterType::L};
    all_instructions[0x56] = {InstructionType::LD, AddressingMode::R_MR, RegisterType::D, RegisterType::HL};
    all_instructions[0x57] = {InstructionType::LD, AddressingMode::R_R, RegisterType::D, RegisterType::A};
    all_instructions[0x58] = {InstructionType::LD, AddressingMode::R_R, RegisterType::E, RegisterType::B};
    all_instructions[0x59] = {InstructionType::LD, AddressingMode::R_R, RegisterType::E, RegisterType::C};
    all_instructions[0x5A] = {InstructionType::LD, AddressingMode::R_R, RegisterType::E, RegisterType::D};
    all_instructions[0x5B] = {InstructionType::LD, AddressingMode::R_R, RegisterType::E, RegisterType::E};
    all_instructions[0x5C] = {InstructionType::LD, AddressingMode::R_R, RegisterType::E, RegisterType::H};
    all_instructions[0x5D] = {InstructionType::LD, AddressingMode::R_R, RegisterType::E, RegisterType::L};
    all_instructions[0x5E] = {InstructionType::LD, AddressingMode::R_MR, RegisterType::E, RegisterType::HL};
    all_instructions[0x5F] = {InstructionType::LD, AddressingMode::R_R, RegisterType::E, RegisterType::A};

    // 0x6X
    all_instructions[0x60] = {InstructionType::LD, AddressingMode::R_R, RegisterType::H, RegisterType::B};
    all_instructions[0x61] = {InstructionType::LD, AddressingMode::R_R, RegisterType::H, RegisterType::C};
    all_instructions[0x62] = {InstructionType::LD, AddressingMode::R_R, RegisterType::H, RegisterType::D};
    all_instructions[0x63] = {InstructionType::LD, AddressingMode::R_R, RegisterType::H, RegisterType::E};
    all_instructions[0x64] = {InstructionType::LD, AddressingMode::R_R, RegisterType::H, RegisterType::H};
    all_instructions[0x65] = {InstructionType::LD, AddressingMode::R_R, RegisterType::H, RegisterType::L};
    all_instructions[0x66] = {InstructionType::LD, AddressingMode::R_MR, RegisterType::H, RegisterType::HL};
    all_instructions[0x67] = {InstructionType::LD, AddressingMode::R_R, RegisterType::H, RegisterType::A};
    all_instructions[0x68] = {InstructionType::LD, AddressingMode::R_R, RegisterType::L, RegisterType::B};
    all_instructions[0x69] = {InstructionType::LD, AddressingMode::R_R, RegisterType::L, RegisterType::C};
    all_instructions[0x6A] = {InstructionType::LD, AddressingMode::R_R, RegisterType::L, RegisterType::D};
    all_instructions[0x6B] = {InstructionType::LD, AddressingMode::R_R, RegisterType::L, RegisterType::E};
    all_instructions[0x6C] = {InstructionType::LD, AddressingMode::R_R, RegisterType::L, RegisterType::H};
    all_instructions[0x6D] = {InstructionType::LD, AddressingMode::R_R, RegisterType::L, RegisterType::L};
    all_instructions[0x6E] = {InstructionType::LD, AddressingMode::R_MR, RegisterType::L, RegisterType::HL};
    all_instructions[0x6F] = {InstructionType::LD, AddressingMode::R_R, RegisterType::L, RegisterType::A};

    // 0x7X
    all_instructions[0x70] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::HL, RegisterType::B};
    all_instructions[0x71] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::HL, RegisterType::C};
    all_instructions[0x72] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::HL, RegisterType::D};
    all_instructions[0x73] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::HL, RegisterType::E};
    all_instructions[0x74] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::HL, RegisterType::H};
    all_instructions[0x75] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::HL, RegisterType::L};
    all_instructions[0x76] = {InstructionType::HALT},
    all_instructions[0x77] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::HL, RegisterType::A};
    all_instructions[0x78] = {InstructionType::LD, AddressingMode::R_R, RegisterType::A, RegisterType::B};
    all_instructions[0x79] = {InstructionType::LD, AddressingMode::R_R, RegisterType::A, RegisterType::C};
    all_instructions[0x7A] = {InstructionType::LD, AddressingMode::R_R, RegisterType::A, RegisterType::D};
    all_instructions[0x7B] = {InstructionType::LD, AddressingMode::R_R, RegisterType::A, RegisterType::E};
    all_instructions[0x7C] = {InstructionType::LD, AddressingMode::R_R, RegisterType::A, RegisterType::H};
    all_instructions[0x7D] = {InstructionType::LD, AddressingMode::R_R, RegisterType::A, RegisterType::L};
    all_instructions[0x7E] = {InstructionType::LD, AddressingMode::R_MR, RegisterType::A, RegisterType::HL};
    all_instructions[0x7F] = {InstructionType::LD, AddressingMode::R_R, RegisterType::A, RegisterType::A};

    // XOR instruction
    all_instructions[0xAF] = {InstructionType::XOR, AddressingMode::R, RegisterType::A};

    // 0xCx
    all_instructions[0xC0] = {
        InstructionType::RET, AddressingMode::IMP, RegisterType::None, RegisterType::None, ConditionType::NZ};

    all_instructions[0xC1] = {InstructionType::POP, AddressingMode::IMP, RegisterType::BC};
    all_instructions[0xC2] = {
        InstructionType::JP, AddressingMode::D16, RegisterType::None, RegisterType::None, ConditionType::NZ};

    all_instructions[0xC3] = {InstructionType::JP, AddressingMode::D16};
    all_instructions[0xC4] = {
        InstructionType::CALL, AddressingMode::D16, RegisterType::None, RegisterType::None, ConditionType::NZ};

    all_instructions[0xC5] = {InstructionType::PUSH, AddressingMode::IMP, RegisterType::BC};
    all_instructions[0xC7] = {InstructionType::RST, AddressingMode::IMP, RegisterType::None,
                              RegisterType::None,   ConditionType::None, 0x00};

    all_instructions[0xC8] = {
        InstructionType::RET, AddressingMode::IMP, RegisterType::None, RegisterType::None, ConditionType::Z};
    all_instructions[0xC9] = {InstructionType::RET, AddressingMode::IMP};

    all_instructions[0xCA] = {
        InstructionType::JP, AddressingMode::D16, RegisterType::None, RegisterType::None, ConditionType::Z};
    all_instructions[0xCC] = {
        InstructionType::CALL, AddressingMode::D16, RegisterType::None, RegisterType::None, ConditionType::Z};

    all_instructions[0xCD] = {InstructionType::CALL, AddressingMode::D16};
    all_instructions[0xCF] = {InstructionType::RST, AddressingMode::IMP, RegisterType::None,
                              RegisterType::None,   ConditionType::None, 0x08};
    // ------------------------------------------------------------------------------------------------------------------------------------------------
    // 0xDx

    all_instructions[0xD0] = {
        InstructionType::RET, AddressingMode::IMP, RegisterType::None, RegisterType::None, ConditionType::NC};

    all_instructions[0xD1] = {InstructionType::POP, AddressingMode::IMP, RegisterType::DE};
    all_instructions[0xD2] = {
        InstructionType::JP, AddressingMode::D16, RegisterType::None, RegisterType::None, ConditionType::NC};

    all_instructions[0xD4] = {
        InstructionType::CALL, AddressingMode::D16, RegisterType::None, RegisterType::None, ConditionType::NC};

    all_instructions[0xD5] = {InstructionType::PUSH, AddressingMode::IMP, RegisterType::DE};

    all_instructions[0xD7] = {InstructionType::RST, AddressingMode::IMP, RegisterType::None,
                              RegisterType::None,   ConditionType::None, 0x10};
    all_instructions[0xD8] = {
        InstructionType::RET, AddressingMode::IMP, RegisterType::None, RegisterType::None, ConditionType::C};
    all_instructions[0xD9] = {InstructionType::RETI, AddressingMode::IMP};

    all_instructions[0xDA] = {
        InstructionType::JP, AddressingMode::D16, RegisterType::None, RegisterType::None, ConditionType::C};
    all_instructions[0xDF] = {InstructionType::RST, AddressingMode::IMP, RegisterType::None,
                              RegisterType::None,   ConditionType::None, 0x18};

    // 0xEx
    all_instructions[0xE0] = {InstructionType::LDH, AddressingMode::A8_R, RegisterType::None, RegisterType::A};
    all_instructions[0xE1] = {InstructionType::POP, AddressingMode::IMP, RegisterType::HL};
    all_instructions[0xE2] = {InstructionType::LD, AddressingMode::MR_R, RegisterType::C, RegisterType::A};
    all_instructions[0xE5] = {InstructionType::PUSH, AddressingMode::IMP, RegisterType::HL};
    all_instructions[0xE7] = {InstructionType::RST, AddressingMode::IMP, RegisterType::None,
                              RegisterType::None,   ConditionType::None, 0x20};

    all_instructions[0xE9] = {InstructionType::JP, AddressingMode::MR, RegisterType::HL};
    all_instructions[0xEA] = {InstructionType::LD, AddressingMode::A16_R, RegisterType::None, RegisterType::A};
    all_instructions[0xEF] = {InstructionType::RST, AddressingMode::IMP, RegisterType::None,
                              RegisterType::None,   ConditionType::None, 0x28};
    // 0xFx
    all_instructions[0xF0] = {InstructionType::LDH, AddressingMode::R_A8, RegisterType::A, RegisterType::None};
    all_instructions[0xF1] = {InstructionType::POP, AddressingMode::IMP, RegisterType::AF};
    all_instructions[0xF2] = {InstructionType::LD, AddressingMode::R_MR, RegisterType::A, RegisterType::C},
    all_instructions[0xF3] = {InstructionType::DI};
    all_instructions[0xF5] = {InstructionType::PUSH, AddressingMode::IMP, RegisterType::AF};
    all_instructions[0xF7] = {InstructionType::RST, AddressingMode::IMP, RegisterType::None,
                              RegisterType::None,   ConditionType::None, 0x30};
    all_instructions[0xFA] = {InstructionType::LD, AddressingMode::R_A16, RegisterType::A};
    all_instructions[0xFF] = {InstructionType::RST, AddressingMode::IMP, RegisterType::None,
                              RegisterType::None,   ConditionType::None, 0x38};
    return all_instructions;
}

static constexpr std::array<std::string_view, NUM_INSTRUCTION_TYPES> instruction_names = {
    "<NONE>", "NOP",    "LD",     "INC",     "DEC",    "RLCA",   "ADD",    "RRCA",   "STOP",   "RLA",
    "JR",     "RRA",    "DAA",    "CPL",     "SCF",    "CCF",    "HALT",   "ADC",    "SUB",    "SBC",
    "AND",    "XOR",    "OR",     "CP",      "POP",    "JP",     "PUSH",   "RET",    "CB",     "CALL",
    "RETI",   "LDH",    "JPHL",   "DI",      "EI",     "RST",    "IN_ERR", "IN_RLC", "IN_RRC", "IN_RL",
    "IN_RR",  "IN_SLA", "IN_SRA", "IN_SWAP", "IN_SRL", "IN_BIT", "IN_RES", "IN_SET"};

// This returns the instruction name
constexpr auto get_instruction_name(InstructionType instr_type) -> std::string_view
{
    return instruction_names[std::to_underlying(instr_type)];
}

};  // namespace gameboy::cpu