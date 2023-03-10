#ifndef CPU_H
#define CPU_H

#include <cstdint>
#include <array>
#include <vector>
#include <algorithm>
#include <iostream>
#include <unordered_map>

class CPU
{
private:
    std::array<uint8_t, 0xFFFF> memory;
    enum Flags : uint8_t {
        Carry = 0b0000'0001,
        Zero = 0b0000'0010,
        Interrupt = 0b0000'0100,
        Decimal = 0b0000'1000,
        Break = 0b0001'0000,
        Unused = 0b0010'0000,
        Overflow = 0b0100'0000,
        Negative = 0b1000'0000,
    };

    enum class AddressingMode {
        Implied,
        Accumulator,
        Immediate,
        ZeroPage,
        ZeroPage_X,
        ZeroPage_Y,
        Relative,
        Absolute,
        Absolute_X,
        Absolute_Y,
        Indirect,
        Indirect_X,
        Indirect_Y,
    };

    struct OpCode {
        void (CPU::* func)(AddressingMode);
        CPU::AddressingMode mode;
    };

    std::unordered_map<uint8_t, OpCode> cpu_op_codes = {
        {0x69, {&CPU::op_adc, AddressingMode::Immediate}},
        {0x65, {&CPU::op_adc, AddressingMode::ZeroPage}},
        {0x75, {&CPU::op_adc, AddressingMode::ZeroPage_X}},
        {0x6D, {&CPU::op_adc, AddressingMode::Absolute}},
        {0x7D, {&CPU::op_adc, AddressingMode::Absolute_X}},
        {0x79, {&CPU::op_adc, AddressingMode::Absolute_Y}},
        {0x61, {&CPU::op_adc, AddressingMode::Indirect_X}},
        {0x71, {&CPU::op_adc, AddressingMode::Indirect_Y}},
        
        {0x29, {&CPU::op_and, AddressingMode::Immediate}},
        {0x25, {&CPU::op_and, AddressingMode::ZeroPage}},
        {0x35, {&CPU::op_and, AddressingMode::ZeroPage_X}},
        {0x2D, {&CPU::op_and, AddressingMode::Absolute}},
        {0x3D, {&CPU::op_and, AddressingMode::Absolute_X}},
        {0x39, {&CPU::op_and, AddressingMode::Absolute_Y}},
        {0x21, {&CPU::op_and, AddressingMode::Indirect_X}},
        {0x31, {&CPU::op_and, AddressingMode::Indirect_Y}},
        
        {0x0A, {&CPU::op_asl, AddressingMode::Accumulator}},
        {0x06, {&CPU::op_asl, AddressingMode::ZeroPage}},
        {0x16, {&CPU::op_asl, AddressingMode::ZeroPage_X}},
        {0x0E, {&CPU::op_asl, AddressingMode::Absolute}},
        {0x1E, {&CPU::op_asl, AddressingMode::Absolute_X}},

        {0x90, {&CPU::op_bcc, AddressingMode::Relative}},

        {0xB0, {&CPU::op_bcs, AddressingMode::Relative}},
        
        {0xF0, {&CPU::op_beq, AddressingMode::Relative}},
        
        {0x24, {&CPU::op_bit, AddressingMode::ZeroPage}},
        {0x2C, {&CPU::op_bit, AddressingMode::Absolute}},
        
        {0x30, {&CPU::op_bmi, AddressingMode::Relative}},

        {0xD0, {&CPU::op_bne, AddressingMode::Relative}},
        
        {0x10, {&CPU::op_bpl, AddressingMode::Relative}},

        {0x00, {&CPU::op_brk, AddressingMode::Implied}},

        {0x50, {&CPU::op_bvc, AddressingMode::Relative}},

        {0x70, {&CPU::op_bvs, AddressingMode::Relative}},

        {0x18, {&CPU::op_clc, AddressingMode::Implied}},

        {0xD8, {&CPU::op_cld, AddressingMode::Implied}},
        
        {0x58, {&CPU::op_cli, AddressingMode::Implied}},

        {0xB8, {&CPU::op_clv, AddressingMode::Implied}},
        
        {0xC9, {&CPU::op_cmp, AddressingMode::Immediate}},
        {0xC5, {&CPU::op_cmp, AddressingMode::ZeroPage}},
        {0xD5, {&CPU::op_cmp, AddressingMode::ZeroPage_X}},
        {0xCD, {&CPU::op_cmp, AddressingMode::Absolute}},
        {0xDD, {&CPU::op_cmp, AddressingMode::Absolute_X}},
        {0xD9, {&CPU::op_cmp, AddressingMode::Absolute_Y}},
        {0xC1, {&CPU::op_cmp, AddressingMode::Indirect_X}},
        {0xD1, {&CPU::op_cmp, AddressingMode::Indirect_Y}},
        
        {0xE0, {&CPU::op_cpx, AddressingMode::Immediate}},
        {0xE4, {&CPU::op_cpx, AddressingMode::ZeroPage}},
        {0xEC, {&CPU::op_cpx, AddressingMode::Absolute}},
        
        {0xC0, {&CPU::op_cpy, AddressingMode::Immediate}},
        {0xC4, {&CPU::op_cpy, AddressingMode::ZeroPage}},
        {0xCC, {&CPU::op_cpy, AddressingMode::Absolute}},
        
        {0xC6, {&CPU::op_dec, AddressingMode::ZeroPage}},
        {0xD6, {&CPU::op_dec, AddressingMode::ZeroPage_X}},
        {0xCE, {&CPU::op_dec, AddressingMode::Absolute}},
        {0xDE, {&CPU::op_dec, AddressingMode::Absolute_X}},

        {0xCA, {&CPU::op_dex, AddressingMode::Implied}},

        {0x88, {&CPU::op_dey, AddressingMode::Implied}},
        
        {0x49, {&CPU::op_eor, AddressingMode::Immediate}},
        {0x45, {&CPU::op_eor, AddressingMode::ZeroPage}},
        {0x55, {&CPU::op_eor, AddressingMode::ZeroPage_X}},
        {0x4D, {&CPU::op_eor, AddressingMode::Absolute}},
        {0x5D, {&CPU::op_eor, AddressingMode::Absolute_X}},
        {0x59, {&CPU::op_eor, AddressingMode::Absolute_Y}},
        {0x41, {&CPU::op_eor, AddressingMode::Indirect_X}},
        {0x51, {&CPU::op_eor, AddressingMode::Indirect_Y}},

        {0xE6, {&CPU::op_inc, AddressingMode::ZeroPage}},
        {0xF6, {&CPU::op_inc, AddressingMode::ZeroPage_X}},
        {0xEE, {&CPU::op_inc, AddressingMode::Absolute}},
        {0xFE, {&CPU::op_inc, AddressingMode::Absolute_X}},
        
        {0xE8, {&CPU::op_inx, AddressingMode::Implied}},

        {0xC8, {&CPU::op_iny, AddressingMode::Implied}},
        
        {0x4C, {&CPU::op_jmp, AddressingMode::Absolute}},
        {0x6C, {&CPU::op_jmp, AddressingMode::Indirect}},

        {0x20, {&CPU::op_jsr, AddressingMode::Absolute}},

        {0xA9, {&CPU::op_lda, AddressingMode::Immediate}},
        {0xA5, {&CPU::op_lda, AddressingMode::ZeroPage}},
        {0xB5, {&CPU::op_lda, AddressingMode::ZeroPage_X}},
        {0xAD, {&CPU::op_lda, AddressingMode::Absolute}},
        {0xBD, {&CPU::op_lda, AddressingMode::Absolute_X}},
        {0xB9, {&CPU::op_lda, AddressingMode::Absolute_Y}},
        {0xA1, {&CPU::op_lda, AddressingMode::Indirect_X}},
        {0xB1, {&CPU::op_lda, AddressingMode::Indirect_Y}},

        {0xA2, {&CPU::op_ldx, AddressingMode::Immediate}},
        {0xA6, {&CPU::op_ldx, AddressingMode::ZeroPage}},
        {0xB6, {&CPU::op_ldx, AddressingMode::ZeroPage_Y}},
        {0xAE, {&CPU::op_ldx, AddressingMode::Absolute}},
        {0xBE, {&CPU::op_ldx, AddressingMode::Absolute_Y}},

        {0xA0, {&CPU::op_ldy, AddressingMode::Immediate}},
        {0xA4, {&CPU::op_ldy, AddressingMode::ZeroPage}},
        {0xB4, {&CPU::op_ldy, AddressingMode::ZeroPage_X}},
        {0xAC, {&CPU::op_ldy, AddressingMode::Absolute}},
        {0xBC, {&CPU::op_ldy, AddressingMode::Absolute_X}},

        {0x4A, {&CPU::op_lsr, AddressingMode::Accumulator}},
        {0x46, {&CPU::op_lsr, AddressingMode::ZeroPage}},
        {0x56, {&CPU::op_lsr, AddressingMode::ZeroPage_X}},
        {0x4E, {&CPU::op_lsr, AddressingMode::Absolute}},
        {0x5E, {&CPU::op_lsr, AddressingMode::Absolute_X}},

        {0xEA, {&CPU::op_nop, AddressingMode::Implied}},

        {0x09, {&CPU::op_ora, AddressingMode::Immediate}},
        {0x05, {&CPU::op_ora, AddressingMode::ZeroPage}},
        {0x15, {&CPU::op_ora, AddressingMode::ZeroPage_X}},
        {0x0D, {&CPU::op_ora, AddressingMode::Absolute}},
        {0x1D, {&CPU::op_ora, AddressingMode::Absolute_X}},
        {0x19, {&CPU::op_ora, AddressingMode::Absolute_Y}},
        {0x01, {&CPU::op_ora, AddressingMode::Indirect_X}},
        {0x11, {&CPU::op_ora, AddressingMode::Indirect_Y}},

        {0x48, {&CPU::op_pha, AddressingMode::Implied}},

        {0x08, {&CPU::op_php, AddressingMode::Implied}},

        {0x68, {&CPU::op_pla, AddressingMode::Implied}},

        {0x28, {&CPU::op_plp, AddressingMode::Implied}},

        {0x2A, {&CPU::op_rol, AddressingMode::Accumulator}},
        {0x26, {&CPU::op_rol, AddressingMode::ZeroPage}},
        {0x36, {&CPU::op_rol, AddressingMode::ZeroPage_X}},
        {0x2E, {&CPU::op_rol, AddressingMode::Absolute}},
        {0x3E, {&CPU::op_rol, AddressingMode::Absolute_X}},

        {0x6A, {&CPU::op_ror, AddressingMode::Accumulator}},
        {0x66, {&CPU::op_ror, AddressingMode::ZeroPage}},
        {0x76, {&CPU::op_ror, AddressingMode::ZeroPage_X}},
        {0x6E, {&CPU::op_ror, AddressingMode::Absolute}},
        {0x7E, {&CPU::op_ror, AddressingMode::Absolute_X}},

        {0x40, {&CPU::op_ror, AddressingMode::Implied}},

        {0x60, {&CPU::op_rts, AddressingMode::Implied}},

        {0xE9, {&CPU::op_sbc, AddressingMode::Immediate}},
        {0xE5, {&CPU::op_sbc, AddressingMode::ZeroPage}},
        {0xF5, {&CPU::op_sbc, AddressingMode::ZeroPage_X}},
        {0xED, {&CPU::op_sbc, AddressingMode::Absolute}},
        {0xFD, {&CPU::op_sbc, AddressingMode::Absolute_X}},
        {0xF9, {&CPU::op_sbc, AddressingMode::Absolute_Y}},
        {0xE1, {&CPU::op_sbc, AddressingMode::Indirect_X}},
        {0xF1, {&CPU::op_sbc, AddressingMode::Indirect_Y}},

        {0x38, {&CPU::op_sec, AddressingMode::Implied}},

        {0xF8, {&CPU::op_sed, AddressingMode::Implied}},

        {0x78, {&CPU::op_sei, AddressingMode::Implied}},

        {0x85, {&CPU::op_sta, AddressingMode::ZeroPage}},
        {0x95, {&CPU::op_sta, AddressingMode::ZeroPage_X}},
        {0x8D, {&CPU::op_sta, AddressingMode::Absolute}},
        {0x9D, {&CPU::op_sta, AddressingMode::Absolute_X}},
        {0x99, {&CPU::op_sta, AddressingMode::Absolute_Y}},
        {0x81, {&CPU::op_sta, AddressingMode::Indirect_X}},
        {0x91, {&CPU::op_sta, AddressingMode::Indirect_Y}},

        {0x86, {&CPU::op_stx, AddressingMode::ZeroPage}},
        {0x96, {&CPU::op_stx, AddressingMode::ZeroPage_Y}},
        {0x8E, {&CPU::op_stx, AddressingMode::Absolute}},

        {0x84, {&CPU::op_sty, AddressingMode::ZeroPage}},
        {0x94, {&CPU::op_sty, AddressingMode::ZeroPage_X}},
        {0x8C, {&CPU::op_sty, AddressingMode::Absolute}},

        {0xAA, {&CPU::op_tax, AddressingMode::Implied}},

        {0xA8, {&CPU::op_tay, AddressingMode::Implied}},

        {0xBA, {&CPU::op_tsx, AddressingMode::Implied}},

        {0x8A, {&CPU::op_txa, AddressingMode::Implied}},

        {0x9A, {&CPU::op_txs, AddressingMode::Implied}},

        {0x98, {&CPU::op_tya, AddressingMode::Implied}},
    };
    

public:
    uint8_t register_a;
    uint8_t register_x;
    uint8_t register_y;
    uint8_t status;
    uint8_t stack_pointer;
    uint16_t program_counter;

    CPU() : register_a{ 0 }, register_x{ 0 }, register_y{ 0 }, status{ 0 }, program_counter{ 0 } {}

    uint8_t mem_read(uint16_t addr);
    void mem_write(uint16_t addr, uint8_t data);
    uint16_t mem_read_uint16_t(uint16_t addr);
    void mem_write_uint16_t(uint16_t addr, uint16_t data);

    uint16_t get_operand_address(AddressingMode &mode);

    void load_and_run(std::vector<uint8_t> program);
    void load(std::vector<uint8_t> program);
    void reset();
    void run(); 


    // instructions
    void op_adc(AddressingMode mode);
    void op_and(AddressingMode mode);
    void op_asl(AddressingMode mode);
    void op_bcc(AddressingMode mode);
    void op_bcs(AddressingMode mode);
    void op_beq(AddressingMode mode);
    void op_bit(AddressingMode mode);
    void op_bmi(AddressingMode mode);
    void op_bne(AddressingMode mode);
    void op_bpl(AddressingMode mode);
    void op_brk(AddressingMode mode);
    void op_bvc(AddressingMode mode);
    void op_bvs(AddressingMode mode);
    void op_clc(AddressingMode mode);
    void op_cld(AddressingMode mode);
    void op_cli(AddressingMode mode);
    void op_clv(AddressingMode mode);
    void op_cmp(AddressingMode mode);
    void op_cpx(AddressingMode mode);
    void op_cpy(AddressingMode mode);
    void op_dec(AddressingMode mode);
    void op_dex(AddressingMode mode);
    void op_dey(AddressingMode mode);
    void op_eor(AddressingMode mode);
    void op_inc(AddressingMode mode);
    void op_inx(AddressingMode mode);
    void op_iny(AddressingMode mode);
    void op_jmp(AddressingMode mode);
    void op_jsr(AddressingMode mode);
    void op_lda(AddressingMode mode);
    void op_ldx(AddressingMode mode);
    void op_ldy(AddressingMode mode);
    void op_lsr(AddressingMode mode);
    void op_nop(AddressingMode mode);
    void op_ora(AddressingMode mode);
    void op_pha(AddressingMode mode);
    void op_php(AddressingMode mode);
    void op_pla(AddressingMode mode);
    void op_plp(AddressingMode mode);
    void op_rol(AddressingMode mode);
    void op_ror(AddressingMode mode);
    void op_rti(AddressingMode mode);
    void op_rts(AddressingMode mode);
    void op_sbc(AddressingMode mode);
    void op_sec(AddressingMode mode);
    void op_sed(AddressingMode mode);
    void op_sei(AddressingMode mode);
    void op_sta(AddressingMode mode);
    void op_stx(AddressingMode mode);
    void op_sty(AddressingMode mode);
    void op_tax(AddressingMode mode);
    void op_tay(AddressingMode mode);
    void op_tsx(AddressingMode mode);
    void op_txa(AddressingMode mode);
    void op_txs(AddressingMode mode);
    void op_tya(AddressingMode mode);

    //helper functions
    void update_zero_and_negative_flags(uint8_t result);
};

#endif