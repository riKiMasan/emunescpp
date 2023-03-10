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
        Carry = 0b00000001,
        Zero = 0b00000010,
        Interrupt = 0b00000100,
        Decimal = 0b00001000,
        Break = 0b00010000,
        Unused = 0b00100000,
        Overflow = 0b01000000,
        Negative = 0b10000000,
    };

    enum class AddressingMode {
        Immediate,
        ZeroPage,
        ZeroPage_X,
        ZeroPage_Y,
        Absolute,
        Absolute_X,
        Absolute_Y,
        Indirect_X,
        Indirect_Y,
        Implied,
    };

    struct OpCode {
        void (CPU::* func)(AddressingMode);
        CPU::AddressingMode mode;
    };

    std::unordered_map<uint8_t, OpCode> cpu_op_codes = {
        {0x00, {&CPU::op_brk, AddressingMode::Implied}},
        {0xA9, {&CPU::op_lda, AddressingMode::Immediate}},
        {0xA5, {&CPU::op_lda, AddressingMode::ZeroPage}},
        {0xAD, {&CPU::op_lda, AddressingMode::Absolute}},
        {0xE8, {&CPU::op_inx, AddressingMode::Implied}},
        {0xAA, {&CPU::op_tax, AddressingMode::Implied}}
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