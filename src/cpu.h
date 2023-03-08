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
        {0x00, {&CPU::brk, AddressingMode::Implied}},
        {0xA9, {&CPU::lda, AddressingMode::Immediate}},
        {0xA5, {&CPU::lda, AddressingMode::ZeroPage}},
        {0xAD, {&CPU::lda, AddressingMode::Absolute}},
        {0xE8, {&CPU::inx, AddressingMode::Implied}},
        {0xAA, {&CPU::tax, AddressingMode::Implied}}
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
    void adc(AddressingMode mode);
    void andy(AddressingMode mode);
    void asl(AddressingMode mode);
    void bcc(AddressingMode mode);
    void bcs(AddressingMode mode);
    void beq(AddressingMode mode);
    void bit(AddressingMode mode);
    void bmi(AddressingMode mode);
    void bne(AddressingMode mode);
    void bpl(AddressingMode mode);
    void brk(AddressingMode mode);
    void bvc(AddressingMode mode);
    void bvs(AddressingMode mode);
    void clc(AddressingMode mode);
    void cld(AddressingMode mode);
    void cli(AddressingMode mode);
    void clv(AddressingMode mode);
    void cmp(AddressingMode mode);
    void cpx(AddressingMode mode);
    void cpy(AddressingMode mode);
    void dec(AddressingMode mode);
    void dex(AddressingMode mode);
    void dey(AddressingMode mode);
    void eor(AddressingMode mode);
    void inc(AddressingMode mode);
    void inx(AddressingMode mode);
    void iny(AddressingMode mode);
    void jmp(AddressingMode mode);
    void jsr(AddressingMode mode);
    void lda(AddressingMode mode);
    void ldx(AddressingMode mode);
    void ldy(AddressingMode mode);
    void lsr(AddressingMode mode);
    void nop(AddressingMode mode);
    void ora(AddressingMode mode);
    void pha(AddressingMode mode);
    void php(AddressingMode mode);
    void pla(AddressingMode mode);
    void plp(AddressingMode mode);
    void rol(AddressingMode mode);
    void ror(AddressingMode mode);
    void rti(AddressingMode mode);
    void rts(AddressingMode mode);
    void sbc(AddressingMode mode);
    void sec(AddressingMode mode);
    void sed(AddressingMode mode);
    void sei(AddressingMode mode);
    void sta(AddressingMode mode);
    void stx(AddressingMode mode);
    void sty(AddressingMode mode);
    void tax(AddressingMode mode);
    void tay(AddressingMode mode);
    void tsx(AddressingMode mode);
    void txa(AddressingMode mode);
    void txs(AddressingMode mode);
    void tya(AddressingMode mode);

    //helper functions
    void update_zero_and_negative_flags(uint8_t result);
};

#endif