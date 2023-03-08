#ifndef CPU_H
#define CPU_H

#include <cstdint>
#include <array>
#include <vector>
#include <algorithm>

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
        NoneAddressing,
    };
    

public:
    uint8_t register_a;
    uint8_t register_x;
    uint8_t register_y;
    uint8_t status;
    uint16_t program_counter;

    CPU() : register_a{ 0 }, status{ 0 }, program_counter{ 0 } {}

    uint8_t mem_read(uint16_t addr);
    void mem_write(uint16_t addr, uint8_t data);
    uint16_t mem_read_uint16_t(uint16_t addr);
    void mem_write_uint16_t(uint16_t addr, uint16_t data);
    void load_and_run(std::vector<uint8_t> program);
    void load(std::vector<uint8_t> program);
    void reset();
    void run(); 

    // instructions
    void adc();
    void andy();
    void asl();
    void bcc();
    void bcs();
    void beq();
    void bit();
    void bmi();
    void bne();
    void bpl();
    void bvc();
    void bvs();
    void clc();
    void cld();
    void cli();
    void clv();
    void cmp();
    void cpx();
    void cpy();
    void dec();
    void dex();
    void dey();
    void eor();
    void inc();
    void inx();
    void iny();
    void jmp();
    void jsr();
    void lda(uint8_t value);
    void ldx();
    void ldy();
    void lsr();
    void nop();
    void ora();
    void pha();
    void php();
    void pla();
    void plp();
    void rol();
    void ror();
    void rti();
    void rts();
    void sbc();
    void sec();
    void sed();
    void sei();
    void sta();
    void stx();
    void sty();
    void tax();
    void tay();
    void tsx();
    void txa();
    void txs();
    void tya();
    //helper functions
    void update_zero_and_negative_flags(uint8_t result);
};

#endif