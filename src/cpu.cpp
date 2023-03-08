#include "cpu.h"

uint8_t CPU::mem_read(uint16_t addr) {
    return memory[addr];
}

void CPU::mem_write(uint16_t addr, uint8_t data) {
    memory[addr] = data;
}

uint16_t CPU::mem_read_uint16_t(uint16_t addr) {
    uint16_t low = mem_read(addr);
    uint16_t high = mem_read(addr + 1);
    return (high << 8) | (low);
}

void CPU::mem_write_uint16_t(uint16_t addr, uint16_t data) {
    uint8_t high = data >> 8;
    uint8_t low = data & 0xff;
    mem_write(addr, low);
    mem_write(addr + 1, high);
}

void CPU::load_and_run(std::vector<uint8_t> program) {
    load(program);
    reset();
    run();
}

void CPU::load(std::vector<uint8_t> program) {
    std::copy(program.begin(), program.end(), std::next(memory.begin(), 0x8000));
    mem_write_uint16_t(0xFFFC, 0x8000);
}

void CPU::reset() {
    register_a = 0;
    register_x = 0;
    register_y = 0;
    status = 0;

    program_counter = mem_read_uint16_t(0xFFFC);
}

void CPU::run()
{
    while(true) {
        uint8_t opcode = mem_read(program_counter);
        program_counter++;

        switch (opcode)
        {
            // BRK - Force Interrupt
            case 0x00:
            {
                return;
            }

            // LDA - Load Accumulator
            case 0xA9:
            {
                uint8_t param = mem_read(program_counter);
                program_counter++;

                lda(param);
                break;
            }

            case 0xE8:
            {
                inx();
                break;
            }

            // TAX - Transfer Accumulator to X
            case 0xAA:
            {
                tax();
                break;
            }

            default:
            {
                break;
            }
        }
    }
}

void CPU::lda(uint8_t value) {
    register_a = value;
    update_zero_and_negative_flags(register_a);
}

void CPU::inx() {
    register_x++;
    update_zero_and_negative_flags(register_x);
}

void CPU::tax() {
    register_x = register_a;
    update_zero_and_negative_flags(register_x);
}

void CPU::update_zero_and_negative_flags(uint8_t result) {
    if(result == 0){
        status |= Flags::Zero;
    }else{
        status &= ~Flags::Zero;
    }

    if((result & Flags::Negative) != 0){
        status |= Flags::Negative;
    }else{
        status &= ~Flags::Negative;
    }
}