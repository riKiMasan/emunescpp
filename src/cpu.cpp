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

uint16_t CPU::get_operand_address(AddressingMode &mode) {
    switch(mode) {
        case AddressingMode::Immediate:
        {
            return program_counter++;
        }
    
        case AddressingMode::ZeroPage:
        {
            return mem_read(program_counter++);
        }

        case AddressingMode::ZeroPage_X:
        {
            uint8_t pos = mem_read(program_counter++);
            return (pos + register_x) & 0xFF;
        }

        case AddressingMode::ZeroPage_Y:
        {
            uint8_t pos = mem_read(program_counter++);
            return (pos + register_y) & 0xFF;
        }

        case AddressingMode::Absolute:
        {
            uint16_t addr = mem_read_uint16_t(program_counter);
            program_counter += 2;
            return addr;
        }

        case AddressingMode::Absolute_X:
        {
            uint16_t pos = mem_read_uint16_t(program_counter);
            program_counter += 2;
            return (pos + register_x) & 0xFFFF;
        }

        case AddressingMode::Absolute_Y:
        {
            uint16_t pos = mem_read_uint16_t(program_counter);
            program_counter += 2;
            return (pos + register_y) & 0xFFFF;
        }

        case AddressingMode::Indirect_X:
        {
            uint8_t base = mem_read(program_counter++);

            uint8_t ptr = (base + register_x) & 0xFF;
            uint16_t low = mem_read(ptr);
            uint16_t high = mem_read((ptr + 1) & 0xFF);

            return (high << 8) | low;
        }

        case AddressingMode::Indirect_Y:
        {
            uint8_t base = mem_read(program_counter++);

            uint16_t low = mem_read(base);
            uint16_t high = mem_read((base + 1) & 0xFF);
            uint16_t deref_base = (high << 8) | low;
            uint16_t deref = (deref_base + register_y) & 0xFFFF;
            return deref;
        }

        case AddressingMode::Implied:
        {
            std::cerr << "mode is not supported" << std::endl;
            abort();
        }
    }
}

void CPU::load_and_run(std::vector<uint8_t> program) {
    load(program);
    reset();
    run();
}

void CPU::load(std::vector<uint8_t> program) {
    if(program.size() > 0xFFFF - 0x8000) {
        std::cerr << "Error: Program is larger than memory capacity." << std::endl;
        abort();
    }
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
        uint8_t opcode = mem_read(program_counter++);
        
        OpCode op = cpu_op_codes[opcode];
        (this->*op.func)(op.mode);

        if(opcode == 0x00) return;
    }
}

void CPU::op_brk(AddressingMode mode) {
    status |= Flags::Break;
}

void CPU::op_lda(AddressingMode mode) {
    uint16_t addr = get_operand_address(mode);
    uint8_t value = mem_read(addr);

    register_a = value;
    update_zero_and_negative_flags(register_a);
}

void CPU::op_inx(AddressingMode mode) {
    register_x++;
    update_zero_and_negative_flags(register_x);
}

void CPU::op_tax(AddressingMode mode) {
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