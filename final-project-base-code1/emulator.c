#include <stdio.h> // for stderr
#include <stdlib.h> // for exit()
#include "types.h"
#include "utils.h"
#include "riscv.h"

void execute_rtype(Instruction, Processor *);
void execute_itype_except_load(Instruction, Processor *);
void execute_branch(Instruction, Processor *);
void execute_jal(Instruction, Processor *);
void execute_load(Instruction, Processor *, Byte *);
void execute_store(Instruction, Processor *, Byte *);
void execute_ecall(Processor *, Byte *);
void execute_lui(Instruction, Processor *);

void execute_instruction(uint32_t instruction_bits, Processor *processor, Byte *memory) {    
    Instruction instruction = parse_instruction(instruction_bits);
    switch(instruction.opcode) {
        case 0x33:
            execute_rtype(instruction, processor);
            break;
        case 0x13:
            execute_itype_except_load(instruction, processor);
            break;
        case 0x73:
            execute_ecall(processor, memory);
            break;
        case 0x63:
            execute_branch(instruction, processor);
            break;
        case 0x6F:
            execute_jal(instruction, processor);
            break;
        case 0x23:
            execute_store(instruction, processor, memory);
            break;
        case 0x03:
            execute_load(instruction, processor, memory);
            break;
        case 0x37:
            execute_lui(instruction, processor);
            break;
        case 0x17:
            // AUIPC - Add Upper Immediate to PC
            processor->R[instruction.utype.rd] = processor->PC + (instruction.utype.imm << 12);
            processor->PC += 4;
            break;
        default: // undefined opcode
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
}

void execute_rtype(Instruction instruction, Processor *processor) {
    switch (instruction.rtype.funct3){
        case 0x0:
            switch (instruction.rtype.funct7) {
                case 0x0:
                    // Add
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1]) +
                        ((sWord)processor->R[instruction.rtype.rs2]);
                    break;
                case 0x1:
                    // Mul
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1]) *
                        ((sWord)processor->R[instruction.rtype.rs2]);
                    break;
                case 0x20:
                    // Sub
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1]) -
                        ((sWord)processor->R[instruction.rtype.rs2]);
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
                    break;
            }
            break;
        case 0x1:
            switch (instruction.rtype.funct7) {
                case 0x0:
                    // SLL - Shift Left Logical
                    processor->R[instruction.rtype.rd] = 
                        processor->R[instruction.rtype.rs1] << (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                case 0x1:
                    // MULH - Multiply High
                    {
                        int64_t result = ((int64_t)(sWord)processor->R[instruction.rtype.rs1]) * 
                                       ((int64_t)(sWord)processor->R[instruction.rtype.rs2]);
                        processor->R[instruction.rtype.rd] = (Word)(result >> 32);
                    }
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
                    break;
            }
            break;
        case 0x2:
            // SLT - Set Less Than
            processor->R[instruction.rtype.rd] = 
                ((sWord)processor->R[instruction.rtype.rs1] < (sWord)processor->R[instruction.rtype.rs2]) ? 1 : 0;
            break;
        case 0x4:
            switch (instruction.rtype.funct7) {
                case 0x0:
                    // XOR
                    processor->R[instruction.rtype.rd] = 
                        processor->R[instruction.rtype.rs1] ^ processor->R[instruction.rtype.rs2];
                    break;
                case 0x1:
                    // DIV
                    if (processor->R[instruction.rtype.rs2] == 0) {
                        processor->R[instruction.rtype.rd] = -1; // Division by zero
                    } else {
                        processor->R[instruction.rtype.rd] = 
                            (sWord)processor->R[instruction.rtype.rs1] / (sWord)processor->R[instruction.rtype.rs2];
                    }
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
                    break;
            }
            break;
        case 0x5:
            switch (instruction.rtype.funct7) {
                case 0x0:
                    // SRL - Shift Right Logical
                    processor->R[instruction.rtype.rd] = 
                        processor->R[instruction.rtype.rs1] >> (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                case 0x20:
                    // SRA - Shift Right Arithmetic
                    processor->R[instruction.rtype.rd] = 
                        (sWord)processor->R[instruction.rtype.rs1] >> (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
                    break;
            }
            break;
        case 0x6:
            switch (instruction.rtype.funct7) {
                case 0x0:
                    // OR
                    processor->R[instruction.rtype.rd] = 
                        processor->R[instruction.rtype.rs1] | processor->R[instruction.rtype.rs2];
                    break;
                case 0x1:
                    // REM - Remainder
                    if (processor->R[instruction.rtype.rs2] == 0) {
                        processor->R[instruction.rtype.rd] = processor->R[instruction.rtype.rs1];
                    } else {
                        processor->R[instruction.rtype.rd] = 
                            (sWord)processor->R[instruction.rtype.rs1] % (sWord)processor->R[instruction.rtype.rs2];
                    }
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
                    break;
            }
            break;
        case 0x7:
            // AND
            processor->R[instruction.rtype.rd] = 
                processor->R[instruction.rtype.rs1] & processor->R[instruction.rtype.rs2];
            break;
        default:
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
    // update PC
    processor->PC += 4;
}

void execute_itype_except_load(Instruction instruction, Processor *processor) {
    sWord imm = sign_extend_number(instruction.itype.imm, 12);
    
    switch (instruction.itype.funct3) {
        case 0x0:
            // ADDI
            processor->R[instruction.itype.rd] = 
                (sWord)processor->R[instruction.itype.rs1] + imm;
            break;
        case 0x1:
            // SLLI - Shift Left Logical Immediate
            processor->R[instruction.itype.rd] = 
                processor->R[instruction.itype.rs1] << (instruction.itype.imm & 0x1F);
            break;
        case 0x2:
            // SLTI - Set Less Than Immediate
            processor->R[instruction.itype.rd] = 
                ((sWord)processor->R[instruction.itype.rs1] < imm) ? 1 : 0;
            break;
        case 0x4:
            // XORI
            processor->R[instruction.itype.rd] = 
                processor->R[instruction.itype.rs1] ^ (Word)imm;
            break;
        case 0x5:
            // SRLI or SRAI
            if ((instruction.itype.imm >> 5) == 0x0) {
                // SRLI - Shift Right Logical Immediate
                processor->R[instruction.itype.rd] = 
                    processor->R[instruction.itype.rs1] >> (instruction.itype.imm & 0x1F);
            } else if ((instruction.itype.imm >> 5) == 0x20) {
                // SRAI - Shift Right Arithmetic Immediate
                processor->R[instruction.itype.rd] = 
                    (sWord)processor->R[instruction.itype.rs1] >> (instruction.itype.imm & 0x1F);
            } else {
                handle_invalid_instruction(instruction);
                exit(-1);
            }
            break;
        case 0x6:
            // ORI
            processor->R[instruction.itype.rd] = 
                processor->R[instruction.itype.rs1] | (Word)imm;
            break;
        case 0x7:
            // ANDI
            processor->R[instruction.itype.rd] = 
                processor->R[instruction.itype.rs1] & (Word)imm;
            break;
        default:
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
    processor->PC += 4;
}

void execute_ecall(Processor *p, Byte *memory) {
    Register i;
    
    // syscall number is given by a0 (x10)
    // argument is given by a1
    switch(p->R[10]) {
        case 1: // print an integer
            printf("%d",p->R[11]);
            p->PC += 4;
            break;
        case 4: // print a string
            for(i=p->R[11];i<MEMORY_SPACE && load(memory,i,LENGTH_BYTE);i++) {
                printf("%c",load(memory,i,LENGTH_BYTE));
            }
            p->PC += 4;
            break;
        case 10: // exit
            printf("exiting the simulator\n");
            exit(0);
            break;
        case 11: // print a character
            printf("%c",p->R[11]);
            p->PC += 4;
            break;
        default: // undefined ecall
            printf("Illegal ecall number %d\n", p->R[10]);
            exit(-1);
            break;
    }
}

void execute_branch(Instruction instruction, Processor *processor) {
    sWord offset = get_branch_offset(instruction);
    
    switch (instruction.sbtype.funct3) {
        case 0x0:
            // BEQ - Branch if Equal
            if (processor->R[instruction.sbtype.rs1] == processor->R[instruction.sbtype.rs2]) {
                processor->PC += offset;
            } else {
                processor->PC += 4;
            }
            break;
        case 0x1:
            // BNE - Branch if Not Equal
            if (processor->R[instruction.sbtype.rs1] != processor->R[instruction.sbtype.rs2]) {
                processor->PC += offset;
            } else {
                processor->PC += 4;
            }
            break;
        case 0x4:
            // BLT - Branch if Less Than (signed)
            if ((sWord)processor->R[instruction.sbtype.rs1] < (sWord)processor->R[instruction.sbtype.rs2]) {
                processor->PC += offset;
            } else {
                processor->PC += 4;
            }
            break;
        case 0x5:
            // BGE - Branch if Greater Than or Equal (signed)
            if ((sWord)processor->R[instruction.sbtype.rs1] >= (sWord)processor->R[instruction.sbtype.rs2]) {
                processor->PC += offset;
            } else {
                processor->PC += 4;
            }
            break;
        case 0x6:
            // BLTU - Branch if Less Than Unsigned
            if (processor->R[instruction.sbtype.rs1] < processor->R[instruction.sbtype.rs2]) {
                processor->PC += offset;
            } else {
                processor->PC += 4;
            }
            break;
        case 0x7:
            // BGEU - Branch if Greater Than or Equal Unsigned
            if (processor->R[instruction.sbtype.rs1] >= processor->R[instruction.sbtype.rs2]) {
                processor->PC += offset;
            } else {
                processor->PC += 4;
            }
            break;
        default:
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
}

void execute_load(Instruction instruction, Processor *processor, Byte *memory) {
    sWord offset = sign_extend_number(instruction.itype.imm, 12);
    Address addr = processor->R[instruction.itype.rs1] + offset;
    
    switch (instruction.itype.funct3) {
        case 0x0:
            // LB - Load Byte (sign-extended)
            processor->R[instruction.itype.rd] = sign_extend_number(load(memory, addr, LENGTH_BYTE), 8);
            break;
        case 0x1:
            // LH - Load Halfword (sign-extended)
            processor->R[instruction.itype.rd] = sign_extend_number(load(memory, addr, LENGTH_HALF_WORD), 16);
            break;
        case 0x2:
            // LW - Load Word
            processor->R[instruction.itype.rd] = load(memory, addr, LENGTH_WORD);
            break;
        case 0x4:
            // LBU - Load Byte Unsigned
            processor->R[instruction.itype.rd] = load(memory, addr, LENGTH_BYTE);
            break;
        case 0x5:
            // LHU - Load Halfword Unsigned
            processor->R[instruction.itype.rd] = load(memory, addr, LENGTH_HALF_WORD);
            break;
        default:
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
    processor->PC += 4;
}

void execute_store(Instruction instruction, Processor *processor, Byte *memory) {
    sWord offset = get_store_offset(instruction);
    Address addr = processor->R[instruction.stype.rs1] + offset;
    
    switch (instruction.stype.funct3) {
        case 0x0:
            // SB - Store Byte
            store(memory, addr, LENGTH_BYTE, processor->R[instruction.stype.rs2]);
            break;
        case 0x1:
            // SH - Store Halfword
            store(memory, addr, LENGTH_HALF_WORD, processor->R[instruction.stype.rs2]);
            break;
        case 0x2:
            // SW - Store Word
            store(memory, addr, LENGTH_WORD, processor->R[instruction.stype.rs2]);
            break;
        default:
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
    processor->PC += 4;
}

void execute_jal(Instruction instruction, Processor *processor) {
    sWord offset = get_jump_offset(instruction);
    
    // Store return address (PC + 4) in rd
    processor->R[instruction.ujtype.rd] = processor->PC + 4;
    
    // Jump to PC + offset
    processor->PC += offset;
}

void execute_lui(Instruction instruction, Processor *processor) {
    // LUI - Load Upper Immediate
    // Load 20-bit immediate into upper 20 bits of rd, lower 12 bits = 0
    processor->R[instruction.utype.rd] = instruction.utype.imm << 12;
    processor->PC += 4;
}

void store(Byte *memory, Address address, Alignment alignment, Word value) {
    if(alignment == LENGTH_BYTE) {
        memory[address] = value & 0xFF;
    } else if(alignment == LENGTH_HALF_WORD) {
        memory[address] = value & 0xFF;
        memory[address + 1] = (value >> 8) & 0xFF;
    } else if(alignment == LENGTH_WORD) {
        memory[address] = value & 0xFF;
        memory[address + 1] = (value >> 8) & 0xFF;
        memory[address + 2] = (value >> 16) & 0xFF;
        memory[address + 3] = (value >> 24) & 0xFF;
    } else {
        printf("Error: Unrecognized alignment %d\n", alignment);
        exit(-1);
    }
}

Word load(Byte *memory, Address address, Alignment alignment) {
    if(alignment == LENGTH_BYTE) {
        return memory[address];
    } else if(alignment == LENGTH_HALF_WORD) {
        return (memory[address+1] << 8) + memory[address];
    } else if(alignment == LENGTH_WORD) {
        return (memory[address+3] << 24) + (memory[address+2] << 16)
               + (memory[address+1] << 8) + memory[address];
    } else {
        printf("Error: Unrecognized alignment %d\n", alignment);
        exit(-1);
    }
}