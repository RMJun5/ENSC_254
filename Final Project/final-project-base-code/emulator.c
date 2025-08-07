#include <stdio.h> // for stderr and printf
#include <stdlib.h> // for exit()
#include "types.h" // includes definitions for Instruction, Processor, etc.
#include "utils.h" // includes helper functions like sign_extend_number, etc.
#include "riscv.h" // includes declarations for riscv-specific functions

// Function declarations for instruction execution
void execute_rtype(Instruction, Processor *);
void execute_itype_except_load(Instruction, Processor *);
void execute_branch(Instruction, Processor *);
void execute_jal(Instruction, Processor *);
void execute_load(Instruction, Processor *, Byte *);
void execute_store(Instruction, Processor *, Byte *);
void execute_ecall(Processor *, Byte *);
void execute_lui(Instruction, Processor *);

// Executes a decoded instruction
void execute_instruction(uint32_t instruction_bits, Processor *processor, Byte *memory) {    
    Instruction instruction = parse_instruction(instruction_bits); // Parse raw bits to structured Instruction
    switch(instruction.opcode) {
        case 0x33:
            execute_rtype(instruction, processor); // R-type
            break;
        case 0x13:
            execute_itype_except_load(instruction, processor); // I-type (arithmetic/logical)
            break;
        case 0x73:
            execute_ecall(processor, memory); // ECALL
            break;
        case 0x63:
            execute_branch(instruction, processor); // Branch
            break;
        case 0x6F:
            execute_jal(instruction, processor); // JAL
            break;
        case 0x23:
            execute_store(instruction, processor, memory); // Store
            break;
        case 0x03:
            execute_load(instruction, processor, memory); // Load
            break;
        case 0x37:
            execute_lui(instruction, processor); // LUI
            break;
        default: // undefined opcode
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
    // Ensure x0 is always zero
    processor->R[0] = 0;
}

// Executes R-type instructions based on funct3 and funct7 fields
void execute_rtype(Instruction instruction, Processor *processor) {
    // Switch based on funct3 to narrow down the type of R-type instruction
    switch (instruction.rtype.funct3){
        case 0x0:  // funct3 == 000: add, sub, mul
            switch (instruction.rtype.funct7) {
                case 0x0:
                    // Add: Signed addition of rs1 and rs2 → store result in rd
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1]) +
                        ((sWord)processor->R[instruction.rtype.rs2]);
                    break;
                case 0x1:
                    // Mul: Signed multiplication of rs1 and rs2 → store result in rd
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1]) *
                        ((sWord)processor->R[instruction.rtype.rs2]);
                    break;
                case 0x20:
                    // Sub: Signed subtraction rs1 - rs2 → store result in rd
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1]) -
                        ((sWord)processor->R[instruction.rtype.rs2]);
                    break;
                default:
                    // Invalid funct7 value for funct3 == 0
                    handle_invalid_instruction(instruction);
                    exit(-1);
                    break;
            }
            break;
        case 0x1: // SLL, MULH
            switch (instruction.rtype.funct7) {
                case 0x00: // SLL (logical shift left)
                    processor->R[instruction.rtype.rd] =
                        processor->R[instruction.rtype.rs1] <<
                        (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                case 0x01: // MULH (high 32 bits of signed multiplication)
                    processor->R[instruction.rtype.rd] =
                        ((int64_t)((sWord)processor->R[instruction.rtype.rs1]) *
                         (int64_t)((sWord)processor->R[instruction.rtype.rs2])) >> 32;
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        case 0x2: // SLT (set if less than for signed - for values that may be negative)
            switch (instruction.rtype.funct7) {
                case 0x00:
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1] <
                         (sWord)processor->R[instruction.rtype.rs2]) ? 1 : 0;
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        case 0x3: // SLTU (set if less than for unsigned - for raw bit values, memory addresses, etc.)
            switch (instruction.rtype.funct7) {
                case 0x00:
                    processor->R[instruction.rtype.rd] =
                        (processor->R[instruction.rtype.rs1] <
                         processor->R[instruction.rtype.rs2]) ? 1 : 0;
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        case 0x4: // XOR, DIV
            switch (instruction.rtype.funct7) {
                case 0x00: // XOR
                    processor->R[instruction.rtype.rd] =
                        (Word)processor->R[instruction.rtype.rs1]^
                        (Word)processor->R[instruction.rtype.rs2];
                    break;
                case 0x01: // DIV
                    processor->R[instruction.rtype.rd] =
                        (sWord)processor->R[instruction.rtype.rs1] /
                        (sWord)processor->R[instruction.rtype.rs2];
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        case 0x5: // SRL, SRA
            switch (instruction.rtype.funct7) {
                case 0x00: // SRL (logical shift right)
                    processor->R[instruction.rtype.rd] =
                        processor->R[instruction.rtype.rs1] >>
                        (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                case 0x20: // SRA (arithmetic right shift sign-extended)
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1]) >>
                        (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                case 0x01: // DIVU (unsigned integer division)
                    processor->R[instruction.rtype.rd] =
                        processor->R[instruction.rtype.rs1] /
                        processor->R[instruction.rtype.rs2];
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        case 0x6: // OR, REM
            switch (instruction.rtype.funct7) {
                case 0x00: // OR
                    processor->R[instruction.rtype.rd] =
                        processor->R[instruction.rtype.rs1] |
                        processor->R[instruction.rtype.rs2];
                    break;
                case 0x01: // REM  (signed remainder - modulus)
                    processor->R[instruction.rtype.rd] =
                        (sWord)processor->R[instruction.rtype.rs1] %
                        (sWord)processor->R[instruction.rtype.rs2];
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        case 0x7: // AND, REMU (remainder unsigned)
            switch (instruction.rtype.funct7) {
                case 0x00: // AND
                    processor->R[instruction.rtype.rd] =
                        processor->R[instruction.rtype.rs1] &
                        processor->R[instruction.rtype.rs2];
                    break;
                case 0x01: // REMU
                    processor->R[instruction.rtype.rd] =
                        processor->R[instruction.rtype.rs1] %
                        processor->R[instruction.rtype.rs2];
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        default:
            handle_invalid_instruction(instruction);
            exit(-1);
    }
    processor->PC += 4; // increments the program counter (PC) by 4 bytes moves to the next instruction
    processor->R[0] = 0; // Ensure x0 remains zero
}
// Handles I-type arithmetic and logical instructions that do not involve memory loading
void execute_itype_except_load(Instruction instruction, Processor *processor) { //opcode 0x13
    switch (instruction.itype.funct3) {
        case 0x0:
        //addi Add Immediate (signed addition of a register and a sign-extended immediate)
            processor->R[instruction.itype.rd] = 
            ((sWord)processor->R[instruction.itype.rs1]) + 
            (sWord)sign_extend_number(instruction.itype.imm, 12);
            // Add rs1 to the sign-extended 12-bit immediate, store in rd
            break;
        case 0x1:
         //slli Shift Left Logical Immediate
            switch (instruction.itype.imm >> 5){ 
                //Check that upper 7 bits of the immediate are valid (should be 0 for slli)
                case 0x0:
                    processor->R[instruction.itype.rd] =
                    ((Word)processor->R[instruction.itype.rs1]) << 
                    ((Word)instruction.itype.imm & 0x1F);
                    // Perform logical left shift of rs1 by lower 5 bits of imm, store in rd
                    break;
                default:
                    handle_invalid_instruction(instruction); // Invalid case
                    exit(-1);
                    break;
            }
            break;
        case 0x2:
        //slti Set Less Than Immediate (signed)
            processor->R[instruction.itype.rd] = 
            ((sWord)processor->R[instruction.itype.rs1]) < 
            ((sWord)sign_extend_number(instruction.itype.imm, 12)) ? 1 : 0;
            // If rs1 < imm, set rd = 1; otherwise, rd = 0.
            break;
        case 0x4:
        //xori Bitwise XOR with Immediate
            processor->R[instruction.itype.rd] = ((Word)processor->R[instruction.itype.rs1]) ^ ((Word)sign_extend_number(instruction.itype.imm, 12));
            // Bitwise XOR rs1 with sign-extended imm, store in rd
            break;
        case 0x5:
        //srli (Logical Shift Right Immediate) and srai (Arithmetic Shift Right Immediate)
            switch ((instruction.itype.imm >> 5) & 0x7F ){
                case 0x0:
                    processor->R[instruction.itype.rd] = ((Word)processor->R[instruction.itype.rs1]) >> ((Word)instruction.itype.imm & 0x1F);
                    break;
                case 0x20:
                    processor->R[instruction.itype.rd] = ((sWord)processor->R[instruction.itype.rs1] >> (instruction.itype.imm & 0x1F));
                    break;
                default:
                    handle_invalid_instruction(instruction); // Invalid case
                    exit(-1);
                    break;
            }
            break;
        case 0x6:
        //ori Bitwise OR with Immediate
            processor->R[instruction.itype.rd] = ((Word)processor->R[instruction.itype.rs1]) | (Word)sign_extend_number(instruction.itype.imm, 12);
            // Perform bitwise OR and store in rd
            break;
        case 0x7:
        //andi Bitwise AND with Immediate
            processor->R[instruction.itype.rd] = (processor->R[instruction.itype.rs1]) & sign_extend_number(instruction.itype.imm, 12);
            // Perform bitwise AND between rs1 and sign-extended imm
            break;
        default:
            handle_invalid_instruction(instruction); // Any other funct3 is not valid for opcode 0x13
            exit(-1);
            break;
    }
    processor->PC += 4; // increments the program counter (PC) by 4 bytes moves to the next instruction
    processor->R[0] = 0; // Ensure x0 remains zero
}

// Handles system call (ecall) instructions based on value in register x10 (a0)
void execute_ecall(Processor *p, Byte *memory) {
    Register i;
    // Check value in register x10 (a0), which determines the syscall type
    switch(p->R[10]) {
        case 1: // print an integer
            printf("%d",p->R[11]);
            p->PC += 4; // Move to next instruction
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
    p->R[0] = 0; // Always keep register x0 set to 0
}

// Handles branch instructions (e.g., beq, bne)
void execute_branch(Instruction instruction, Processor *processor) {
    int taken = 0; // Flag to indicate if branch is taken
    switch (instruction.sbtype.funct3) {
        case 0x0: // beq (branch if equal)
            if ((sWord)processor->R[instruction.sbtype.rs1] == (sWord)processor->R[instruction.sbtype.rs2]) {
                processor->PC += get_branch_offset(instruction);
                taken = 1;
                // Compare registers rs1 and rs2 as signed integers
                // If equal, apply branch offset to PC
            }
            break;
        case 0x1: // bne (branch if not equal)
            if ((sWord)processor->R[instruction.sbtype.rs1] != (sWord)processor->R[instruction.sbtype.rs2]) {
                processor->PC += get_branch_offset(instruction);
                taken = 1;
                // Compare registers rs1 and rs2 as signed integers
                // If not equal, apply branch offset to PC
            }
            break;
        default:
            // Invalid funct3 for branch instruction
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
    if (!taken) processor->PC += 4; // If the branch was not taken, move to the next instruction (normal PC += 4)
    processor->R[0] = 0; // Always keep register x0 set to 0
}

// Executes load instructions (e.g., LB, LH, LW, LBU, LHU)
void execute_load(Instruction instruction, Processor *processor, Byte *memory) {
    Address addr = processor->R[instruction.itype.rs1] + sign_extend_number(instruction.itype.imm, 12);
    switch (instruction.itype.funct3) {
        case 0x0: // LB (Load Byte, sign-extended)
            processor->R[instruction.itype.rd] = (sByte)load(memory, addr, LENGTH_BYTE);
            break;
        case 0x1: // LH (Load Half-word, sign-extended)
            processor->R[instruction.itype.rd] = (sHalf)load(memory, addr, LENGTH_HALF_WORD);
            break;
        case 0x2: // LW (Load Word, sign-extended)
            processor->R[instruction.itype.rd] = (sWord)load(memory, addr, LENGTH_WORD);
            break;
        case 0x4: // LBU (Load Byte Unsigned)
            processor->R[instruction.itype.rd] = (Byte)load(memory, addr, LENGTH_BYTE);
            break;
        case 0x5: // LHU (Load Half-word Unsigned)
            processor->R[instruction.itype.rd] = (Half)load(memory, addr, LENGTH_HALF_WORD);
            break;
        default: // Invalid funct3 for load
            handle_invalid_instruction(instruction);
            break;
    }
    processor->PC += 4; // Advance the program counter to the next instruction
    processor->R[0] = 0; // Ensure register x0 remains zero
}

// Executes store instructions: SB, SH, SW
void execute_store(Instruction instruction, Processor *processor, Byte *memory) {
    // Calculate effective memory address:
    // base address is in rs1, offset is sign-extended from immediate fields
    Address addr = processor->R[instruction.stype.rs1] + sign_extend_number(get_store_offset(instruction), 12);
    switch (instruction.stype.funct3) {
        case 0x0: // SB (Store Byte)
            // Store least significant byte of rs2 into memory at addr
            store(memory, addr, LENGTH_BYTE, processor->R[instruction.stype.rs2]);
            break;
        case 0x1: // SH (Store Half-word)
            // Store least significant 2 bytes of rs2 into memory at addr
            store(memory, addr, LENGTH_HALF_WORD, processor->R[instruction.stype.rs2]);
            break;
        case 0x2: // SW (Store Word)
            // Store full 4 bytes of rs2 into memory at addr
            store(memory, addr, LENGTH_WORD, processor->R[instruction.stype.rs2]);
            break;
        default: // Invalid funct3 for store
            handle_invalid_instruction(instruction);
            exit(-1); // Exit program
            break;
    }
    processor->PC += 4; // Move to the next instruction
}

// Executes JAL (Jump And Link) instruction
void execute_jal(Instruction instruction, Processor *processor) {
    // Calculate jump offset using instruction fields
    int offset = get_jump_offset(instruction);
    if (instruction.ujtype.rd > 31) {
        handle_invalid_instruction(instruction); // Invalid rd, handle error
        exit(-1); // Exit program due to invalid instruction
    } else {
        // Store the return address (PC + 4) into rd
        processor->R[instruction.ujtype.rd] = processor->PC + 4;
    }
    // Update PC to jump target address (PC + offset)
    processor->PC = processor->PC + offset;
    processor->R[0] = 0; // Ensure x0 is always zero
}

// Executes LUI (Load Upper Immediate) instruction
void execute_lui(Instruction instruction, Processor *processor) {
    // rd == 0 is invalid because x0 cannot be ten to
    if (instruction.utype.rd == 0) {
        handle_invalid_instruction(instruction); // Handle invalid rd error
        exit(-1);
    } else {
        // Load immediate value shifted left by 12 bits into rd
        processor->R[instruction.utype.rd] = instruction.utype.imm << 12;
        processor->PC += 4; // Advance to next instruction
    }
    processor->R[0] = 0; // Ensure x0 is always zero
}

// Store a value into memory with the specified alignment (byte, half-word, or word)
void store(Byte *memory, Address address, Alignment alignment, Word value) {
    if (alignment == LENGTH_BYTE) {
        // Store the least significant byte of 'value' into memory at 'address'
        memory[address] = value & 0xFF; 
    } else if (alignment == LENGTH_HALF_WORD) {
        // Store the least significant 2 bytes (16 bits) of 'value' into memory
        memory[address] = value & 0xFF;
        memory[address + 1] = (value >> 8) & 0xFF;
    } else if (alignment == LENGTH_WORD) {
        // Store all 4 bytes (32 bits) of 'value' into memory sequentially
        memory[address] = value & 0xFF;
        memory[address + 1] = (value >> 8) & 0xFF;
        memory[address + 2] = (value >> 16) & 0xFF;
        memory[address + 3] = (value >> 24) & 0xFF;
    } else {
        // Invalid alignment value: print error and exit
        printf("Error: Unrecognized alignment %d\n", alignment);
        exit(-1);
    }
}

// Load a value from memory with the specified alignment (byte, half-word, or word)
Word load(Byte *memory, Address address, Alignment alignment) {
    if(alignment == LENGTH_BYTE) {
        // Return a single byte from memory at 'address'
        return memory[address];
    } else if(alignment == LENGTH_HALF_WORD) {
        // Return two bytes (16 bits) combined as a half-word from memory
        return (memory[address+1] << 8) + memory[address];
        // Note: lower byte at address, higher byte at address+1
    } else if(alignment == LENGTH_WORD) {
        // Return four bytes (32 bits) combined as a word from memory
        return (memory[address+3] << 24) + (memory[address+2] << 16)
               + (memory[address+1] << 8) + memory[address];
        // Bytes combined in order (lowest byte at lowest address)
    } else {
        // Invalid alignment value: print error and exit
        printf("Error: Unrecognized alignment %d\n", alignment);
        exit(-1);
    }
}