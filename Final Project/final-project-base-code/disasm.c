#include <stdio.h>                       // Standard I/O library for printf
#include <stdlib.h>                      // Standard library 
#include "types.h"                       // Contains definition of Instruction and other types
#include "utils.h"                       // Contains helper functions like sign_extend_number, etc.

// Declaration of helper functions that print the decoded instruction to the console
void print_rtype(const char *name, Instruction instruction);
void print_itype_except_load(const char *name, Instruction instruction, int imm);
void print_load(const char *name, Instruction instruction);
void print_store(const char *name, Instruction instruction);
void print_branch(const char *name, Instruction instruction);
void print_lui(Instruction instruction);
void print_jal(Instruction instruction);
void print_ecall(void);

// Declaration of write functions for each instruction type
void write_rtype(Instruction instruction);
void write_itype_except_load(Instruction instruction);
void write_load(Instruction instruction);
void write_store(Instruction instruction);
void write_branch(Instruction instruction);

// Decodes the 32-bit instruction and dispatches it to the proper handler
void decode_instruction(uint32_t instruction_bits) {
    Instruction instruction = parse_instruction(instruction_bits);  // Parse into structured format

    switch(instruction.opcode) {
        case 0x33: // R-type instructions (e.g., add, sub, mul)
            write_rtype(instruction);
            break;
        case 0x13: // I-type instructions (except loads)
            write_itype_except_load(instruction);
            break;
        case 0x03: // Load instructions (e.g., lw, lb, lh)
            write_load(instruction);
            break;
        case 0x23: // Store instructions (e.g., sw, sb, sh)
            write_store(instruction);
            break;
        case 0x63: // Branch instructions (e.g., beq, bne)
            write_branch(instruction);
            break;
        case 0x37: // LUI (load upper immediate)
            print_lui(instruction);
            break;
        case 0x6F: // JAL (jump and link)
            print_jal(instruction);
            break;
        case 0x73: // ECALL (system call)
            print_ecall();
            break;
        default:    // If opcode is not recognized, handle as invalid
            handle_invalid_instruction(instruction);
            break;
        }

}

// Handles R-type instructions based on funct3 and funct7
// Based on the combination of these fields it identifies the correct instruction and prints with the name and struct
void write_rtype(Instruction instruction) {
    uint8_t funct3 = instruction.rtype.funct3;  // Extract funct3
    uint8_t funct7 = instruction.rtype.funct7;  // Extract funct7

    switch (funct3) {
        case 0x0:
            if (funct7 == 0x00)
                print_rtype("add", instruction);
            else if (funct7 == 0x01)
                print_rtype("mul", instruction);
            else if (funct7 == 0x20)
                print_rtype("sub", instruction);
            else
                handle_invalid_instruction(instruction);
            break;

        case 0x1:
            if (funct7 == 0x00)
                print_rtype("sll", instruction);
            else if (funct7 == 0x01)
                print_rtype("mulh", instruction);
            else
                handle_invalid_instruction(instruction);
            break;

        case 0x2:
            if (funct7 == 0x00)
                print_rtype("slt", instruction);
            else
                handle_invalid_instruction(instruction);
            break;

        case 0x4:
            if (funct7 == 0x00)
                print_rtype("xor", instruction);
            else if (funct7 == 0x01)
                print_rtype("div", instruction);
            else
                handle_invalid_instruction(instruction);
            break;

        case 0x5:
            if (funct7 == 0x00)
                print_rtype("srl", instruction);
            else if (funct7 == 0x01)
                print_rtype("divu", instruction);
            else if (funct7 == 0x20)
                print_rtype("sra", instruction); // shift right arithematic
            else
                handle_invalid_instruction(instruction);
            break;

        case 0x6:
            if (funct7 == 0x00)
                print_rtype("or", instruction);
            else if (funct7 == 0x01)
                print_rtype("rem", instruction); //remainder
            else
                handle_invalid_instruction(instruction);
            break;

        case 0x7:
            if (funct7 == 0x00)
                print_rtype("and", instruction);
            else if (funct7 == 0x01)
                print_rtype("remu", instruction);
            else
                handle_invalid_instruction(instruction);
            break;

        default:
            handle_invalid_instruction(instruction);
            break;
    }
}

// Handles I-type arithmetic/logical instructions (not load)
void write_itype_except_load(Instruction instruction) {
    uint8_t funct3 = instruction.itype.funct3;                   // Extract funct3
    int imm = sign_extend_number(instruction.itype.imm, 12);     // Sign-extend the 12-bit immediate


    // Determine the exact instruction based on funct3 and funct7
    switch (funct3) {
        case 0x0:
            print_itype_except_load("addi", instruction, imm);  
            break;
        case 0x2:
            print_itype_except_load("slti", instruction, imm); // set less than immediate
            break;
        case 0x3:
            print_itype_except_load("sltiu", instruction, imm);  // set less than immediate unsigned
            break;
        case 0x4:
            print_itype_except_load("xori", instruction, imm); 
            break;
        case 0x6:
            print_itype_except_load("ori", instruction, imm); 
            break;
        case 0x7:
            print_itype_except_load("andi", instruction, imm); 
            break;
        case 0x1: {
            // For shift operations, only the lower 5 bits are used for the shift amount
            if ((instruction.itype.imm & ~0x1F) == 0) {
                int shamt = instruction.itype.imm & 0x1F; // get shift amount
                print_itype_except_load("slli", instruction, shamt); //logical shift left
            } else {
                handle_invalid_instruction(instruction);
            }
            break;
        }
        case 0x5: {
            int shamt = instruction.itype.imm & 0x1F;
            // Bit 10 of immediate distinguishes srai (arithmetic) from srli (logical)
            if ((instruction.itype.imm >> 10) & 0x1)
                print_itype_except_load("srai", instruction, shamt);  // Arithmetic shift right
            else
                print_itype_except_load("srli", instruction, shamt);  // Logical shift right
            break;
        }
        default:
            handle_invalid_instruction(instruction);
            break;
    }
}

// Handles I-type load instructions
void write_load(Instruction instruction) {
    // Use funct3 to decide which load variant
    switch (instruction.itype.funct3) {
        case 0x0:
            print_load("lb", instruction); // load byte
            break;
        case 0x1:
            print_load("lh", instruction);  //load halfword
            break;
        case 0x2:
            print_load("lw", instruction); // load word
            break;
        case 0x4:
            print_load("lbu", instruction); // load byte unsigned
            break;
        case 0x5:
            print_load("lhu", instruction); // load halfword unsigned
            break;
        default:
            handle_invalid_instruction(instruction); // invalid
            break;
    }
}

// Handles S-type store instructions
void write_store(Instruction instruction) {
    switch (instruction.stype.funct3) {
        case 0x0:
            print_store("sb", instruction); //store byte
            break;
        case 0x1:
            print_store("sh", instruction); //store halfword
            break;
        case 0x2:
            print_store("sw", instruction); //store word
            break;
        default:
            handle_invalid_instruction(instruction); 
            break;
    }
}

// Handles SB-type branch instructions
void write_branch(Instruction instruction) {
    switch (instruction.sbtype.funct3) {
        case 0x0:
            print_branch("beq", instruction); // branch if equal
            break;
        case 0x1:
            print_branch("bne", instruction); // branch if not equal
            break;
        case 0x4:
            print_branch("blt", instruction); //branch if less than 
            break;
        case 0x5:
            print_branch("bge", instruction); // branch if greater/equal
            break;
        case 0x6:
            print_branch("bltu", instruction); //unsigned versions of 2 above
            break;
        case 0x7:
            print_branch("bgeu", instruction); 
            break;
        default:
            handle_invalid_instruction(instruction);
            break;
    }
}

// Each print function formats and prints the decoded instruction in a readable format 

// Print function for R-type instructions
void print_rtype(const char *name, Instruction instruction) {
    printf(RTYPE_FORMAT, name, instruction.rtype.rd, instruction.rtype.rs1, instruction.rtype.rs2);
}  // Ex. "add, x1, x2, x3"

// Print function for I-type (non-load) instructions
void print_itype_except_load(const char *name, Instruction instruction, int imm) {
    printf(ITYPE_FORMAT, name, instruction.itype.rd, instruction.itype.rs1, imm);
}  // Ex. "addi x1, x2, 10"

// Print function for load instructions
void print_load(const char *name, Instruction instruction) {
    int imm = sign_extend_number(instruction.itype.imm, 12);  // Sign-extend the immediate
    printf(MEM_FORMAT, name, instruction.itype.rd, imm, instruction.itype.rs1);
} // Computed with helper functions to account for how immediate values are laid out in the instruction bits 
//Ex. "lw x1 8(x2)"

// Print function for store instructions
void print_store(const char *name, Instruction instruction) {
    int imm = get_store_offset(instruction);  // Calculate the full offset
    printf(MEM_FORMAT, name, instruction.stype.rs2, imm, instruction.stype.rs1);
} //Ex."sw x3, 8(x2)"

// Print function for branch instructions
void print_branch(const char *name, Instruction instruction) {
    int offset = get_branch_offset(instruction);  // Calculate offset for jump
    printf(BRANCH_FORMAT, name, instruction.sbtype.rs1, instruction.sbtype.rs2, offset);
}//Ex."beq x1, x2, 16"

// Print function for LUI instruction
void print_lui(Instruction instruction) {
    unsigned int imm = instruction.utype.imm;  // No shift needed
    printf(LUI_FORMAT, instruction.utype.rd, imm);
} //Ex. "lui x1, 0x12345"

// Print function for JAL instruction
void print_jal(Instruction instruction) {
    int offset = get_jump_offset(instruction);  // Compute jump offset
    printf(JAL_FORMAT, instruction.ujtype.rd, offset);
}// Ex."jal x1, 1024"

// Print function for ECALL
void print_ecall(void) {
    printf(ECALL_FORMAT); //system calls to request a service
} 
