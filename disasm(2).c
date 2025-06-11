#include <stdio.h>
#include <stdlib.h>
#include "types.h"
#include "utils.h"

// Helper print functions declaration
void print_rtype(const char *name, Instruction instruction);
void print_itype_except_load(const char *name, Instruction instruction, int imm);
void print_load(const char *name, Instruction instruction);
void print_store(const char *name, Instruction instruction);
void print_branch(const char *name, Instruction instruction);
void print_lui(Instruction instruction);
void print_jal(Instruction instruction);
void print_ecall(void);

// Write functions declaration
void write_rtype(Instruction instruction);
void write_itype_except_load(Instruction instruction);
void write_load(Instruction instruction);
void write_store(Instruction instruction);
void write_branch(Instruction instruction);

void decode_instruction(uint32_t instruction_bits) {
    Instruction instruction = parse_instruction(instruction_bits);

    switch(instruction.opcode) {
        case 0x33: // R-type
            write_rtype(instruction);
            break;
        case 0x13: // I-type except loads
            write_itype_except_load(instruction);
            break;
        case 0x03: // Load instructions
            write_load(instruction);
            break;
        case 0x23: // Store instructions
            write_store(instruction);
            break;
        case 0x63: // Branch instructions
            write_branch(instruction);
            break;
        case 0x37: // LUI
            print_lui(instruction);
            break;
        case 0x6F: // JAL
            print_jal(instruction);
            break;
        case 0x73: // ECALL
            print_ecall();
            break;
        default:
            handle_invalid_instruction(instruction);
            break;
    }
}

void write_rtype(Instruction instruction) {
    uint8_t funct3 = instruction.rtype.funct3;
    uint8_t funct7 = instruction.rtype.funct7;

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
                print_rtype("sra", instruction);
            else
                handle_invalid_instruction(instruction);
            break;

        case 0x6:
            if (funct7 == 0x00)
                print_rtype("or", instruction);
            else if (funct7 == 0x01)
                print_rtype("rem", instruction);
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

void write_itype_except_load(Instruction instruction) {
    uint8_t funct3 = instruction.itype.funct3;
    int imm = sign_extend_number(instruction.itype.imm, 12);

    switch (funct3) {
        case 0x0: // addi
            print_itype_except_load("addi", instruction, imm);
            break;
        case 0x2: // slti
            print_itype_except_load("slti", instruction, imm);
            break;
        case 0x3: // sltiu
            print_itype_except_load("sltiu", instruction, imm);
            break;
        case 0x4: // xori
            print_itype_except_load("xori", instruction, imm);
            break;
        case 0x6: // ori
            print_itype_except_load("ori", instruction, imm);
            break;
        case 0x7: // andi
            print_itype_except_load("andi", instruction, imm);
            break;
        case 0x1: { // slli (shift left logical immediate)
            int shamt = instruction.itype.imm & 0x1F;
            print_itype_except_load("slli", instruction, shamt);
            break;
        }
        case 0x5: { // srli or srai
            int shamt = instruction.itype.imm & 0x1F;
            if ((instruction.itype.imm >> 10) & 0x1)
                print_itype_except_load("srai", instruction, shamt);
            else
                print_itype_except_load("srli", instruction, shamt);
            break;
        }
        default:
            handle_invalid_instruction(instruction);
            break;
    }
}

void write_load(Instruction instruction) {
    switch (instruction.itype.funct3) {
        case 0x0:
            print_load("lb", instruction);
            break;
        case 0x1:
            print_load("lh", instruction);
            break;
        case 0x2:
            print_load("lw", instruction);
            break;
        case 0x4:
            print_load("lbu", instruction);
            break;
        case 0x5:
            print_load("lhu", instruction);
            break;
        default:
            handle_invalid_instruction(instruction);
            break;
    }
}

void write_store(Instruction instruction) {
    switch (instruction.stype.funct3) {
        case 0x0:
            print_store("sb", instruction);
            break;
        case 0x1:
            print_store("sh", instruction);
            break;
        case 0x2:
            print_store("sw", instruction);
            break;
        default:
            handle_invalid_instruction(instruction);
            break;
    }
}

void write_branch(Instruction instruction) {
    switch (instruction.sbtype.funct3) {
        case 0x0:
            print_branch("beq", instruction);
            break;
        case 0x1:
            print_branch("bne", instruction);
            break;
        case 0x4:
            print_branch("blt", instruction);
            break;
        case 0x5:
            print_branch("bge", instruction);
            break;
        case 0x6:
            print_branch("bltu", instruction);
            break;
        case 0x7:
            print_branch("bgeu", instruction);
            break;
        default:
            handle_invalid_instruction(instruction);
            break;
    }
}

void print_rtype(const char *name, Instruction instruction) {
    printf(RTYPE_FORMAT, name, instruction.rtype.rd, instruction.rtype.rs1, instruction.rtype.rs2);
}

void print_itype_except_load(const char *name, Instruction instruction, int imm) {
    printf(ITYPE_FORMAT, name, instruction.itype.rd, instruction.itype.rs1, imm);
}

void print_load(const char *name, Instruction instruction) {
    int imm = sign_extend_number(instruction.itype.imm, 12);
    printf(MEM_FORMAT, name, instruction.itype.rd, imm, instruction.itype.rs1);
}

void print_store(const char *name, Instruction instruction) {
    int imm = get_store_offset(instruction);
    printf(MEM_FORMAT, name, instruction.stype.rs2, imm, instruction.stype.rs1);
}

void print_branch(const char *name, Instruction instruction) {
    int offset = get_branch_offset(instruction);
    printf(BRANCH_FORMAT, name, instruction.sbtype.rs1, instruction.sbtype.rs2, offset);
}

void print_lui(Instruction instruction) {
    unsigned int imm = instruction.utype.imm;  // no shift here
    printf(LUI_FORMAT, instruction.utype.rd, imm);
}

void print_jal(Instruction instruction) {
    int offset = get_jump_offset(instruction);
    printf(JAL_FORMAT, instruction.ujtype.rd, offset);
}

void print_ecall(void) {
    printf(ECALL_FORMAT);
}
