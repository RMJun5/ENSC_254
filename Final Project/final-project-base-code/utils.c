#include "utils.h"
#include <stdio.h>
#include <stdlib.h>

// Parses a 32-bit instruction into a structured format
Instruction parse_instruction(uint32_t instruction_bits) {
  Instruction instruction;
  instruction.bits = instruction_bits; // Store raw instruction bits

  // This opcode is extracted from the lowest 7 bits using a bitmask
  instruction.opcode = instruction_bits & 0x7F; // Extract opcode (bits 0-6)

  switch (instruction.opcode) {
  case 0x33: // R-Type instruction (e.g. add, sub, mul)
    instruction.rtype.rd = (instruction_bits >> 7) & 0x1F; // Extract rd (bits 7-11)
    instruction.rtype.funct3 = (instruction_bits >> 12) & 0x7; // Extract funct3 (bits 12-14)
    instruction.rtype.rs1 = (instruction_bits >> 15) & 0x1F; // Extract rs1 (bits 15-19)
    instruction.rtype.rs2 = (instruction_bits >> 20) & 0x1F; // Extract rs2 (bits 20-24)
    instruction.rtype.funct7 = (instruction_bits >> 25) & 0x7F; // Extract funct7 (bits 25-31)
    break;

  case 0x13: // I-Type instruction (e.g., arithmetic immediate)
  case 0x3:  // Load instruction
    instruction.itype.rd = (instruction_bits >> 7) & 0x1F; // Extract rd
    instruction.itype.funct3 = (instruction_bits >> 12) & 0x7; // Extract funct3
    instruction.itype.rs1 = (instruction_bits >> 15) & 0x1F; // Extract rs1
    instruction.itype.imm = (instruction_bits >> 20) & 0xFFF; // Extract 12-bit immediate
    break;

  case 0x23: // S-Type (Stores the Instructions)
    instruction.stype.imm5 = (instruction_bits >> 7) & 0x1F; // Extract imm[4:0] Bits 7-11
    instruction.stype.funct3 = (instruction_bits >> 12) & 0x7; // Extract funct3
    instruction.stype.rs1 = (instruction_bits >> 15) & 0x1F; // Extract rs1
    instruction.stype.rs2 = (instruction_bits >> 20) & 0x1F; // Extract rs2
    instruction.stype.imm7 = (instruction_bits >> 25) & 0x7F; // Extract imm[11:5] Bits 25-31
    break;

  case 0x63: // SB-Type (Branch Instructions) both imm are stored for now
    instruction.sbtype.imm5 = (instruction_bits >> 7) & 0x1F; // Extract imm[11:7]
    instruction.sbtype.funct3 = (instruction_bits >> 12) & 0x7; // Extract funct3
    instruction.sbtype.rs1 = (instruction_bits >> 15) & 0x1F; // Extract rs1
    instruction.sbtype.rs2 = (instruction_bits >> 20) & 0x1F; // Extract rs2
    instruction.sbtype.imm7 = (instruction_bits >> 25) & 0x7F; // Extract imm[12|10:5]
    break;

  case 0x37: // U-Type (LUI)
    instruction.utype.rd = (instruction_bits >> 7) & 0x1F;      // bits [11:7]
    instruction.utype.imm = (instruction_bits >> 12) & 0xFFFFF; // 20 -bit immediate bits [31:12]
    break;
  case 0x6F: // UJ-Type (JAL)
  case 0x73: // I-Type (ECALL)
    // No additional unpacking for these here
    break;

  default:
  // #ifndef TESTING
  //   exit(EXIT_FAILURE); // Exit on unknown instruction unless testing
  // #endif
    break;
  }
  return instruction; // Return filled instruction struct
}

/************************Helper functions************************/

// Sign extends field interpreted as n-bit number to 32 bits
int sign_extend_number(unsigned int field, unsigned int n) {
  if ((field >> (n - 1)) & 1) { // Check sign bit   1 indicates a negative # in two's complement
    return field | (~0u << n); // Fill upper bits with 1s 
  } else {
    return field; // Positive, no change
  }
}

// Calculates branch offset (SB-type format)
int get_branch_offset(Instruction instruction) {
  int imm = (((instruction.bits >> 31) & 0x1) << 12) | // imm[12]
            (((instruction.bits >> 7) & 0x1) << 11) |  // imm[11]
            (((instruction.bits >> 25) & 0x3F) << 5) | // imm[10:5]
            (((instruction.bits >> 8) & 0xF) << 1);    // imm[4:1]
  return sign_extend_number(imm, 13); // Sign-extend 13-bit offset  extend it to 32 bits
}

// Calculates jump offset (UJ-type format)
int get_jump_offset(Instruction instruction) {
  int imm = (((instruction.bits >> 31) & 0x1) << 20) | // imm[20]
            (((instruction.bits >> 12) & 0xFF) << 12) | // imm[19:12]
            (((instruction.bits >> 20) & 0x1) << 11) | // imm[11]
            (((instruction.bits >> 21) & 0x3FF) << 1); // imm[10:1]
  return sign_extend_number(imm, 21); // Sign-extend 21-bit offset  32-bit signed jump offset
}

// Calculates store offset (S-type format)
int get_store_offset(Instruction instruction) {
  int imm = (((instruction.bits >> 25) & 0x7F) << 5) | // imm[11:5]  Bits 25-31
            ((instruction.bits >> 7) & 0x1F);           // imm[4:0]  Bits 7-11
  return sign_extend_number(imm, 12); // Sign-extend 12-bit offset
}

/************************Helper functions************************/
// Error Handling functions 
// Prints message for invalid instruction
void handle_invalid_instruction(Instruction instruction) {
  //printf("Invalid Instruction: 0x%08x\n", instruction.bits);
  printf("\n");
}// Print raw instruction as hex if it cannot be parsed

// Prints message for invalid memory access read and exits
void handle_invalid_read(Address address) {
  printf("Bad Read. Address: 0x%08x\n", address);
  exit(-1);
}// Terminates the program using exit(-1)

// Prints message for invalid write and exits
void handle_invalid_write(Address address) {
  printf("Bad Write. Address: 0x%08x\n", address);
  exit(-1);
}
