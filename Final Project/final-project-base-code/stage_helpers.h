#ifndef __STAGE_HELPERS_H__
#define __STAGE_HELPERS_H__

#include <stdio.h>
#include "utils.h"
#include "pipeline.h"

/// EXECUTE STAGE HELPERS ///

/**
 * input  : idex_reg_t
 * output : uint32_t alu_control signal
 **/

uint32_t gen_alu_control(idex_reg_t idex_reg){
  uint32_t alu_control = 0; // Initialize ALU control signal variable to 0
  
  switch(idex_reg.alu_op){ // Determine ALU operation type based on alu_op field
    
    case 0: // For load, store, branch, and jump instructions
      alu_control = 0;   // ALU control set to 0 (usually ADD)
      break;

    case 01: // I-type instructions
      switch (idex_reg.instr.itype.funct3){ // Check funct3 field in I-type instruction
        
        case 0x0: // ADDI (Add immediate)
          alu_control = 0x0; // Set ALU control to 0 (ADD)
          break;
          
        case 0x1: // SLLI (Shift left logical immediate)
          alu_control = 0x3; // Set ALU control to 3 (SLL)
          break;
          
        case 0x2: // SLTI (Set less than immediate)
          alu_control = 0x5; // Set ALU control to 5 (SLT)
          break;
          
        case 0x4: // XORI (XOR immediate)
          alu_control = 0x6; // Set ALU control to 6 (XOR)
          break;
          
        case 0x5: // SRLI or SRAI (Shift right logical/arithmetic immediate)
          // Check the upper bits of immediate to distinguish SRLI vs SRAI
          if ((idex_reg.instr.itype.imm >> 5) == 0x0){ // If upper bits are 0, it's SRLI
            alu_control = 0x8; // ALU control for SRL
          }
          else if ((idex_reg.instr.itype.imm >> 5) == 0x20){ // If upper bits are 0x20, it's SRAI
            alu_control = 0x9; // ALU control for SRA
          }
          break;
          
        case 0x6: // ORI (OR immediate)
          alu_control = 0x10; // ALU control for OR
          break;
          
        case 0x7: // ANDI (AND immediate)
          alu_control = 0x12; // ALU control for AND
          break;
          
        default: // Default case for unknown funct3 values
          break;
      }
      break;

    case 10: // R-type instructions
      switch (idex_reg.instr.rtype.funct3){ // Inspect funct3 field in R-type instruction
        
        case 0x0: // Funct3 = 0x0 (ADD, MUL, SUB)
          switch (idex_reg.instr.rtype.funct7){ // Inspect funct7 field
            case 0x00: // ADD instruction
              alu_control = 0x0; // ALU control for ADD
              break;
            case 0x01: // MUL instruction
              alu_control = 0x1; // ALU control for MUL
              break;
            case 0x20: // SUB instruction
              alu_control = 0x2; // ALU control for SUB
              break;
          }
          break;
          
        case 0x1: // Funct3 = 0x1 (SLL, MULH)
          switch (idex_reg.instr.rtype.funct7){
            case 0x0: // SLL (Shift left logical)
              alu_control = 0x3; // ALU control for SLL
              break;
            case 0x01: // MULH (Multiply high)
              alu_control = 0x4; // ALU control for MULH
              break;
          }
          break;
          
        case 0x2: // Funct3 = 0x2 (SLT)
          switch (idex_reg.instr.rtype.funct7){
            case 0x0: // SLT (Set less than)
              alu_control = 0x5; // ALU control for SLT
              break;
          }
          break;
          
        case 0x4: // Funct3 = 0x4 (XOR, DIV)
          switch (idex_reg.instr.rtype.funct7){
            case 0x0: // XOR
              alu_control = 0x6; // ALU control for XOR
              break;
            case 0x01: // DIV (Divide)
              alu_control = 0x7; // ALU control for DIV
              break;
          }
          break;
          
        case 0x5: // Funct3 = 0x5 (SRL, SRA)
          switch (idex_reg.instr.rtype.funct7){
            case 0x0: // SRL (Shift right logical)
              alu_control = 0x8; // ALU control for SRL
              break;
            case 0x20: // SRA (Shift right arithmetic)
              alu_control = 0x9; // ALU control for SRA
              break;
          }
          break;
          
        case 0x6: // Funct3 = 0x6 (OR, REM)
          switch (idex_reg.instr.rtype.funct7){
            case 0x0: // OR
              alu_control = 0x10; // ALU control for OR
              break;
            case 0x01: // REM (Remainder)
              alu_control = 0x11; // ALU control for REM
              break;
          }
          break;
          
        case 0x7: // Funct3 = 0x7 (AND)
          switch (idex_reg.instr.rtype.funct7){
            case 0x0: // AND
              alu_control = 0x12; // ALU control for AND
              break;
          }
          break;
          
        default: // Default catch-all
          break;
      }
      break;

    case 11: // Unknown/Reserved ALU operation (possibly system)
      alu_control = 0x13; // Set ALU control to 0x13 (special code)
      break;

    default: // If alu_op does not match any case
      break;
  }
  
  return alu_control; // Return the computed ALU control code
}

/**
 * input  : alu_inp1, alu_inp2, alu_control
 * output : uint32_t alu_result
 **/

uint32_t execute_alu(uint32_t alu_inp1, uint32_t alu_inp2, uint32_t alu_control)
{
  uint32_t result = 0; // Initialize result variable

  switch (alu_control){ // Select operation based on ALU control code

    case 0x0: // ADD operation
      result = alu_inp1 + alu_inp2; // Perform addition
      break;

    case 0x1: // MUL operation
      result = alu_inp1 * alu_inp2; // Perform multiplication
      break;

    case 0x2: // SUB operation
      result = alu_inp1 - alu_inp2; // Perform subtraction
      break;

    case 0x3: // SLL operation (Shift left logical)
      result = alu_inp1 << alu_inp2; // Shift alu_inp1 left by alu_inp2 bits
      break;

    case 0x4: // MULH operation (Multiply high)
      // Multiply as 64-bit, then shift right 32 bits to get higher half
      result = ((uint64_t)alu_inp1 * (uint64_t)alu_inp2) >> 32;
      break;

    case 0x5: // SLT operation (Set less than)
      // Compare as signed integers, result is 1 if alu_inp1 < alu_inp2, else 0
      result = ((sWord)alu_inp1 < (sWord)alu_inp2);
      break;

    case 0x6: // XOR operation
      result = alu_inp1 ^ alu_inp2; // Bitwise XOR
      break;

    case 0x7: // DIV operation (Division)
      result = alu_inp1 / alu_inp2; // Divide alu_inp1 by alu_inp2
      break;

    case 0x8: // SRL operation (Shift right logical)
      result = alu_inp1 >> alu_inp2; // Shift alu_inp1 right by alu_inp2 bits
      break;

    case 0x9: // SRA operation (Shift right arithmetic)
      // Arithmetic shift right with sign extension on alu_inp1 by alu_inp2 bits
      result = sign_extend_number((sWord)alu_inp1 >> alu_inp2, 31);
      break;

    case 0x10: // OR operation
      result = alu_inp1 | alu_inp2; // Bitwise OR
      break;

    case 0x11: // REM operation (Remainder)
      result = alu_inp1 % alu_inp2; // Remainder of division
      break;

    case 0x12: // AND operation
      result = alu_inp1 & alu_inp2; // Bitwise AND
      break;

    case 0x13: // LUI operation (Load upper immediate)
      result = alu_inp2 << 12; // Shift alu_inp2 left by 12 bits (imm to upper bits)
      break;

    default: // Unknown ALU control code
      result = 0xBADCAFFE; // Error code value
      break;
  }

  return result; // Return the computed ALU result
}

/// DECODE STAGE HELPERS ///

/**
 * Generate immediate value from instruction
 * input  : Instruction instruction - the instruction to decode
 * output : uint32_t - sign-extended immediate value
 **/
uint32_t gen_imm(Instruction instruction)
{
  int imm_val = 0; // Initialize immediate value variable

  switch(instruction.opcode) { // Determine type of instruction based on opcode

    case 0x63: // SB-type (branch instructions)
      imm_val = get_branch_offset(instruction); // Compute branch offset immediate
      break;

    case 0x03: // I-type (load instructions)
    case 0x13: // I-type (immediate arithmetic instructions)
      imm_val = sign_extend_number(instruction.itype.imm, 12); // Sign-extend 12-bit immediate
      break;

    case 0x23: // S-type (store instructions)
      imm_val = get_store_offset(instruction); // Compute store offset immediate
      break;

    case 0x37: // U-type (LUI)
      imm_val = instruction.utype.imm; // Take U-type immediate directly (upper 20 bits)
      break;

    case 0x6f: // UJ-type (JAL instruction)
      imm_val = get_jump_offset(instruction); // Compute jump offset immediate
      break;

    default: // R-type and other undefined opcodes
      // No immediate value needed
      break;
  };

  return imm_val; // Return the generated immediate value
}

/**
 * generates all the control logic that flows around in the pipeline
 * input  : Instruction
 * output : idex_reg_t
 **/

idex_reg_t gen_control(Instruction instr)
{
  idex_reg_t idex_reg = {0}; // Initialize control register struct with zeros
  int rs1 = 0; // Source register 1 number
  int rs2 = 0; // Source register 2 number
  int rd = 0;  // Destination register number

  // Determine control signals and register fields based on instruction opcode
  switch (instr.opcode) {

    case 0x33: // R-type instructions
      rd = instr.rtype.rd;     // Destination register
      rs1 = instr.rtype.rs1;   // Source register 1
      rs2 = instr.rtype.rs2;   // Source register 2

      idex_reg.alu_src = 0;    // ALU source from register (not immediate)
      idex_reg.mem_to_reg = 0; // Result comes from ALU, not memory
      idex_reg.reg_write = 1;  // Enable write back to register file
      idex_reg.mem_read = 0;   // No memory read
      idex_reg.mem_write = 0;  // No memory write
      idex_reg.branch = 0;     // No branch
      idex_reg.alu_op = 10;     // ALU operation code for R-type
      break;

    case 0x3: // I-type load instructions
      rd = instr.itype.rd;     // Destination register
      rs1 = instr.itype.rs1;   // Source register 1

      idex_reg.alu_src = 1;    // ALU source is immediate (offset)
      idex_reg.mem_to_reg = 1; // Load data from memory to register
      idex_reg.reg_write = 1;  // Enable write back
      idex_reg.mem_read = 1;   // Memory read enabled
      idex_reg.mem_write = 0;  // No memory write
      idex_reg.branch = 0;     // No branch
      idex_reg.alu_op = 0;      // ALU op for load/store/branch/jump
      break;

    case 0x13: // I-type arithmetic instructions
      rd = instr.itype.rd;     // Destination register
      rs1 = instr.itype.rs1;   // Source register 1

      idex_reg.alu_src = 1;    // ALU source is immediate
      idex_reg.mem_to_reg = 0; // Result from ALU (not memory)
      idex_reg.reg_write = 1;  // Enable register write
      idex_reg.mem_read = 0;   // No memory read
      idex_reg.mem_write = 0;  // No memory write
      idex_reg.branch = 0;     // No branch
      idex_reg.alu_op = 1;      // ALU op code for I-type arithmetic (fixed 1, not octal)
      break;

    case 0x23: // S-type (store) instructions
      rs1 = instr.stype.rs1;   // Base register for address
      rs2 = instr.stype.rs2;   // Source register to store

      idex_reg.alu_src = 1;    // ALU source is immediate (offset)
      idex_reg.mem_to_reg = 0; // No register writeback
      idex_reg.reg_write = 0;  // Disable register write
      idex_reg.mem_read = 0;   // No memory read
      idex_reg.mem_write = 1;  // Enable memory write
      idex_reg.branch = 0;     // No branch
      idex_reg.alu_op = 0;      // ALU op for load/store/branch/jump
      break;

    case 0x63: // SB-type (branch) instructions
      rs1 = instr.sbtype.rs1;  // Source register 1
      rs2 = instr.sbtype.rs2;  // Source register 2

      idex_reg.alu_src = 0;    // ALU source from registers
      idex_reg.mem_to_reg = 0; // No register writeback
      idex_reg.reg_write = 0;  // Disable register write
      idex_reg.mem_read = 0;   // No memory read
      idex_reg.mem_write = 0;  // No memory write
      idex_reg.branch = 1;     // Enable branch control
      idex_reg.alu_op = 0;      // ALU op for load/store/branch/jump
      break;

    case 0x37: // U-type (LUI) instructions
      rd = instr.utype.rd;     // Destination register

      idex_reg.alu_src = 1;    // ALU source is immediate (upper bits)
      idex_reg.mem_to_reg = 0; // Result from ALU
      idex_reg.reg_write = 1;  // Enable register write
      idex_reg.mem_read = 0;   // No memory read
      idex_reg.mem_write = 0;  // No memory write
      idex_reg.branch = 0;     // No branch
      idex_reg.alu_op = 11;     // ALU op code for U-type
      break;

    case 0x6F: // UJ-type (JAL) instructions
      rd = instr.ujtype.rd;    // Destination register

      idex_reg.alu_src = 1;    // ALU source is immediate (jump offset)
      idex_reg.mem_to_reg = 0; // Result from ALU
      idex_reg.reg_write = 1;  // Enable register write
      idex_reg.mem_read = 0;   // No memory read
      idex_reg.mem_write = 0;  // No memory write
      idex_reg.branch = 1;     // Branch enabled (jump)
      idex_reg.alu_op = 0;      // ALU op for load/store/branch/jump
      break;

    default: // Undefined opcode: leave all control signals at 0
      break;
  }

  // Copy instruction and register numbers into idex_reg struct
  idex_reg.instr = instr;
  idex_reg.rdreg = rd;
  idex_reg.rs1reg = rs1;
  idex_reg.rs2reg = rs2;

  return idex_reg; // Return the populated control register struct
}

/// MEMORY STAGE HELPERS ///

/**
 * evaluates whether a branch must be taken
 * input  : <open to implementation>
 * output : bool
 **/

// Determine if a branch or jump should be taken based on instruction and register values
bool gen_branch(idex_reg_t idex_reg, pipeline_wires_t *pwires_p)
{
  // Check if instruction is branch type (opcode 0x63)
  if (idex_reg.instr.opcode == 0x63) {
    switch (idex_reg.instr.sbtype.funct3) {
      case 0x0: // BEQ: branch if equal
        if (idex_reg.rs1val == idex_reg.rs2val) {
          return true; // Branch taken
        }
        break;
      case 0x1: // BNE: branch if not equal
        if (idex_reg.rs1val != idex_reg.rs2val) {
          return true; // Branch taken
        }
        break;
      default:
        break; // Other branch types not handled here
    }
  }
  // For JAL (opcode 0x6F), always take branch (jump)
  else if (idex_reg.instr.opcode == 0x6F) {
    return true;
  }
  return false; // Otherwise, do not take branch
}

// Extract rs2 register number based on instruction format
uint32_t rs2r(Instruction instr) {
  uint32_t rs2 = 0;

  switch(instr.opcode) {
    case 0x33: // R-type
      rs2 = instr.rtype.rs2;
      break; // missing break added here

    case 0x23: // S-type (store)
      rs2 = instr.stype.rs2;
      break;

    case 0x63: // SB-type (branch)
      rs2 = instr.sbtype.rs2;
      break;

    default:
      break; // Other instruction formats do not have rs2
  }
  return rs2;
}

// Extract rs1 register number based on instruction format
uint32_t rs1r(Instruction instr) {
  uint32_t rs1 = 0;

  switch(instr.opcode) {
    case 0x33: // R-type
      rs1 = instr.rtype.rs1;
      break;

    case 0x3: // I-type load
      rs1 = instr.itype.rs1;
      break;

    case 0x13: // I-type arithmetic
      rs1 = instr.itype.rs1;
      break;

    case 0x23: // S-type store
      rs1 = instr.stype.rs1;
      break;

    case 0x63: // SB-type branch
      rs1 = instr.sbtype.rs1;
      break;

    default:
      break; // Other instructions do not use rs1
  }
  return rs1;
}

// Extract destination register number (rd) based on instruction format
uint32_t rdr(Instruction instr) {
  uint32_t rd = 0;

  switch(instr.opcode) {
    case 0x33: // R-type
      rd = instr.rtype.rd;
      break;

    case 0x3: // I-type load
      rd = instr.itype.rd;
      break;

    case 0x13: // I-type arithmetic
      rd = instr.itype.rd;
      break;

    case 0x37: // U-type (LUI)
      rd = instr.utype.rd;
      break;

    case 0x6F: // UJ-type (JAL)
      rd = instr.ujtype.rd;
      break;

    default:
      break; // Other instructions do not write to rd
  }
  return rd;
}

/// PIPELINE FORWARDING CONTROL ///

/**
 * Update forwarding signals for resolving data hazards.
 * Inputs:
 *   - pregs_p: pointer to pipeline registers struct
 *   - pwires_p: pointer to pipeline wires struct
 *   - i: integer indicating which forwarding case to apply
 * Behavior:
 *   - Updates idex stage register values with forwarded data from later pipeline stages
 *   - Increments forwarding counters
 *   - Optionally prints debug info if enabled
 */
void gen_forward(pipeline_regs_t *pregs_p, pipeline_wires_t *pwires_p, int i)
{
  if (pwires_p->fwdreg != 0) { // Check forwarding register is valid
    switch (i) {
      case 0: // EX hazard on rs1
        fwd_exex_counter++;
        pregs_p->idex_preg.out.rs1val = pregs_p->exmem_preg.out.result;
        #ifdef DEBUG_CYCLE
          printf("[FWD]: Resolving EX hazard on rs1: x%d\n", pwires_p->fwdreg);
        #endif
        break;
      case 1: // EX hazard on rs2
        fwd_exex_counter++;
        pregs_p->idex_preg.out.rs2val = pregs_p->exmem_preg.out.result;
        #ifdef DEBUG_CYCLE
          printf("[FWD]: Resolving EX hazard on rs2: x%d\n", pwires_p->fwdreg);
        #endif
        break;
      case 2: // MEM hazard on rs1
        fwd_exmem_counter++;
        pregs_p->idex_preg.out.rs1val = pregs_p->memwb_preg.out.result;
        #ifdef DEBUG_CYCLE
          printf("[FWD]: Resolving MEM hazard on rs1: x%d\n", pwires_p->fwdreg);
        #endif
        break;
      case 3: // MEM hazard on rs2
        fwd_exmem_counter++;
        pregs_p->idex_preg.out.rs2val = pregs_p->memwb_preg.out.result;
        #ifdef DEBUG_CYCLE
          printf("[FWD]: Resolving MEM hazard on rs2: x%d\n", pwires_p->fwdreg);
        #endif
        break;
      default:
        break;
    }
  }
}

/**
 * Task   : Sets the pipeline wires for the hazard unit's control signals
 *           based on the pipeline register values.
 * input  : pipeline_regs_t*, pipeline_wires_t*
 * output : None
 */

void detect_hazard(pipeline_regs_t *pregs_p, pipeline_wires_t *pwires_p, regfile_t *regfile_p, int i)
{
  if (i == 0) {  // Forwarding and control hazard detection
    pwires_p->fwdreg = 0;  // Initialize forwarding register to zero (no forwarding)

    // Check EX hazard on rs1: if EX/MEM destination register matches ID/EX rs1 register
    if (pregs_p->exmem_preg.out.rdreg == pregs_p->idex_preg.out.rs1reg) {
      pwires_p->fwdreg = pregs_p->exmem_preg.out.rdreg; // Set forwarding register
      gen_forward(pregs_p, pwires_p, 0);                // Forward EX hazard for rs1
    }

    // Check EX hazard on rs2: if EX/MEM destination register matches ID/EX rs2 register
    if (pregs_p->exmem_preg.out.rdreg == pregs_p->idex_preg.out.rs2reg) {
      pwires_p->fwdreg = pregs_p->exmem_preg.out.rdreg; // Set forwarding register
      gen_forward(pregs_p, pwires_p, 1);                // Forward EX hazard for rs2
    }

    // Check MEM hazard only if MEM/WB destination register is different from forwarding register
    if (pregs_p->memwb_preg.inp.rdreg != pwires_p->fwdreg) {
      // MEM hazard on rs1: if MEM/WB destination matches ID/EX rs1
      if (pregs_p->memwb_preg.out.rdreg == pregs_p->idex_preg.out.rs1reg) {
        pwires_p->fwdreg = pregs_p->memwb_preg.inp.rdreg;
        gen_forward(pregs_p, pwires_p, 2);              // Forward MEM hazard for rs1
      }
      // MEM hazard on rs2: if MEM/WB destination matches ID/EX rs2
      if (pregs_p->memwb_preg.out.rdreg == pregs_p->idex_preg.out.rs2reg) {
        pwires_p->fwdreg = pregs_p->memwb_preg.inp.rdreg;
        gen_forward(pregs_p, pwires_p, 3);              // Forward MEM hazard for rs2
      }
    }

    // Check for control hazards (branches)
    if (gen_branch(pregs_p->idex_preg.out, pwires_p)) {
      pwires_p->fixcontrol = true; // Set control hazard flag to flush pipeline
    }
  }

  if (i == 1) {  // Load-use hazard detection and stall logic
    if (pwires_p->fixload) { // If load hazard fix is pending
      pregs_p->idex_preg.inp.rdreg = 0; // Zero destination register in pipeline register input
      pregs_p->idex_preg.out.rdreg = 0; // Zero destination register in pipeline register output
      pwires_p->fixload = false;         // Clear fixload flag
    }

    // Check for load-use hazard:
    // If current instruction in ID/EX is a load (opcode 0x3)
    // and IF/ID rs1 or rs2 matches ID/EX destination register
    if (pregs_p->idex_preg.out.instr.opcode == 0x3 &&
        (pregs_p->ifid_preg.out.rs1reg == pregs_p->idex_preg.out.rdreg ||
         pregs_p->ifid_preg.out.rs2reg == pregs_p->idex_preg.out.rdreg)) {

      regfile_p->PC -= 4; // Stall PC by decrementing to re-fetch same instruction

      // Create bubble by rewriting IF/ID pipeline input with current IF/ID output (stalling)
      pregs_p->ifid_preg.inp.instrAddr = pregs_p->ifid_preg.out.instrAddr;
      pregs_p->ifid_preg.inp.instr = pregs_p->ifid_preg.out.instr;

      #ifdef DEBUG_CYCLE
        printf("[HZD]: Stalling and rewriting PC: 0x0000%x\n", regfile_p->PC);
      #endif

      stall_counter++;       // Increment stall count
      pwires_p->fixload = true; // Set load hazard fix flag to handle bubble next cycle
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

/// RESERVED FOR PRINTING REGISTER TRACE AFTER EACH CLOCK CYCLE ///

void print_register_trace(regfile_t *regfile_p)
{
  // Print registers in 8 rows, 4 columns each (total 32 registers)
  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t j = 0; j < 4; j++) {
      uint8_t reg_idx = i * 4 + j;
      // Print register index and its 8-digit hex value
      printf("r%2d=%08x ", reg_idx, regfile_p->R[reg_idx]);
    }
    printf("\n");  // New line after each row
  }
  printf("\n");  // Extra newline for spacing
}


#endif // __STAGE_HELPERS_H__