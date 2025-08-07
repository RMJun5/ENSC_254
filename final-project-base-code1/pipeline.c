#include <stdbool.h>
#include "cache.h"
#include "riscv.h"
#include "types.h"
#include "utils.h"
#include "pipeline.h"
#include "stage_helpers.h"

// Counters for pipeline simulation statistics

uint64_t fwd_exex_counter = 0;    // Counts forwarding occurrences from EX to EX stage
uint64_t fwd_exmem_counter = 0;   // Counts forwarding occurrences from EX to MEM stage
uint64_t total_cycle_counter = 0; // Total clock cycles elapsed in simulation
uint64_t mem_access_counter = 0;  // Number of memory accesses (loads/stores)
uint64_t stall_counter = 0;       // Number of pipeline stalls due to hazards
uint64_t branch_counter = 0;      // Number of branch instructions executed
uint64_t miss_count = 0;          // Cache miss count during memory operations
uint64_t hit_count = 0;           // Cache hit count during memory operations

simulator_config_t sim_config = {0}; // Simulator configuration parameters

///////////////////////////////////////////////////////////////////////////////

void bootstrap(pipeline_wires_t *pwires_p, pipeline_regs_t *pregs_p, regfile_t *regfile_p)
{
  pwires_p->pc_src0 = regfile_p->PC;  // Initialize the first program counter source from the register file
}

///////////////////////////
/// STAGE FUNCTIONALITY ///
///////////////////////////

/**
 * STAGE  : stage_fetch
 * output : ifid_reg_t
 **/ 

ifid_reg_t stage_fetch(pipeline_wires_t *pwires_p, regfile_t *regfile_p, Byte *memory_p)
{
    ifid_reg_t ifid_reg = {0};                                      // Initialize IF/ID register to zero

    // Fetch the 32-bit instruction from memory at the current PC
    uint32_t fetched_instr = *(uint32_t *)(memory_p + regfile_p->PC);
    ifid_reg.instructionBits = fetched_instr;                       // Store raw bits
    ifid_reg.instr = parse_instruction(fetched_instr);             // Decode instruction format

    #ifdef DEBUG_CYCLE
        printf("[IF ]: Instruction [%08x]@[%08x]: ", fetched_instr, regfile_p->PC);
        decode_instruction(fetched_instr);                          // Display decoded instruction
    #endif

    if (pwires_p->pcsrc == 1) {                                     // If branch taken
        ifid_reg.instrAddr = regfile_p->PC;                         // Save current PC to IF/ID
        regfile_p->PC = pwires_p->pc_src1;                          // Jump to branch target
        branch_counter++;                                           // Track number of branches
        pwires_p->pcsrc = 0;                                        // Reset branch control
    } else {                                                        // Normal sequential PC increment
        ifid_reg.instrAddr = regfile_p->PC;                         // Save current PC to IF/ID
        regfile_p->PC += 4;                                         // Move to next instruction
        pwires_p->pc_src0 = regfile_p->PC;                          // Update next PC wire
    }

    // Extract register indices
    ifid_reg.rs1reg = rs1r(ifid_reg.instr);                         // Source register 1
    ifid_reg.rs2reg = rs2r(ifid_reg.instr);                         // Source register 2
    ifid_reg.rdreg = rdr(ifid_reg.instr);                           // Destination register

    return ifid_reg;                                                // Return filled IF/ID register
}

/**
 * STAGE  : stage_decode
 * output : idex_reg_t
 **/ 
idex_reg_t stage_decode(ifid_reg_t ifid_reg, pipeline_wires_t *pwires_p, regfile_t *regfile_p)
{
  idex_reg_t idex_reg;

  idex_reg = gen_control(ifid_reg.instr);                // Get control signals and register numbers
  idex_reg.imm = gen_imm(ifid_reg.instr);                // Get immediate values

  idex_reg.rs1val = regfile_p->R[idex_reg.rs1reg];       // Convert register number to register values
  idex_reg.rs2val = regfile_p->R[idex_reg.rs2reg];
  idex_reg.rdval = regfile_p->R[idex_reg.rdreg];

  idex_reg.instructionBits = ifid_reg.instructionBits;   // Transfer data
  idex_reg.instrAddr = ifid_reg.instrAddr;

#ifdef DEBUG_CYCLE
  printf("[ID ]: Instruction [%08x]@[%08x]: ", idex_reg.instr.bits, ifid_reg.instrAddr);
  decode_instruction(idex_reg.instr.bits);
#endif

  return idex_reg;
}


/**
 * STAGE  : stage_execute
 * output : exmem_reg_t
 **/
exmem_reg_t stage_execute(idex_reg_t idex_reg, pipeline_wires_t *pwires_p)
{
  exmem_reg_t exmem_reg = {0}; // Initialize the EX/MEM pipeline register with default (zeroed) values

  // Generate ALU control signals based on instruction type and opcode/funct3/funct7
  exmem_reg.control = gen_alu_control(idex_reg);

  // Choose ALU operand source: either immediate value or register rs2
  if(idex_reg.alu_src == 1) { 
      // ALU performs operation using rs1 and the immediate (e.g., for I-type instructions)
      exmem_reg.result = execute_alu(idex_reg.rs1val, idex_reg.imm, exmem_reg.control);
  }
  else if(idex_reg.alu_src == 0) {
      // ALU performs operation using rs1 and rs2 (e.g., for R-type instructions)
      exmem_reg.result = execute_alu(idex_reg.rs1val, idex_reg.rs2val, exmem_reg.control);
  }

  // Handle conditional branches
  if(idex_reg.branch){
    // Check branch condition result (e.g., beq, bne, etc.)
    if (gen_branch(idex_reg, pwires_p)){
      // If condition passes, signal branch to PC mux
      pwires_p->pcsrc = 1;
      // Compute new target PC address using current instruction address and immediate
      pwires_p->pc_src1 = idex_reg.instrAddr + idex_reg.imm;
    }
  }

  // Pass the instruction bits to the next pipeline stage
  exmem_reg.instr.bits = idex_reg.instr.bits;

  // Pass the instruction's address to the next stage
  exmem_reg.instrAddr = idex_reg.instrAddr;

  // Pass register values used in this stage (may be needed by MEM or WB)
  exmem_reg.rdval = idex_reg.rdval;     // Value in destination register (not yet written)
  exmem_reg.rs1val = idex_reg.rs1val;   // Source register 1 value
  exmem_reg.rs2val = idex_reg.rs2val;   // Source register 2 value

  // Pass the destination register index to MEM stage
  exmem_reg.rdreg = idex_reg.rdreg;

  // Forward memory access control signals
  exmem_reg.mem_read = idex_reg.mem_read;     // Whether we need to read from memory
  exmem_reg.mem_write = idex_reg.mem_write;   // Whether we need to write to memory
  exmem_reg.reg_write = idex_reg.reg_write;   // Whether we will write to a register in WB stage

#ifdef DEBUG_CYCLE
  // Debug print for this cycle's instruction in EX stage
  printf("[EX ]: Instruction [%08x]@[%08x]: ", idex_reg.instr.bits, idex_reg.instrAddr);
  decode_instruction(idex_reg.instr.bits); // Human-readable decode of instruction
#endif

  return exmem_reg; // Return the populated EX/MEM pipeline register
}

/**
 * STAGE  : stage_mem
 * output : memwb_reg_t
 **/ 
memwb_reg_t stage_mem(exmem_reg_t exmem_reg, pipeline_wires_t *pwires_p, Byte *memory_p, Cache *cache_p)
{
  memwb_reg_t memwb_reg = {0};               // Initialize the MEM/WB pipeline register to zero
  uint32_t address = 0;                      // Address for memory operation (load/store)

  // Handle store instructions (memory write)
  if (exmem_reg.mem_write == 1) {
    // Calculate memory address using base register + offset
    address = exmem_reg.rs1val + get_store_offset(exmem_reg.instr);

    // Choose data size and perform memory write based on funct3
    if (exmem_reg.instr.stype.funct3 == 0x0) {
      store(memory_p, address, LENGTH_BYTE, exmem_reg.rs2val);        // Store byte
    } else if (exmem_reg.instr.stype.funct3 == 0x1) {
      store(memory_p, address, LENGTH_HALF_WORD, exmem_reg.rs2val);   // Store half-word
    } else if (exmem_reg.instr.stype.funct3 == 0x2) {
      store(memory_p, address, LENGTH_WORD, exmem_reg.rs2val);        // Store word
    }
  }

  // Handle load instructions (memory read)
  else if (exmem_reg.mem_read == 1) {
    // Compute effective address using sign-extended immediate
    address = exmem_reg.rs1val + sign_extend_number(exmem_reg.instr.itype.imm, 12);

    // Select size and perform memory load based on funct3
    if (exmem_reg.instr.itype.funct3 == 0x0) {
      exmem_reg.result = sign_extend_number(load(memory_p, address, LENGTH_BYTE), 8); // Load byte
    } else if (exmem_reg.instr.itype.funct3 == 0x1) {
      exmem_reg.result = sign_extend_number(load(memory_p, address, LENGTH_HALF_WORD), 16); // Load half-word
    } else if (exmem_reg.instr.itype.funct3 == 0x2) {
      exmem_reg.result = load(memory_p, address, LENGTH_WORD); // Load word (no extension needed)
    }
  }

  // Pass ALU or loaded result to the next stage
  memwb_reg.result = exmem_reg.result;

  // Pass instruction data to MEM/WB stage
  memwb_reg.instr.bits = exmem_reg.instr.bits;
  memwb_reg.instrAddr = exmem_reg.instrAddr;

  // Forward destination register index
  memwb_reg.rdreg = exmem_reg.rdreg;

  // Forward register write enable signal
  memwb_reg.reg_write = exmem_reg.reg_write;

#ifdef DEBUG_CYCLE
  // Print debug info about the current instruction in MEM stage
  printf("[MEM]: Instruction [%08x]@[%08x]: ", exmem_reg.instr.bits, exmem_reg.instrAddr);
  decode_instruction(exmem_reg.instr.bits);
#endif

  // If instruction is a memory access (load or store)
  if (exmem_reg.instr.opcode == 0x03 || exmem_reg.instr.opcode == 0x23) {

    #ifndef CACHE_ENABLE
      // Without cache: account for fixed memory latency
      total_cycle_counter += (MEM_LATENCY == 0) ? 0 : MEM_LATENCY - 1;
      mem_access_counter++;   // Increment memory access count
    #endif

    #ifdef CACHE_ENABLE
      // If cache is enabled, simulate cache operation
      int res = processCacheOperation(address, cache_p); // Get latency result
      total_cycle_counter += res - 1;                    // Add latency minus one (since cycle already counted)

      // Update global cache stats
      miss_count = cache_p->miss_count;
      hit_count = cache_p->hit_count;

      #ifdef PRINT_CACHE_TRACES
        // Debug print for cache latency at the given address
        printf("[MEM]: Cache latency at addr: 0x%08x: %d cycles\n", address, res);
      #endif

    #endif
  }

  return memwb_reg; // Return the completed MEM/WB pipeline register
}

/**
 * STAGE  : stage_writeback
 * output : nothing - The state of the register file may be changed
 **/ 
void stage_writeback(memwb_reg_t memwb_reg, pipeline_wires_t *pwires_p, regfile_t *regfile_p)  // Write-back stage of the pipeline
{
  if(memwb_reg.reg_write == 1){                                              // Check if the register write control signal is enabled
    if (memwb_reg.rdreg != 0){                                               // Ensure we are not writing to register x0
      regfile_p->R[memwb_reg.rdreg] = memwb_reg.result;                     // Write the result to the destination register
    }
    if (memwb_reg.rdreg == 0){                                               // Special case: destination register is x0
      regfile_p->R[memwb_reg.rdreg] = 0x0;                                  // Enforce that x0 is always 0
    }
  }
  
#ifdef DEBUG_CYCLE
  printf("[WB ]: Instruction [%08x]@[%08x]: ", memwb_reg.instr.bits, memwb_reg.instrAddr); // Print instruction info for debugging
  decode_instruction(memwb_reg.instr.bits);                                   // Decode and print the instruction being written back
#endif
}

///////////////////////////////////////////////////////////////////////////////

/**
 * excite the pipeline with one clock cycle
 **/
void cycle_pipeline(regfile_t *regfile_p, Byte *memory_p, Cache *cache_p, pipeline_regs_t *pregs_p, pipeline_wires_t *pwires_p, bool *ecall_exit)
{
#ifdef DEBUG_CYCLE
  printf("v==============");                                                          // Print cycle start marker
  printf("Cycle Counter = %5ld", total_cycle_counter);                               // Show current cycle number
  printf("==============v\n\n");                                                     // Print cycle end marker
#endif

#ifdef PRINT_STATS
  if (pwires_p->fixcontrol)                                                          // If control hazard fix is needed
  {
    pwires_p->fixedcontrol = true;                                                   // Indicate control is fixed
    pwires_p->fixcontrol = false;                                                    // Reset control fix flag
  }
#endif

  pregs_p->ifid_preg.inp = stage_fetch(pwires_p, regfile_p, memory_p);              // Instruction Fetch stage

#ifdef PRINT_STATS
  detect_hazard(pregs_p, pwires_p, regfile_p, 1);                                    // Detect load-use hazard
#endif

  pregs_p->idex_preg.inp = stage_decode(pregs_p->ifid_preg.out, pwires_p, regfile_p); // Instruction Decode stage

#ifdef PRINT_STATS
  detect_hazard(pregs_p, pwires_p, regfile_p, 0);                                    // Detect forwarding/control hazards
#endif

  pregs_p->exmem_preg.inp = stage_execute(pregs_p->idex_preg.out, pwires_p);        // Execute stage

  pregs_p->memwb_preg.inp = stage_mem(pregs_p->exmem_preg.out, pwires_p, memory_p, cache_p); // Memory stage

  stage_writeback(pregs_p->memwb_preg.out, pwires_p, regfile_p);                    // Writeback stage

  // Update pipeline registers for next cycle
  pregs_p->ifid_preg.out = pregs_p->ifid_preg.inp;                                   // Update IF/ID pipeline register
  pregs_p->idex_preg.out = pregs_p->idex_preg.inp;                                   // Update ID/EX pipeline register
  pregs_p->exmem_preg.out = pregs_p->exmem_preg.inp;                                 // Update EX/MEM pipeline register
  pregs_p->memwb_preg.out = pregs_p->memwb_preg.inp;                                 // Update MEM/WB pipeline register

#ifdef PRINT_STATS
  if (pwires_p->fixedcontrol){                                                      // If control fix was applied
    #ifdef DEBUG_CYCLE
      printf("[CPL]: Pipeline Flushed\n");                                          // Debug print for flushing
    #endif    

    // Flush values from pipeline registers
    pregs_p->idex_preg.out.rdreg = 0;                                                // Clear destination register
    pregs_p->exmem_preg.out.rdreg = 0;
    pregs_p->memwb_preg.out.rdreg = 0;
    pregs_p->idex_preg.out.rs1reg = 0;                                               // Clear source registers
    pregs_p->exmem_preg.out.rs2reg = 0;
    pregs_p->ifid_preg.out.instr.bits = 0x00000013;                                  // Insert NOP (ADDI x0, x0, 0)
    pregs_p->idex_preg.out.instr.bits = 0x00000013;
    pregs_p->exmem_preg.out.instr.bits = 0x00000013;

    pwires_p->pcsrc = false;                                                         // Reset branch signal
    pwires_p->fixcontrol = false;                                                    // Clear fixcontrol
    pwires_p->fixedcontrol = false;                                                  // Clear fixedcontrol
  }
#endif

  ///////////////// NO CHANGES BELOW THIS ARE REQUIRED //////////////////////

  total_cycle_counter++;                                                             // Increment total cycle count

#ifdef DEBUG_REG_TRACE
  print_register_trace(regfile_p);                                                   // Print register trace for debugging
#endif

  /**
   * check ecall condition
   * To do this, the value stored in R[10] (a0 or x10) should be 10.
   * Hence, the ecall condition is checked by the existence of following
   * two instructions in sequence:
   * 1. <instr>  x10, <val1>, <val2>
   * 2. ecall
   *
   * The first instruction must write the value 10 to x10.
   * The second instruction is the ecall (opcode: 0x73)
   *
   * The condition checks whether the R[10] value is 10 when the
   * `memwb_reg.instr.opcode` == 0x73 (to propagate the ecall)
   *
   * If more functionality on ecall needs to be added, it can be done
   * by adding more conditions on the value of R[10]
   */
  
  // Check ECALL exit condition
  // If ECALL instruction (opcode 0x73) is in WB and x10 == 10, then exit
  if ((pregs_p->memwb_preg.out.instr.bits == 0x00000073) &&
      (regfile_p->R[10] == 10))
  {
    *(ecall_exit) = true;                                                            // Signal program termination
  }
}
