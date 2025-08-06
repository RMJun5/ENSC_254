#include <stdbool.h>
#include "cache.h"
#include "riscv.h"
#include "types.h"
#include "utils.h"
#include "pipeline.h"
#include "stage_helpers.h"
#include <stdint.h>

uint64_t total_cycle_counter = 0;
uint64_t miss_count = 0;
uint64_t hit_count = 0;
uint64_t stall_counter = 0;
uint64_t branch_counter = 0;
uint64_t fwd_exex_counter = 0;
uint64_t fwd_exmem_counter = 0;
uint64_t mem_access_counter = 0;

simulator_config_t sim_config = {0};

///////////////////////////////////////////////////////////////////////////////

void bootstrap(pipeline_wires_t* pwires_p, pipeline_regs_t* pregs_p, regfile_t* regfile_p)
{
  pwires_p->pc_src0 = regfile_p->PC;
}

///////////////////////////
/// STAGE FUNCTIONALITY ///
///////////////////////////

/**
 * STAGE  : stage_fetch
 * output : ifid_reg_t
 **/ 
ifid_reg_t stage_fetch(pipeline_wires_t* pwires_p, regfile_t* regfile_p, Byte* memory_p)
{
    ifid_reg_t ifid_reg = {0};
    /**
     * YOUR CODE HERE
     */
    
    uint32_t instruction_bits = 0;

    if (regfile_p->PC < MEMORY_SPACE - 3) {  // Check we're not at end of memory
        instruction_bits = (memory_p[regfile_p->PC + 3] << 24) |
                          (memory_p[regfile_p->PC + 2] << 16) |
                          (memory_p[regfile_p->PC + 1] << 8)  |
                          memory_p[regfile_p->PC];
    }
    
    // If we got all zeros or are beyond program, inject NOP
    if (instruction_bits == 0) {
        instruction_bits = 0x00000013; // ADDI x0, x0, 0 (NOP)
    }
    
    ifid_reg.instr = parse_instruction(instruction_bits);
    ifid_reg.instr_addr = regfile_p->PC;
    
    if (!pwires_p->stall_pc) {
        regfile_p->PC += 4;
    }
    
    #ifdef DEBUG_CYCLE
    printf("[IF ]: Instruction [%08x]@[%08x]: ", instruction_bits, ifid_reg.instr_addr);
    decode_instruction(instruction_bits);
    #endif
    
    return ifid_reg;
}

/**
 * STAGE  : stage_decode
 * output : idex_reg_t
 **/ 
idex_reg_t stage_decode(ifid_reg_t ifid_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
    idex_reg_t idex_reg = {0};
    
    // Copy instr and PC
    Instruction instr = ifid_reg.instr;
    idex_reg.instr = instr;
    idex_reg.instr_addr = ifid_reg.instr_addr;

    pwires_p->stall_pc = false;
    pwires_p->stall_ifid = false;

    // Extract source registers based on opcode type
    uint32_t rs1 = 0, rs2 = 0;
    switch (instr.opcode) {
        case 0x33: // R-type
            rs1 = instr.rtype.rs1;
            rs2 = instr.rtype.rs2;
            break;
        case 0x13: // I-type
        case 0x03: // Load
            rs1 = instr.itype.rs1;
            break;
        case 0x23: // S-type
            rs1 = instr.stype.rs1;
            rs2 = instr.stype.rs2;
            break;
        case 0x63: // B-type
            rs1 = instr.sbtype.rs1;
            rs2 = instr.sbtype.rs2;
            break;
        case 0x37: // LUI
        case 0x17: // AUIPC
            break;
        default:
            break;
    }
    // Only check for hazard if previous instruction is a load
    if (pwires_p->prev_is_load && (
            (rs1 != 0 && rs1 == pwires_p->prev_rd) || 
            (rs2 != 0 && rs2 == pwires_p->prev_rd))) {
        pwires_p->stall_pc = true;
        pwires_p->stall_ifid = true;
        stall_counter++;
    }

    // Read register file values (default to zero if no source needed)
    idex_reg.rs1_val = (rs1 != 0) ? regfile_p->R[rs1] : 0;
    idex_reg.rs2_val = (rs2 != 0) ? regfile_p->R[rs2] : 0;

    // Generate immediate value
    idex_reg.imm = gen_imm(instr);

    if (instr.opcode == 0x33) {
        idex_reg.rd = instr.rtype.rd;
    } else if (instr.opcode == 0x13 || instr.opcode == 0x03) {
        idex_reg.rd = instr.itype.rd;
    } else if (instr.opcode == 0x37 || instr.opcode == 0x17) {
        idex_reg.rd = instr.utype.rd;
    } else {
        idex_reg.rd = 0;
    }

    // Generate control signals
    switch (instr.opcode) {
        case 0x33: // R-type
        case 0x13: // I-type
        case 0x03: // Load
        case 0x37: // LUI
        case 0x17: // AUIPC
            idex_reg.reg_write = true;
            break;
        default:
            idex_reg.reg_write = false;
    }
    idex_reg.mem_read = (instr.opcode == 0x03);  // Load
    idex_reg.mem_write = (instr.opcode == 0x23); // Store
    idex_reg.mem_to_reg = (instr.opcode == 0x03);  // True for loads, false elsewhere
    idex_reg.alu_src = (instr.opcode == 0x13 || instr.opcode == 0x03 || instr.opcode == 0x37 || instr.opcode == 0x17);
    idex_reg.alu_control = gen_alu_control(instr);

#ifdef DEBUG_CYCLE
    uint32_t instr_bits = idex_reg.instr.bits;
    printf("[ID ]: Instruction [%08x]@[%08x]: ", instr_bits, idex_reg.instr_addr);
    decode_instruction(instr_bits);
#endif

    return idex_reg;
}

/**
 * STAGE  : stage_execute
 * output : exmem_reg_t
 **/
exmem_reg_t stage_execute(idex_reg_t idex_reg, pipeline_wires_t* pwires_p)
{
    exmem_reg_t exmem_reg = {0};

    exmem_reg.instr = idex_reg.instr;
    exmem_reg.instr_addr = idex_reg.instr_addr;
    exmem_reg.rd = idex_reg.rd;

    uint32_t operand1 = idex_reg.rs1_val;
    uint32_t operand2 = idex_reg.alu_src ? idex_reg.imm : idex_reg.rs2_val;
    exmem_reg.alu_result = execute_alu(operand1, operand2, idex_reg.alu_control);
    exmem_reg.rs2_val = idex_reg.rs2_val;
    exmem_reg.reg_write = idex_reg.reg_write;
    exmem_reg.mem_write = idex_reg.mem_write;
    exmem_reg.mem_read = idex_reg.mem_read;
    exmem_reg.mem_to_reg = idex_reg.mem_to_reg;


#ifdef DEBUG_CYCLE
    uint32_t instr_bits = idex_reg.instr.bits;
    printf("[EX ]: Instruction [%08x]@[%08x]: ", instr_bits, idex_reg.instr_addr);
    decode_instruction(instr_bits);
#endif

    return exmem_reg;
}

/**
 * STAGE  : stage_mem
 * output : memwb_reg_t
 **/ 
memwb_reg_t stage_mem(exmem_reg_t exmem_reg, pipeline_wires_t* pwires_p, Byte* memory_p, Cache* cache_p)
{
    memwb_reg_t memwb_reg = {0};

    memwb_reg.instr = exmem_reg.instr;
    memwb_reg.instr_addr = exmem_reg.instr_addr;
    memwb_reg.alu_result = exmem_reg.alu_result;
    memwb_reg.rd = exmem_reg.rd;
    memwb_reg.reg_write = exmem_reg.reg_write;
    memwb_reg.mem_to_reg = exmem_reg.mem_to_reg;

#ifdef DEBUG_CYCLE
    uint32_t instr_bits = exmem_reg.instr.bits;
    printf("[MEM]: Instruction [%08x]@[%08x]: ", instr_bits, exmem_reg.instr_addr);
    decode_instruction(instr_bits);
#endif

    return memwb_reg;
}

/**
 * STAGE  : stage_writeback
 * output : nothing - The state of the register file may be changed
 **/ 
void stage_writeback(memwb_reg_t memwb_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
#ifdef DEBUG_CYCLE
    uint32_t instr_bits = memwb_reg.instr.bits;
    printf("[WB ]: Instruction [%08x]@[%08x]: ", instr_bits, memwb_reg.instr_addr);
    decode_instruction(instr_bits);
#endif

    if (memwb_reg.reg_write ) {
        regfile_p->R[memwb_reg.rd] = memwb_reg.mem_to_reg 
                                    ? memwb_reg.mem_data 
                                    : memwb_reg.alu_result;
        regfile_p->R[0]==0;
    }
}

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

/** 
 * excite the pipeline with one clock cycle
 **/
void cycle_pipeline(regfile_t* regfile_p, Byte* memory_p, Cache* cache_p, pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p, bool* ecall_exit)
{
  #ifdef DEBUG_CYCLE
  printf("v==============");
  printf("Cycle Counter = %5ld", total_cycle_counter);
  printf("==============v\n\n");
  #endif

  // process each stage

  /* Output               |    Stage      |       Inputs  */
  pregs_p->ifid_preg.inp  = stage_fetch     (pwires_p, regfile_p, memory_p);

  pregs_p->idex_preg.inp  = stage_decode    (pregs_p->ifid_preg.out, pwires_p, regfile_p);

  pregs_p->exmem_preg.inp = stage_execute   (pregs_p->idex_preg.out, pwires_p);

  pregs_p->memwb_preg.inp = stage_mem       (pregs_p->exmem_preg.out, pwires_p, memory_p, cache_p);

                            stage_writeback (pregs_p->memwb_preg.out, pwires_p, regfile_p);

  // update all the output registers for the next cycle from the input registers in the current cycle
  pregs_p->ifid_preg.out  = pregs_p->ifid_preg.inp;
  pregs_p->idex_preg.out  = pregs_p->idex_preg.inp;
  pregs_p->exmem_preg.out = pregs_p->exmem_preg.inp;
  pregs_p->memwb_preg.out = pregs_p->memwb_preg.inp;

  /////////////////// NO CHANGES BELOW THIS ARE REQUIRED //////////////////////

  // increment the cycle
  total_cycle_counter++;

  #ifdef DEBUG_REG_TRACE
  print_register_trace(regfile_p);
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
   if( (pregs_p->memwb_preg.out.instr.bits == 0x00000073) &&
       (regfile_p->R[10] == 10) )
   {
      *(ecall_exit) = true;
   }
}