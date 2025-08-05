#include <stdbool.h>
#include "cache.h"
#include "riscv.h"
#include "types.h"
#include "utils.h"
#include "pipeline.h"
#include "stage_helpers.h"

uint64_t total_cycle_counter = 0;
uint64_t miss_count = 0;
uint64_t hit_count = 0;
uint64_t stall_counter = 0;
uint64_t branch_counter = 0;
uint64_t fwd_exex_counter = 0;
uint64_t fwd_exmem_counter = 0;

simulator_config_t sim_config = {0};

///////////////////////////////////////////////////////////////////////////////

void bootstrap(pipeline_wires_t* pwires_p, pipeline_regs_t* pregs_p, regfile_t* regfile_p)
{
  // PC src must get the same value as the default PC value
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
   uint32_t instruction_bits = load(memory_p, regfile_p->PC, LENGTH_WORD);
   
   regfile_p -> PC += 4;
   //update PC with a wire
   ifid_reg.PC = pwires_p->pc_src0;
   ifid_reg.instr_addr = instruction_bits;

  #ifdef DEBUG_CYCLE
  printf("[IF ]: Instruction [%08x]@[%08x]: ", instruction_bits, regfile_p->PC);
  decode_instruction(instruction_bits);
  #endif
  ifid_reg.instr_addr = regfile_p->PC;
  return ifid_reg; 
}

/**
 * STAGE  : stage_decode
 * output : idex_reg_t
 **/ 
idex_reg_t stage_decode(ifid_reg_t ifid_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
  idex_reg_t idex_reg = {0};
  /**
   * YOUR CODE HERE
   */
  // 1. Get instruction bits and decoded instruction
  //uint32_t instr_bits = ifid_reg.instruction_bits;
  Instruction instr = ifid_reg.instr;
  // 2. save to idex_reg

  uint8_t rs1 = instr.rtype.rs1;
  uint8_t rs2 = instr.rtype.rs2;
  

  //read register values
  switch (instr.opcode){
    case 0x33:
    //R-type - rs1,rs2,rd - no imm
    idex_reg.rs1_val= regfile_p->R[rs1];
    idex_reg.rs2_val =regfile_p->R[rs2];
    idex_reg.rd = instr.rtype.rd;
    //pipeline wires
    pwires_p->alu_op; /* ALU operation based on funct3/funct7 */
    pwires_p->reg_write = true;
    pwires_p->mem_read = false;
    pwires_p->mem_write = false;
    break;
    case 0x13:
    
    //I-type/load -rs1, rd - imm
    idex_reg.rs1_val= regfile_p->R[rs1];
    idex_reg.rd = instr.itype.rd;    
    //Generate immediate 
    idex_reg.imm = sign_extend_number(instr.itype.imm, 12);
    case 0x03:
      pwires_p->alu_op; /* ALU add for address calc */
      pwires_p->reg_write = true;
      pwires_p->mem_read = true;
      pwires_p->mem_to_reg = true;
    break;
    case 0x23:
    //Store
    idex_reg.rs1_val= regfile_p->R[rs1];
    idex_reg.rs2_val = regfile_p->R[rs2];
    idex_reg.imm5 = sign_extend_number(instr.stype.imm5,12);
    idex_reg.imm7 = sign_extend_number(instr.stype.imm7,12);
    break;
    case 0x63:
    //Branch
    int b_offset = get_branch_offset(instr);
    idex_reg.rs1_val=regfile_p->R[rs1];
    idex_reg.rs1_val=regfile_p->R[rs2];
    idex_reg.imm5 = sign_extend_number(instr.sbtype.imm5, b_offset);
    idex_reg.imm7 = sign_extend_number(instr.sbtype.imm7, b_offset);
    //pipeline wires: 
     pwires_p->branch_taken = idex_reg.condition  /* branch condition */;
    pwires_p->branch_target = idex_reg.target /* calculated target PC */;
    break;
    case 0x37:
    //load upper immediate
    idex_reg.imm = instr.utype.imm;
    break;
    case 0x6F:
    //Jump and link
    int j_offset=get_jump_offset(instr);
    idex_reg.imm = sign_extend_number(instr.ujtype.imm, j_offset);
    break;
    case 0x73:
    //ECAll
    print(ECALL_FORMAT);
    break;
    default:
    handle_invalid_instruction(instr);
    exit(-1);
    break;
  }
  
  return idex_reg;
}

/**
 * STAGE  : stage_execute
 * output : exmem_reg_t
 **/ 
exmem_reg_t stage_execute(idex_reg_t idex_reg, pipeline_wires_t* pwires_p)
{
  exmem_reg_t exmem_reg = {0};
  /**
   * YOUR CODE HERE
   */
  
  return exmem_reg;
}

/**
 * STAGE  : stage_mem
 * output : memwb_reg_t
 **/ 
memwb_reg_t stage_mem(exmem_reg_t exmem_reg, pipeline_wires_t* pwires_p, Byte* memory_p, Cache* cache_p)
{
  memwb_reg_t memwb_reg = {0};
  /**
   * YOUR CODE HERE
   */
  return memwb_reg;
}

/**
 * STAGE  : stage_writeback
 * output : nothing - The state of the register file may be changed
 **/ 
void stage_writeback(memwb_reg_t memwb_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
  /**
   * YOUR CODE HERE
   */
}

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

