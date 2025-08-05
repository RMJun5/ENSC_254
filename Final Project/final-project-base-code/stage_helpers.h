#ifndef __STAGE_HELPERS_H__
#define __STAGE_HELPERS_H__

#include <stdio.h>
#include "utils.h"
#include "pipeline.h"

/// EXECUTE STAGE HELPERS ///

/**
 * input  : Instruction
 * output : uint32_t alu_control signal
 **/
uint32_t gen_alu_control(Instruction instr)
{
    uint32_t alu_control = 0;

    switch (instr.opcode) {
        case 0x33: // R-type 
            switch (instr.rtype.funct3) {
                case 0x0:
                    switch (instr.rtype.funct7) {
                        case 0x00: // add
                            alu_control = 0;
                            break;
                        case 0x01: // mul
                            alu_control = 1;
                            break;
                        case 0x20: // sub
                            alu_control = 2;
                            break;
                        default:
                            break;
                    }
                    break; 

                case 0x1:
                    switch (instr.rtype.funct7) {
                        case 0x00: // sll
                            alu_control = 3;
                            break;
                        case 0x01: // mulh
                            alu_control = 4;
                            break;
                        default:
                            break;
                    }
                    break; 

                case 0x2: // slt
                    alu_control = 5;
                    break;

                case 0x4:
                    switch (instr.rtype.funct7) {
                        case 0x00: // xor
                            alu_control = 6;
                            break;
                        case 0x01: // div
                            alu_control = 7;
                            break;
                        default:
                            break;
                    }
                    break;

                case 0x5:
                    switch (instr.rtype.funct7) {
                        case 0x00: // srl
                            alu_control = 8;
                            break;
                        case 0x20: // sra
                            alu_control = 9;
                            break;
                        default:
                            break;
                    }
                    break;

                case 0x6:
                    switch (instr.rtype.funct7) {
                        case 0x00: // or
                            alu_control = 10;
                            break;
                        case 0x01: // rem
                            alu_control = 11;
                            break;
                        default:
                            break;
                    }
                    break;

                case 0x7: // and
                    alu_control = 12;
                    break;

                default:
                    alu_control = 0;
                    break;
            }
            break; // end of R-type

        case 0x13: // I-type (except load)
            switch (instr.itype.funct3) {
                case 0x0: // addi
                    alu_control = 0;
                    break;
                case 0x1: // slli
                    alu_control = 3;
                    break;
                case 0x2: // slti
                    alu_control = 5;
                    break;
                case 0x4: // xori
                    alu_control = 6;
                    break;
                case 0x5: // srli / srai
                    if (((instr.itype.imm >> 5) & 0x7F) == 0x00) // srli
                        alu_control = 8;
                    else if (((instr.itype.imm >> 5) & 0x7F) == 0x20) // srai
                        alu_control = 9;
                    break;
                case 0x6: // ori
                    alu_control = 10;
                    break;
                case 0x7: // andi
                    alu_control = 12;
                    break;
                default:
                    break;
            }
            break;

        case 0x63: // B-type
            alu_control = 2; // SUB for comparisons
            break;

        case 0x03: // Load
        case 0x23: // Store
            alu_control = 0; // Use ADD for address calculation
            break;

        default:
            alu_control = 0; // Default ALU operation (ADD)
            break;
    }

    return alu_control;
}

/**
 * input  : alu_inp1, alu_inp2, alu_control
 * output : uint32_t alu_result
 **/
uint32_t execute_alu(uint32_t alu_inp1, uint32_t alu_inp2, uint32_t alu_control)
{
    uint32_t result;
    switch (alu_control) {
        case 0: // add
            result = alu_inp1 + alu_inp2;
            break;
        case 1: // mul
            result = alu_inp1 * alu_inp2;
            break;
        case 2: // sub
            result = alu_inp1 - alu_inp2;
            break;
        case 3: // sll
            result = alu_inp1 << (alu_inp2 & 0x1F);
            break;
        case 4: // mulh
            result = ((int64_t)sign_extend_number(alu_inp1, 32) * 
                      (int64_t)sign_extend_number(alu_inp2, 32)) >> 32;
            break;
        case 5: // slt
            result = (sign_extend_number(alu_inp1, 32) < 
                      sign_extend_number(alu_inp2, 32)) ? 1 : 0;
            break;
        case 6: // xor
            result = alu_inp1 ^ alu_inp2;
            break;
        case 7: // div
            if (alu_inp2 == 0) result = -1;
            else result = sign_extend_number(alu_inp1, 32) / 
                          sign_extend_number(alu_inp2, 32);
            break;
        case 8: // srl
            result = alu_inp1 >> (alu_inp2 & 0x1F);
            break;
        case 9: // sra
            result = sign_extend_number(alu_inp1, 32) >> (alu_inp2 & 0x1F);
            break;
        case 10: // or
            result = alu_inp1 | alu_inp2;
            break;
        case 11: // rem
            if (alu_inp2 == 0) result = alu_inp1;
            else result = sign_extend_number(alu_inp1, 32) % 
                          sign_extend_number(alu_inp2, 32);
            break;
        case 12: // and
            result = alu_inp1 & alu_inp2;
            break;
        default:
            result = 0xBADCAFFE;
            break;
    }
    return result;
}

/// DECODE STAGE HELPERS ///

/**
 * input  : Instruction
 * output : immediate value (int32_t)
 **/
int32_t gen_imm(Instruction instruction)
{
    int32_t imm_val = 0;
    switch (instruction.opcode) {
        case 0x63: // B-type
            imm_val = get_branch_offset(instruction);
            break;
        case 0x03: // Load (I-type)
        case 0x13: // I-type
        case 0x73: // SYSTEM (ECALL/EBREAK)
            imm_val = sign_extend_number(instruction.itype.imm, 12);
            break;
        case 0x23: // S-type
            imm_val = get_store_offset(instruction);
            break;
        case 0x37: // U-type (LUI)
            imm_val = instruction.utype.imm << 12;
            break;
        case 0x6F: // J-type (JAL)
            imm_val = get_jump_offset(instruction);
            break;
        default: // R-type and others have no immediate
            imm_val = 0;
            break;
    };
    return imm_val;
}

/**
 * generates all the control logic that flows around in the pipeline
 * input  : Instruction
 * output : idex_reg_t
 **/
idex_reg_t gen_control(Instruction instruction)
{
    idex_reg_t idex_reg = {0};

    switch (instruction.opcode) {
        case 0x33:  // R-type
            idex_reg.alu_src = 0;
            idex_reg.reg_write = 1;
            idex_reg.mem_read = 0;
            idex_reg.mem_write = 0;
            idex_reg.mem_to_reg = 0;
            idex_reg.alu_control = gen_alu_control(idex_reg.instr);
            break;

        case 0x13:  // I-type (immediate ALU ops)
            idex_reg.alu_src = 1;
            idex_reg.reg_write = 1;
            idex_reg.mem_read = 0;
            idex_reg.mem_write = 0;
            idex_reg.mem_to_reg = 0;
            idex_reg.alu_control = gen_alu_control(idex_reg.instr);
            break;

        case 0x03: // Load
            idex_reg.alu_src = 1;
            idex_reg.reg_write = 1;
            idex_reg.mem_read = 1;
            idex_reg.mem_write = 0;
            idex_reg.mem_to_reg = 1;
            idex_reg.alu_control = gen_alu_control(idex_reg.instr);
            break;

        case 0x23:  // Store (S-type)
            idex_reg.alu_src = 1;
            idex_reg.reg_write = 0;
            idex_reg.mem_read = 0;
            idex_reg.mem_write = 1;
            idex_reg.mem_to_reg = 0;
            idex_reg.alu_control = gen_alu_control(idex_reg.instr);
            break;

        case 0x63:  // Branch (B-type)
            idex_reg.alu_src = 0;
            idex_reg.reg_write = 0;
            idex_reg.mem_read = 0;
            idex_reg.mem_write = 0;
            idex_reg.mem_to_reg = 0;
            idex_reg.alu_control = gen_alu_control(idex_reg.instr);
            break;

        case 0x37:  // LUI (U-type)
            idex_reg.alu_src = 1;
            idex_reg.reg_write = 1;
            idex_reg.mem_read = 0;
            idex_reg.mem_write = 0;
            idex_reg.mem_to_reg = 0;
            idex_reg.alu_control = gen_alu_control(idex_reg.instr);
            break;

        case 0x6F:  // JAL (UJ-type)
            idex_reg.alu_src = 0;
            idex_reg.reg_write = 1;
            idex_reg.mem_read = 0;
            idex_reg.mem_write = 0;
            idex_reg.mem_to_reg = 0;
            idex_reg.alu_control = gen_alu_control(idex_reg.instr);
            break;

        default:  // Unknown opcode
            idex_reg.alu_src = 0;
            idex_reg.reg_write = 0;
            idex_reg.mem_read = 0;
            idex_reg.mem_write = 0;
            idex_reg.mem_to_reg = 0;
            idex_reg.alu_control = 0;
            break;
    }
    return idex_reg;
}

/// MEMORY STAGE HELPERS ///

/**
 * evaluates whether a branch must be taken
 * input  : rs1_val, rs2_val, funct3
 * output : bool
 **/
bool gen_branch(uint32_t rs1_val, uint32_t rs2_val, uint32_t funct3)
{
    // int32_t rs1Signed = sign_extend_number(rs1_val, 32);
    // int32_t rs2Signed = sign_extend_number(rs2_val, 32);

    switch (funct3) {
        case 0x0: // BEQ
            return (rs1_val == rs2_val);
        case 0x1: // BNE
            return (rs1_val != rs2_val);
        /*
        case 0x4: // BLT (signed)
            return (rs1Signed < rs2Signed);
        case 0x5: // BGE (signed)
            return (rs1Signed >= rs2Signed);
        case 0x6: // BLTU (unsigned)
            return (rs1_val < rs2_val);
        case 0x7: // BGEU (unsigned)
            return (rs1_val >= rs2_val);
        */
        default:
            return false; // Invalid funct3
    }
}

/// PIPELINE FEATURES ///

/**
 * Sets the forwarding unit control signals based on pipeline register values
 * input  : pipeline_regs_t*, pipeline_wires_t*
 * output : None
 **/
void gen_forward(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p)
{
    pwires_p->forward_1 = 0;
    pwires_p->forward_2 = 0;

    idex_reg_t* idex = &pregs_p->idex_preg.out; // instruction in EX stage
    exmem_reg_t* exmem = &pregs_p->exmem_preg.out; // MEM stage
    memwb_reg_t* memwb = &pregs_p->memwb_preg.out; // WB stage

    // Extract rs1 and rs2 from idex instruction
    Instruction instr = idex->instr;
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
        default:
            // other types don't use rs1 or rs2 for forwarding
            break;
    }

    // forward 1 detect (using local rs1)
    if (exmem->reg_write && exmem->rd != 0 && exmem->rd == rs1) {
      pwires_p->forward_1 = 1; // forward from EX/MEM
    }
    else if (memwb->reg_write && memwb->rd != 0 && memwb->rd == rs1) {
      pwires_p->forward_1 = 2; // forward from MEM/WB
    }

    // forward 2 detect (using local rs2)
    if (exmem->reg_write && exmem->rd != 0 && exmem->rd == rs2) {
      pwires_p->forward_2 = 1; // forward from EX/MEM
    }
    else if (memwb->reg_write && memwb->rd != 0 && memwb->rd == rs2) {
      pwires_p->forward_2 = 2; // forward from MEM/WB
    }

}

/**
 * Sets hazard detection signals based on pipeline register values
 * input  : pipeline_regs_t*, pipeline_wires_t*, regfile_t*
 * output : None
 **/
void detect_hazard(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
    pwires_p->hazard_detected = false;

    const idex_reg_t* idex = &pregs_p->idex_preg.out;
    const ifid_reg_t* ifid = &pregs_p->ifid_preg.out;

    // Only check for load-use hazards
    if (idex->mem_read && idex->rd != 0) {
        Instruction ifid_instr = ifid->instr;
        uint8_t rs1 = 0, rs2 = 0;

        // Extract rs1/rs2 based on instruction type
        switch (ifid_instr.opcode) {
            case 0x33: // R-type
                rs1 = ifid_instr.rtype.rs1;
                rs2 = ifid_instr.rtype.rs2;
                break;
            case 0x03: // Load
            case 0x13: // I-type
                rs1 = ifid_instr.itype.rs1;
                break;
            case 0x23: // S-type
                rs1 = ifid_instr.stype.rs1;
                rs2 = ifid_instr.stype.rs2;
                break;
            case 0x63: // B-type
                rs1 = ifid_instr.sbtype.rs1;
                rs2 = ifid_instr.sbtype.rs2;
                break;
            default:
                break;
        }

        if (idex->rd == rs1 || idex->rd == rs2) {
            pwires_p->hazard_detected = true;
            pwires_p->stall_pc = true;
            pwires_p->stall_ifid = true;
        }
    } else {
        pwires_p->stall_pc = false;
        pwires_p->stall_ifid = false;
    }
}
// Helper function to check if an instruction is a NOP (ADDI x0,x0,0)
bool is_nop_instruction(Instruction instr) {
    return instr.bits == 0x00000013; // Standard RISC-V NOP
}

// Helper function to check if the pipeline is empty (all NOPs)
bool is_pipeline_empty(const pipeline_regs_t* pregs_p) {
    return is_nop_instruction(pregs_p->ifid_preg.out.instr)
        && is_nop_instruction(pregs_p->idex_preg.out.instr)
        && is_nop_instruction(pregs_p->exmem_preg.out.instr)
        && is_nop_instruction(pregs_p->memwb_preg.out.instr);
}

///////////////////////////////////////////////////////////////////////////////

/// RESERVED FOR PRINTING REGISTER TRACE AFTER EACH CLOCK CYCLE ///
void print_register_trace(regfile_t* regfile_p)
{
    // Print 8 columns of 4 registers each = 32 registers total
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            printf("r%2d=%08x ", i * 4 + j, regfile_p->R[i * 4 + j]);
        }
        printf("\n");
    }
    printf("\n");
}

#endif // __STAGE_HELPERS_H__
