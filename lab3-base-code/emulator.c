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

void execute_instruction(uint32_t instruction_bits, Processor *processor,Byte *memory) {    
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
        /* YOUR CODE HERE */
	/* deal with other cases */
     case 0x1: // SLL, MULH
            switch (instruction.rtype.funct7) {
                case 0x00: // SLL
                    processor->R[instruction.rtype.rd] =
                        processor->R[instruction.rtype.rs1] <<
                        (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                case 0x01: // MULH
                    processor->R[instruction.rtype.rd] =
                        // Implement MULH (high part of signed multiplication)
                        ((int64_t)((sWord)processor->R[instruction.rtype.rs1]) *
                         (int64_t)((sWord)processor->R[instruction.rtype.rs2])) >> 32;
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        case 0x2: // SLT
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
        case 0x3: // SLTU
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
                case 0x00: // SRL
                    processor->R[instruction.rtype.rd] =
                        processor->R[instruction.rtype.rs1] >>
                        (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                case 0x20: // SRA
                    processor->R[instruction.rtype.rd] =
                        ((sWord)processor->R[instruction.rtype.rs1]) >>
                        (processor->R[instruction.rtype.rs2] & 0x1F);
                    break;
                case 0x01: // DIVU
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
                case 0x01: // REM
                    processor->R[instruction.rtype.rd] =
                        (sWord)processor->R[instruction.rtype.rs1] %
                        (sWord)processor->R[instruction.rtype.rs2];
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
            }
            break;
        case 0x7: // AND, REMU
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
    processor->PC += 4;
}

void execute_itype_except_load(Instruction instruction, Processor *processor) { //opcode 0x13
    switch (instruction.itype.funct3) {
        /* YOUR CODE HERE */
        case 0x0:
        //addi
            processor->R[instruction.itype.rd] = 
            ((sWord)processor->R[instruction.itype.rs1]) + 
            (sWord)sign_extend_number(instruction.itype.imm, 12);
            break;
        case 0x1:
        //slli
            switch (instruction.itype.imm >> 5){
                case 0x0:
                    processor->R[instruction.itype.rd] =
                    ((Word)processor->R[instruction.itype.rs1]) << 
                    ((Word)instruction.itype.imm & 0x1F);
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
                    break;
            }
            break;
        case 0x2:
        //slti
            processor->R[instruction.itype.rd] = 
            ((sWord)processor->R[instruction.itype.rs1]) < 
            ((sWord)sign_extend_number(instruction.itype.imm, 12)) ? 1 : 0;
            break;
        case 0x4:
        //xori
            processor->R[instruction.itype.rd] = ((Word)processor->R[instruction.itype.rs1]) ^ ((Word)sign_extend_number(instruction.itype.imm, 12));
            break;
        case 0x5:
        //srli and srai
            switch ((instruction.itype.imm >> 5) & 0x7F ){
                case 0x0:
                    processor->R[instruction.itype.rd] = ((Word)processor->R[instruction.itype.rs1]) >> ((Word)instruction.itype.imm & 0x1F);
                    break;
                case 0x20:
                    processor->R[instruction.itype.rd] = ((sWord)processor->R[instruction.itype.rs1] >> (instruction.itype.imm & 0x1F));
                    break;
                default:
                    handle_invalid_instruction(instruction);
                    exit(-1);
                    break;
            }
            break;
        case 0x6:
        //ori
            processor->R[instruction.itype.rd] = ((Word)processor->R[instruction.itype.rs1]) | (Word)sign_extend_number(instruction.itype.imm, 12);
            break;
        case 0x7:
        //andi
            processor->R[instruction.itype.rd] = ((Word)processor->R[instruction.itype.rs1]) & (Word)sign_extend_number(instruction.itype.imm, 12);
            break;
        default:
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
    //update PC
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
    switch (instruction.sbtype.funct3) {
        /* YOUR CODE HERE */
        case 0x0:
        //branch == beq
       if ((sWord)processor->R[instruction.sbtype.rs1] == (sWord)processor->R[instruction.sbtype.rs2]) 
                processor->PC += get_branch_offset(instruction);
        break;
        case 0x1:
        //branch != bne
        if (((sWord)processor->R[instruction.sbtype.rs1])!= ((sWord)processor->R[instruction.sbtype.rs2]))
            processor -> PC += get_branch_offset(instruction);
        break;
        default:
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
    processor->PC += 4; // Update PC after branch execution
}

void execute_load(Instruction instruction, Processor *processor, Byte *memory) {
    switch (instruction.itype.funct3) {
        /* YOUR CODE HERE */
        case 0x0:
        //Load Byte
        processor->R[instruction.itype.rd] =
             (sByte)load(memory, instruction.itype.rs1, LENGTH_BYTE);
        break;
        case 0x1:
        //Load Half
        processor->R[instruction.itype.rd]=
            (sHalf)load(memory,instruction.itype.rs1, LENGTH_HALF_WORD);
        break;
        case 0x2:
        //Load Word
        processor ->R[instruction.itype.rd]=
            (sWord)load(memory,instruction.itype.rs1, LENGTH_WORD);
        break;
        case 0x4:
        //Load Byte(U)
        processor -> R[instruction.itype.rd]=
            (Byte)load(memory, instruction.itype.rs1, LENGTH_BYTE);
        break;
        case 0x5:
        //Load Half(U)
        processor->R[instruction.itype.rd]=
            (Half)load(memory,instruction.itype.rs1, LENGTH_HALF_WORD);
        break;
        default:
            handle_invalid_instruction(instruction);
            break;
    }
    processor->PC+=4;
}

void execute_store(Instruction instruction, Processor *processor, Byte *memory) {
    switch (instruction.stype.funct3) {
        /* YOUR CODE HERE */
        case 0x0:
        //Store Byte
        store(memory,instruction.stype.rs1,LENGTH_BYTE,((sByte)processor->R[instruction.stype.rs2]));
        break;
        case 0x1:
        //Store Half
        store(memory,instruction.stype.rs1,LENGTH_HALF_WORD,((sHalf)processor->R[instruction.stype.rs2]));
        break;
        case 0x2:
        //Store Word
        store(memory,instruction.stype.rs1,LENGTH_WORD,((sByte)processor->R[instruction.stype.rs2]));
        break;

        default:
            handle_invalid_instruction(instruction);
            exit(-1);
            break;
    }
}
void execute_jal(Instruction instruction, Processor *processor) {
    /* YOUR CODE HERE */
    int offset = get_jump_offset(instruction);
    processor->R[instruction.ujtype.rd] = processor->PC + 4; 
    processor->PC += offset; 
}
void execute_lui(Instruction instruction, Processor *processor) {
    /* YOUR CODE HERE */
    int offset=get_store_offset(instruction);
    processor->R[instruction.utype.rd]= offset<<12;
}

void store(Byte *memory, Address address, Alignment alignment, Word value) {
    /* YOUR CODE HERE */
   if (alignment == LENGTH_BYTE) {
        memory[address] = value & 0xFF; 
    } else if (alignment == LENGTH_HALF_WORD) {
        memory[address] = value & 0xFF;
        memory[address + 1] = (value >> 8) & 0xFF;
    } else if (alignment == LENGTH_WORD) {
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
