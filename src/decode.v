//=============================================================================
// EE108B Lab 3
//
// Decode module. Determines what to do with an instruction.
//=============================================================================

`include "mips_defines.v"

module decode (
    input [31:0] pc,
    input [31:0] instr,
    input [31:0] rs_data_in,
    input [31:0] rt_data_in,
    input [31:0] alu_result_ex,
    input [31:0] reg_write_data_mem,
    input [4:0] reg_write_addr_ex,
    input [4:0] reg_write_addr_mem,
    input reg_we_ex,
    input reg_we_mem,
    input mem_read_ex,

    output wire [4:0] reg_write_addr,
    output wire jump_branch,
    output wire jump_target,
    output wire jump_reg,
    output wire [31:0] jr_pc,
    output reg [3:0] alu_opcode,
    output wire [31:0] alu_op_x,
    output wire [31:0] alu_op_y,
    output wire mem_we,
    output wire [31:0] mem_write_data,
    output wire mem_read,
    output wire reg_we,
    output wire [4:0] rs_addr,
    output wire [4:0] rt_addr,

    output wire stall//, // put more signal inputs here that will be used for
                         // forwarding and stalling
                         // (don't forget about the comma)
);

//******************************************************************************
// instruction field
//******************************************************************************

    wire [5:0] op = instr[31:26];
    assign rs_addr = instr[25:21];
    assign rt_addr = instr[20:16];
    wire [4:0] rd_addr = instr[15:11];
    wire [4:0] shamt = instr[10:6];
    wire [5:0] funct = instr[5:0];
    wire [15:0] immediate = instr[15:0];

    reg [31:0] rs_data, rt_data;

//******************************************************************************
// branch instructions decode
//******************************************************************************

    wire isBEQ  = (op == `BEQ);
    wire isBGEZ = (op == `BLTZ_GEZ) & ((rt_addr == `BGEZ) | (rt_addr == `BGEZAL));
    wire isBGTZ = (op == `BGTZ) & (rt_addr == 5'b00000);
    wire isBLEZ = (op == `BLEZ) & (rt_addr == 5'b00000);
    wire isBLTZ = (op == `BLTZ_GEZ) & ((rt_addr == `BLTZ) | (rt_addr == `BLTZAL));
    wire isBNE  = (op == `BNE);
    wire isBranchLink = (isBGEZ & (rt_addr == `BGEZAL)) | (isBLTZ & (rt_addr == `BLTZAL));
    

//******************************************************************************
// jump instructions decode
//******************************************************************************

    wire isJ    = (op == `J);
    wire isJAL  = (op == `JAL);
    wire isJALR = (op == `SPECIAL) & (funct == `JALR);  
    wire isJR   = (op == `SPECIAL) & (funct == `JR);
    
    // determine if the next pc will need to be stored
    wire isLink = isJALR | isJAL | isBranchLink;

//******************************************************************************
// shift instruction decode
//******************************************************************************

    wire isSLL = (op == `SPECIAL) & (funct == `SLL);
    wire isSRA = (op == `SPECIAL) & (funct == `SRA);
    wire isSRL = (op == `SPECIAL) & (funct == `SRL);
    wire isSLLV = (op == `SPECIAL) & (funct == `SLLV);
    wire isSRAV = (op == `SPECIAL) & (funct == `SRAV);
    wire isSRLV = (op == `SPECIAL) & (funct == `SRLV);
    
    wire isShiftImm = isSLL | isSRA | isSRL;
    wire isShift = isShiftImm | isSLLV | isSRAV | isSRLV;

//******************************************************************************
// ALU instructions decode / control signal for ALU datapath
//******************************************************************************
    
    always @* begin
        casex({op, funct})
            {`ADDI, `DC6}:      alu_opcode = `ALU_ADD;
            {`ADDIU, `DC6}:     alu_opcode = `ALU_ADDU;
            {`SLTI, `DC6}:      alu_opcode = `ALU_SLT;
            {`SLTIU, `DC6}:     alu_opcode = `ALU_SLTU;	
            {`ANDI, `DC6}:      alu_opcode = `ALU_AND;
            {`ORI, `DC6}:       alu_opcode = `ALU_OR;
            {`XORI, `DC6}:      alu_opcode = `ALU_XOR;
            {`LW, `DC6}:        alu_opcode = `ALU_ADD;
            {`SW, `DC6}:        alu_opcode = `ALU_ADD;
            {`BEQ, `DC6}:       alu_opcode = `ALU_SUBU;
            {`BNE, `DC6}:       alu_opcode = `ALU_SUBU;
            {`SPECIAL, `ADD}:   alu_opcode = `ALU_ADD;
            {`SPECIAL, `ADDU}:  alu_opcode = `ALU_ADDU;
            {`SPECIAL, `SUB}:   alu_opcode = `ALU_SUB;
            {`SPECIAL, `SUBU}:  alu_opcode = `ALU_SUBU;
            {`SPECIAL, `AND}:   alu_opcode = `ALU_AND;
            {`SPECIAL, `OR}:    alu_opcode = `ALU_OR;
            {`SPECIAL, `XOR}:   alu_opcode = `ALU_XOR;
            {`SPECIAL, `NOR}:   alu_opcode = `ALU_NOR;
            {`SPECIAL, `SLT}:   alu_opcode = `ALU_SLT;
            {`SPECIAL, `SLTU}:  alu_opcode = `ALU_SLTU;
            {`SPECIAL, `SLL}:   alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRL}:   alu_opcode = `ALU_SRL;
            {`SPECIAL, `SRA}:   alu_opcode = `ALU_SRA;
            {`SPECIAL, `SLLV}:  alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRLV}:  alu_opcode = `ALU_SRL;
            {`SPECIAL, `SRAV}:  alu_opcode = `ALU_SRA;
            // compare rs data to 0, only care about 1 operand
            {`BGTZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLEZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLTZ_GEZ, `DC6}: begin
                if (isBranchLink)
                    alu_opcode = `ALU_PASSY; // pass link address for mem stage
                else
                    alu_opcode = `ALU_PASSX;
            end
            // pass link address to be stored in $ra
            {`JAL, `DC6}:       alu_opcode = `ALU_PASSY;
            {`SPECIAL, `JALR}:  alu_opcode = `ALU_PASSY;
            // or immediate with 0
            {`LUI, `DC6}:       alu_opcode = `ALU_PASSY;
            default:            alu_opcode = `ALU_PASSX;
    	endcase
    end

//******************************************************************************
// Compute value for 32 bit immediate data
//******************************************************************************

    wire use_imm = &{op != `SPECIAL, op != `BNE, op != `BEQ}; // where to get 2nd ALU operand from: 0 for RtData, 1 for Immediate
    
    wire [31:0] imm_sign_extend = {{16{immediate[15]}}, immediate};  
    wire [31:0] imm_zero_extend = {16'b0, immediate};	
    wire [31:0] imm_upper = {immediate, 16'b0};
    
    reg [31:0] imm;
    always @* begin
        if (op == `LUI)
            imm = imm_upper;
        else if (|{op == `ORI, op == `ANDI, op == `XORI})
            imm = imm_zero_extend;
        else
            imm = imm_sign_extend;
    end

//******************************************************************************
// forwarding and stalling logic
//******************************************************************************

    // TODO: Set rs_data and rt_data so that forwarded data from the X and M
    // stages can be used where appropriate. rs_data_in and rt_data_in are
    // the values read from the register file

    //if EX/MEM.RegWrite && EX/MEM.RegRd == ID/EX.RegRs && EX/MEM.RegRd != $zero,
    //forward 

    wire EXForwardRs = (reg_we_ex && (reg_write_addr_ex != `ZERO) && (reg_write_addr_ex == rs_addr));
    wire EXForwardRt = (reg_we_ex && (reg_write_addr_ex != `ZERO) && (reg_write_addr_ex == rt_addr));
    wire MEMForwardRs = ( reg_we_mem && (reg_write_addr_mem != `ZERO) && !(reg_we_ex && (reg_write_addr_ex != `ZERO)
		&& (reg_write_addr_ex == rs_addr)) && (reg_write_addr_mem == rs_addr) );
    wire MEMForwardRt = ( reg_we_mem && (reg_write_addr_mem != `ZERO) && !(reg_we_ex && (reg_write_addr_ex != `ZERO)
		&& (reg_write_addr_ex == rt_addr)) && (reg_write_addr_mem == rt_addr) );


    always @(*) begin
	casex ({EXForwardRs, MEMForwardRs})
		2'b1x: rs_data = alu_result_ex;
		2'b01: rs_data = reg_write_data_mem;
		default: rs_data = rs_data_in;
	endcase

	casex ({EXForwardRt, MEMForwardRt})
		2'b1x: rt_data = alu_result_ex;
		2'b01: rt_data = reg_write_data_mem;
		default: rt_data = rt_data_in;
	endcase		
    end
    
    // TODO: Assert the stall signal when a hazard occurs that cannot be
    // resolved by forwarding.
    assign stall = mem_read_ex && ((reg_write_addr_ex == rs_addr) || (reg_write_addr_ex == rt_addr));

    // these signals will automatically be set to the forwarded versions
    // once you have implemented forwarding in general
    assign jr_pc = rs_data;
    assign mem_write_data = rt_data;

//******************************************************************************
// Determine ALU inputs and register writeback address
//******************************************************************************

    // for shift operations, use either shamt field or lower 5 bits of rs
    // otherwise use rs

    wire [31:0] shift_amount = isShiftImm ? shamt : rs_data[4:0];
    assign alu_op_x = isShift ? shift_amount : rs_data;
    		
    // for link operations, use next pc (current pc + 4)
    // for immediate operations, use Imm
    // otherwise use rt
    
    assign alu_op_y = isLink ? pc + 4'h8 : (use_imm ? imm : rt_data);
    assign reg_write_addr = isLink ? `RA : (use_imm ? rt_addr : rd_addr);
    
    // determine when to write back to a register (any operation that isn't a branch, jump, or store)
    assign reg_we = ~|{mem_we, isJ, isJR, isBGEZ, isBGTZ, isBLEZ, isBLTZ, isBNE, isBEQ};
  
//******************************************************************************
// Memory control
//******************************************************************************
    assign mem_we = (op == `SW);    // write to memory
    assign mem_read = (op == `LW);    // use memory data for writing to register

//******************************************************************************
// Branch resolution
//******************************************************************************
    
    wire isEqual = rs_data == rt_data;
    wire isZero = ~|rs_data;
    wire isNeg = rs_data[31];
    wire isPos = ~(isZero | isNeg);

    assign jump_branch = |{isBEQ & isEqual,
                           isBNE & ~isEqual,
                           isBGEZ & ~isNeg,
                           isBGTZ & isPos,
                           isBLEZ & ~isPos,
                           isBLTZ & isNeg};
    
    assign jump_target = isJ | isJAL;
    assign jump_reg = isJALR | isJR;

endmodule
