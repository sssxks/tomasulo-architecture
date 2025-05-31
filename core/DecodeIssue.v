`timescale 1ns / 1ps

`include "define.vh"

module DecodeDispatch(
	input[31:0] inst,

	output ALU_dispatch, mul_dispatch, div_dispatch, load_dispatch, store_dispatch,
	output ujump_dispatch, branch_dispatch, // ujump: unconditional jump, branch: conditional jump

	output[2:0] ImmSel,
	output[3:0] ALU_op,
	output[3:0] JUMP_op,
	output ALUSrcA,
	output ALUSrcB,
	output FU_regWrite
);
	// instruction field
	wire [6:0] funct7 = inst[31:25];
	wire [2:0] funct3 = inst[14:12];
	wire [6:0] opcode = inst[6:0];
	wire [4:0] rd = inst[11:7];
	wire [4:0] rs1 = inst[19:15];
	wire [4:0] rs2 = inst[24:20];

	// type specification
	wire Rop = opcode == 7'b0110011;
	wire Iop = opcode == 7'b0010011;
	wire Bop = opcode == 7'b1100011;
	wire Lop = opcode == 7'b0000011;
	wire Sop = opcode == 7'b0100011;

	wire funct7_0  = funct7 == 7'h0;
	wire funct7_1  = funct7 == 7'h1;
	wire funct7_32 = funct7 == 7'h20;

	wire funct3_0 = funct3 == 3'h0;
	wire funct3_1 = funct3 == 3'h1;
	wire funct3_2 = funct3 == 3'h2;
	wire funct3_3 = funct3 == 3'h3;
	wire funct3_4 = funct3 == 3'h4;
	wire funct3_5 = funct3 == 3'h5;
	wire funct3_6 = funct3 == 3'h6;
	wire funct3_7 = funct3 == 3'h7;

	wire ADD  = Rop & funct3_0 & funct7_0;
	wire SUB  = Rop & funct3_0 & funct7_32;
	wire SLL  = Rop & funct3_1 & funct7_0;
	wire SLT  = Rop & funct3_2 & funct7_0;
	wire SLTU = Rop & funct3_3 & funct7_0;
	wire XOR  = Rop & funct3_4 & funct7_0;
	wire SRL  = Rop & funct3_5 & funct7_0;
	wire SRA  = Rop & funct3_5 & funct7_32;
	wire OR   = Rop & funct3_6 & funct7_0;
	wire AND  = Rop & funct3_7 & funct7_0;

	wire MUL    = Rop & funct3_0 & funct7_1;
	wire MULH   = Rop & funct3_1 & funct7_1;
	wire MULHSU = Rop & funct3_2 & funct7_1;
	wire MULHU  = Rop & funct3_3 & funct7_1;
	wire DIV    = Rop & funct3_4 & funct7_1;
	wire DIVU   = Rop & funct3_5 & funct7_1;
	wire REM    = Rop & funct3_6 & funct7_1;
	wire REMU   = Rop & funct3_7 & funct7_1;

	wire ADDI  = Iop & funct3_0;
	wire SLTI  = Iop & funct3_2;
	wire SLTIU = Iop & funct3_3;
	wire XORI  = Iop & funct3_4;
	wire ORI   = Iop & funct3_6;
	wire ANDI  = Iop & funct3_7;
	wire SLLI  = Iop & funct3_1 & funct7_0;
	wire SRLI  = Iop & funct3_5 & funct7_0;
	wire SRAI  = Iop & funct3_5 & funct7_32;

	wire BEQ = Bop & funct3_0;
	wire BNE = Bop & funct3_1;
	wire BLT = Bop & funct3_4;
	wire BGE = Bop & funct3_5;
	wire BLTU = Bop & funct3_6;
	wire BGEU = Bop & funct3_7;

	wire LB =  Lop & funct3_0;
	wire LH =  Lop & funct3_1;
	wire LW =  Lop & funct3_2;
	wire LBU = Lop & funct3_4;
	wire LHU = Lop & funct3_5;

	wire SB = Sop & funct3_0;
	wire SH = Sop & funct3_1;
	wire SW = Sop & funct3_2;

	wire LUI   = opcode == 7'b0110111;
	wire AUIPC = opcode == 7'b0010111;

	wire JAL  =  opcode == 7'b1101111;
	wire JALR = (opcode == 7'b1100111) && funct3_0;

	wire R_valid = AND | OR | ADD | XOR | SLL | SRL | SRA | SUB | SLT | SLTU
		| MUL | MULH | MULHSU | MULHU | DIV | DIVU | REM | REMU;
	wire I_valid = ANDI | ORI | ADDI | XORI | SLLI | SRLI | SRAI | SLTI | SLTIU;
	wire B_valid = BEQ | BNE | BLT | BGE | BLTU | BGEU;
	wire L_valid = LW | LH | LB | LHU | LBU;
	wire S_valid = SW | SH | SB;

	assign ALU_dispatch = (AND | OR | ADD | XOR | SLL | SRL | SRA | SUB | SLT | SLTU
		| I_valid | LUI | AUIPC) & |rd;
	assign mul_dispatch = (MUL | MULH | MULHSU | MULHU) & |rd;
	assign div_dispatch = (DIV | DIVU | REM | REMU) & |rd;
	assign load_dispatch = L_valid & |rd;
	assign store_dispatch = S_valid;
	assign branch_dispatch = B_valid;
	assign ujump_dispatch = JAL | JALR;

	localparam Imm_type_I = 3'b001;
	localparam Imm_type_B = 3'b010;
	localparam Imm_type_J = 3'b011;
	localparam Imm_type_S = 3'b100;
	localparam Imm_type_U = 3'b101;
	assign ImmSel = {3{JALR | L_valid | I_valid}} & Imm_type_I |
					{3{B_valid}}                  & Imm_type_B |
					{3{JAL}}                      & Imm_type_J |
					{3{S_valid}}                  & Imm_type_S |
					{3{LUI | AUIPC}}              & Imm_type_U ;

	localparam JUMP_BEQ  = 4'b0_001;
	localparam JUMP_BNE  = 4'b0_010;
	localparam JUMP_BLT  = 4'b0_011;
	localparam JUMP_BGE  = 4'b0_100;
	localparam JUMP_BLTU = 4'b0_101;
	localparam JUMP_BGEU = 4'b0_110;
	localparam JUMP_JAL  = 4'b0_000;
	localparam JUMP_JALR = 4'b1_000;
	assign JUMP_op ={4{BEQ}}  & JUMP_BEQ  |
					{4{BNE}}  & JUMP_BNE  |
					{4{BLT}}  & JUMP_BLT  |
					{4{BGE}}  & JUMP_BGE  |
					{4{BLTU}} & JUMP_BLTU |
					{4{BGEU}} & JUMP_BGEU |
					{4{JAL}}  & JUMP_JAL  |
					{4{JALR}} & JUMP_JALR ;

	localparam ALU_ADD  = 4'b0001;
	localparam ALU_SUB  = 4'b0010;
	localparam ALU_AND  = 4'b0011;
	localparam ALU_OR   = 4'b0100;
	localparam ALU_XOR  = 4'b0101;
	localparam ALU_SLL  = 4'b0110;
	localparam ALU_SRL  = 4'b0111;
	localparam ALU_SLT  = 4'b1000;
	localparam ALU_SLTU = 4'b1001;
	localparam ALU_SRA  = 4'b1010;
	localparam ALU_Ap4  = 4'b1011;
	localparam ALU_Bout = 4'b1100;
	assign ALU_op = {4{ADD | ADDI | AUIPC}} & ALU_ADD  |
					{4{SUB}}                & ALU_SUB  |
					{4{AND | ANDI}}         & ALU_AND  |
					{4{OR | ORI}}           & ALU_OR   |
					{4{XOR | XORI}}         & ALU_XOR  |
					{4{SLL | SLLI}}         & ALU_SLL  |
					{4{SRL | SRLI}}         & ALU_SRL  |
					{4{SLT | SLTI}}         & ALU_SLT  |
					{4{SLTU | SLTIU}}       & ALU_SLTU |
					{4{SRA | SRAI}}         & ALU_SRA  |
					{4{LUI}}                & ALU_Bout ;

	assign ALUSrcA = AUIPC;

	assign ALUSrcB = I_valid | LUI | AUIPC;

	assign FU_regWrite = ALU_dispatch | mul_dispatch | div_dispatch | load_dispatch;

endmodule