`timescale 1ns / 1ps

`include "define.vh"

module JumpHandle(
	input branch_issue, ujump_issue,
	input[3:0]JUMP_op,
	input[`NUM_SRBITS-1:0] q_rs1_in, q_rs2_in,
	input[31:0] rs1_data_in, rs2_data_in, imm, PC,
	input[`NUM_CDBBITS-1:0] cdb,
	output[31:0] PC_jump,
	output to_jump,
	output jump_stall
);

	wire JALR = ujump_issue & JUMP_op[3];
	wire JAL = ujump_issue & ~JUMP_op[3];

	wire cdb_match1 = cdb[`CDB_ON_FIELD] && q_rs1_in == cdb[`CDB_TAG_FIELD];
	wire cdb_match2 = cdb[`CDB_ON_FIELD] && q_rs2_in == cdb[`CDB_TAG_FIELD];

	wire[31:0] rs1_data = cdb_match1 ? cdb[`CDB_DATA_FIELD] : rs1_data_in;
	wire[31:0] rs2_data = cdb_match2 ? cdb[`CDB_DATA_FIELD] : rs2_data_in;

	wire cmp_res;
	cmp_32 cmp(.a(rs1_data),.b(rs2_data),.ctrl(JUMP_op[2:0]),.c(cmp_res));

	add_32 a(.a(JALR ? rs1_data : PC),.b(imm),.c(PC_jump));

	assign jump_stall = |q_rs1_in & ~cdb_match1 & (branch_issue | JALR) |
						|q_rs2_in & ~cdb_match2 & branch_issue;

	assign to_jump = ujump_issue | branch_issue & cmp_res;

endmodule