`timescale 1ns / 1ps

module taggedRegs_sim;

	reg clk, rst, issue;
	reg branch_issue, ujump_issue;
	reg[4:0] raddr_A, raddr_B, waddr;
	reg[7:0] w_tag;
	wire[31:0] PC_jump, PCp4;

	wire to_jump, jump_stall;
	wire[40:0] cdb;
	wire[39:0] rdata_A, rdata_B;

	reg ALU_cdb_request;
	reg[39:0] ALU_cdb_out;


	taggedRegs regs(.clk(clk),.rst(rst),.issue(issue),
		.ujump_wb(ujump_issue & ~jump_stall),.raddr_A(raddr_A),.raddr_B(raddr_B),
		.waddr(waddr),.w_tag(w_tag),.cdb(cdb),.PCp4(PCp4),
		.rdata_A(rdata_A),.rdata_B(rdata_B));

	common_data_bus cd_b(.clk(clk),.rst(rst),
		.ALU_cdb_request(ALU_cdb_request),.mul_cdb_request(1'b0),.div_cdb_request(1'b0),.ls_cdb_request(1'b0),
		.ALU_cdb_in(ALU_cdb_out),.mul_cdb_in(40'b0),.div_cdb_in(40'b0),.ls_cdb_in(40'b0),.cdb(cdb));

	JumpHandle jh(.branch_issue(branch_issue),.ujump_issue(ujump_issue),.JUMP_op(4'b1_000),
		.q_rs1_in(rdata_A[`CDB_TAG_FIELD]),.q_rs2_in(rdata_B[`CDB_TAG_FIELD]),
		.rs1_data_in(rdata_A[`CDB_DATA_FIELD]),.rs2_data_in(rdata_B[`CDB_DATA_FIELD]),
		.imm(32'hA6),.PC(32'hC0),.cdb(cdb),.PC_jump(PC_jump),.PCp4(PCp4),
		.to_jump(to_jump),.jump_stall(jump_stall));

	initial begin
		clk = 1;
		rst = 1;
		issue = 0;
		branch_issue = 0;
		ujump_issue = 0;
		raddr_A = 1;
		raddr_B = 2;
		waddr = 0;
		w_tag = 8'b10000_001;

		ALU_cdb_request = 0;
		ALU_cdb_out = 40'h82_0000_0048;

		#2;
		rst = 0;
		issue = 1;
		waddr = 1;

		#10;
		w_tag = 8'b10000_010;

		#10;
		waddr = 2;
		w_tag = 8'b10000_100;

		#10;
		issue = 0;
		ujump_issue = 1;

		#20;
		ALU_cdb_request = 1;

		#10;
		ALU_cdb_request = 0;

		#10;
		ujump_issue = 0;

	end

	always #5 clk = ~clk;

endmodule