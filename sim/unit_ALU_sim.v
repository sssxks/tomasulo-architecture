`timescale 1ns / 1ps

module unit_ALU_sim;

	reg clk, rst, reg_issue, ALU_issue;
	wire[40:0]cdb;

	reg[4:0] waddr;
	reg[7:0] w_tag;
	wire[39:0] rdata_A, rdata_B;

	reg mul_cdb_request;

	wire all_busy;
	wire ALU_cdb_request;
	wire[39:0] ALU_cdb_out;
	wire[7:0] issue_tag;

	taggedRegs regs(.clk(clk),.rst(rst),.FU_regWrite(reg_issue),
		.ujump_wb(1'b0),.raddr_A(5'd1),.raddr_B(5'd2),
		.waddr(waddr),.w_tag(w_tag),.cdb(cdb),
		.rdata_A(rdata_A),.rdata_B(rdata_B));

	unit_ALU unit(.clk(clk),.rst(rst),.issue(ALU_issue),
		.cdb(cdb),.ALUControl_in(4'd1),.q1_in(rdata_A[39:32]),.q2_in(rdata_B[39:32]),
		.v1_in(rdata_A[31:0]),.v2_in(rdata_B[31:0]),.all_busy(all_busy),.cdb_request(ALU_cdb_request),
		.cdb_out(ALU_cdb_out),.issue_tag(issue_tag));

	common_data_bus cd_b(.clk(clk),.rst(rst),
		.ALU_cdb_request(ALU_cdb_request),.mul_cdb_request(mul_cdb_request),.div_cdb_request(1'b0),.ls_cdb_request(1'b0),
		.ALU_cdb_in(ALU_cdb_out),.mul_cdb_in(40'h42_0000_00A6),.div_cdb_in(40'b0),.ls_cdb_in(40'b0),.cdb(cdb));

	initial begin
		clk = 1;
		rst = 1;
		reg_issue = 0;
		ALU_issue = 0;

		waddr = 1;
		w_tag = 8'h42;
		mul_cdb_request = 0;

		#2;
		rst = 0;
		reg_issue = 1;

		#10;
		waddr = 2;

		#10;
		reg_issue = 0;

		#10;
		mul_cdb_request = 1;
		#10;
		mul_cdb_request = 0;

		#10;
		reg_issue = 1;
		ALU_issue = 1;

		#1;
		w_tag = issue_tag;

		#9;
		ALU_issue = 0;
		reg_issue = 0;

		#10;

	end

	always #5 clk = ~clk;

endmodule