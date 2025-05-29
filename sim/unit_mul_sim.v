`timescale 1ns / 1ps

module unit_mul_sim;

	reg clk, rst, issue;
	wire[40:0]cdb;

	reg[7:0]  q1_in, q2_in;
	reg[31:0] v1_in, v2_in;

	wire all_busy;
	wire cdb_request;
	wire[39:0] mul_cdb_out;

	unit_mul unit(.clk(clk),.rst(rst),.issue(issue),
		.cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),
		.v1_in(v1_in),.v2_in(v2_in),.all_busy(all_busy),.cdb_request(cdb_request),
		.cdb_out(mul_cdb_out));

	common_data_bus cd_b(.clk(clk),.rst(rst),
		.ALU_cdb_request(1'b0),.mul_cdb_request(cdb_request),.div_cdb_request(1'b0),.ls_cdb_request(1'b0),
		.ALU_cdb_in(40'b0),.mul_cdb_in(mul_cdb_out),.div_cdb_in(40'b0),.ls_cdb_in(40'b0),.cdb(cdb));

	initial begin
		clk = 1;
		rst = 1;
		issue = 0;

		q1_in = 0;
		q2_in = 0;
		v1_in = 0;
		v2_in = 0;

		#5;
		rst = 0;
		issue = 1;
		q1_in = 8'b01000_001;
		v1_in = 0;
		q2_in = 0;
		v2_in = 2;

		#10;
		q1_in = 0;
		v1_in = 2;
		v2_in = 3;

		#10;
		v1_in = 4;
		v2_in = 5;

		#6;
		issue = 0;
	end

	always #5 clk = ~clk;

endmodule