`timescale 1ns / 1ps

`include "define.vh"

module unit_div(
	input clk, rst, issue,
	input[`NUM_CDBBITS-1:0]cdb,

	input[7:0]  q1_in, q2_in,
	input[31:0] v1_in, v2_in,

	output all_busy,
	output cdb_request,
	output[`NUM_CDBBITS-2:0] cdb_out,
	output[`NUM_SRBITS-1:0] issue_tag
);



endmodule