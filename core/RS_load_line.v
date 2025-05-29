`timescale 1ns / 1ps

`include "define.vh"

module RS_load_line(
	input clk, rst, issue,
	input FU_result_taken,
	input[40:0] cdb,

	input[31:0] addr_in,
	input[2:0] mem_u_b_h_w_in,

	output busy,

	output [31:0] addr,
	output reg[2:0] mem_u_b_h_w
);



endmodule