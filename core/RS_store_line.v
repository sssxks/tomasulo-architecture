`timescale 1ns / 1ps

`include "define.vh"

module RS_store_line(
	input clk, rst, issue,
	input result_taken,
	input[40:0] cdb,

	input[7:0]  q_data_in,
	input[31:0] addr_in, data_in,
	input[2:0] mem_u_b_h_w_in,


	output data_ready,
	output busy,

	output [31:0] addr, data,
	output reg[2:0] mem_u_b_h_w
);



endmodule