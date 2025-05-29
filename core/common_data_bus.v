`timescale 1ns / 1ps

`include "define.vh"

module common_data_bus(
	input clk, rst,
	input ALU_cdb_request, mul_cdb_request, div_cdb_request, ls_cdb_request,
	input[39:0] ALU_cdb_in, mul_cdb_in, div_cdb_in, ls_cdb_in,
	output reg[40:0] cdb
);



endmodule