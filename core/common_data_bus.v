`timescale 1ns / 1ps

`include "define.vh"

module common_data_bus(
	input clk, rst,
	input ALU_cdb_request, mul_cdb_request, div_cdb_request, ls_cdb_request,
	input[39:0] ALU_cdb_in, mul_cdb_in, div_cdb_in, ls_cdb_in,
	output reg[40:0] cdb
);

	// Arbitration: prioritize ALU > mul > div > ld/st
	always @(posedge clk or posedge rst) begin
		if (rst) begin
			cdb <= {1'b0, 40'b0};
		end else if (ALU_cdb_request) begin
			cdb <= {1'b1, ALU_cdb_in};
		end else if (mul_cdb_request) begin
			cdb <= {1'b1, mul_cdb_in};
		end else if (div_cdb_request) begin
			cdb <= {1'b1, div_cdb_in};
		end else if (ls_cdb_request) begin
			cdb <= {1'b1, ls_cdb_in};
		end else begin
			cdb <= {1'b0, 40'b0};
		end
	end

endmodule