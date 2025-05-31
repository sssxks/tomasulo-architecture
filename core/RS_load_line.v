`timescale 1ns / 1ps

`include "define.vh"

module RS_load_line(
	input clk, rst, issue,
	input FU_result_taken,
	input[40:0] cdb,

	input[31:0] addr_in,
	input[2:0] mem_u_b_h_w_in,

	output reg busy,

	output reg [31:0] addr,
	output reg[2:0] mem_u_b_h_w
);

// Latch busy, address, and mem type on issue; clear busy on result taken; reset on rst
always @(posedge clk or posedge rst) begin
    if (rst) begin
        busy <= 1'b0;
        addr <= 32'b0;
        mem_u_b_h_w <= 3'b0;
    end else if (issue) begin
        busy <= 1'b1;
        addr <= addr_in;
        mem_u_b_h_w <= mem_u_b_h_w_in;
    end else if (FU_result_taken) begin
        busy <= 1'b0;
    end
end

endmodule