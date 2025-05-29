`timescale 1ns / 1ps

`include "define.vh"

module RS_ALU_line(
	input clk, rst, issue,
	input FU_result_taken,
	input[40:0] cdb,

	input[3:0] ALUControl_in,
	input[7:0]  q1_in, q2_in,
	input[31:0] v1_in, v2_in,

	output data_ready,
	output busy,

	output reg[3:0] ALUControl,
	output [31:0] v1, v2
);

	RS_generic_line rs_g(.clk(clk),.rst(rst),.issue(issue),.FU_result_taken(FU_result_taken),
		.cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),.v1_in(v1_in),.v2_in(v2_in),
		.data_ready(data_ready),.busy(busy),.v1(v1),.v2(v2));

	always@(posedge clk or posedge rst)begin
		if(rst) ALUControl <= 4'b0;
		else if(issue) ALUControl <= ALUControl_in;
	end

endmodule