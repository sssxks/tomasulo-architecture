`timescale 1ns / 1ps

`include "define.vh"

module RS_generic_line(
	input clk, rst, issue,
	input FU_result_taken,
	input[40:0] cdb,

	input[7:0]  q1_in, q2_in,
	input[31:0] v1_in, v2_in,

	output data_ready,
	output reg busy,
	output reg[31:0] v1, v2
);

	reg[7:0]  q1, q2;
	
	wire cdb_match_q1 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q1;
	wire cdb_match_q2 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q2;

	wire init_clk_cdb_match_q1 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q1_in;
	wire init_clk_cdb_match_q2 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q2_in;

	assign data_ready = busy && q1 == 8'b0 && q2 == 8'b0;

	always@(posedge clk or posedge rst) begin
		if(rst) busy <= 1'b0;
		else if(issue) busy <= 1'b1;
		else if(FU_result_taken) busy <= 1'b0;
	end

	always@(posedge clk or posedge rst) begin
		if(rst) begin
			q1 <= 8'b0;
			v1 <= 32'b0;
		end
		else if(issue) begin
			if(init_clk_cdb_match_q1) begin
				q1 <= 8'b0;
				v1 <= cdb[`CDB_DATA_FIELD];
			end
			else begin
				q1 <= q1_in;
				v1 <= v1_in;
			end
		end
		else if(cdb_match_q1) begin
			q1 <= 8'b0;
			v1 <= cdb[`CDB_DATA_FIELD];
		end
	end

	always@(posedge clk or posedge rst) begin
		if(rst) begin
			q2 <= 8'b0;
			v2 <= 32'b0;
		end
		else if(issue) begin
			if(init_clk_cdb_match_q2) begin
				q2 <= 8'b0;
				v2 <= cdb[`CDB_DATA_FIELD];
			end
			else begin
				q2 <= q2_in;
				v2 <= v2_in;
			end
		end
		else if(cdb_match_q2) begin
			q2 <= 8'b0;
			v2 <= cdb[`CDB_DATA_FIELD];
		end
	end

endmodule