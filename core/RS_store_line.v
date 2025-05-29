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
	output reg busy,

	output reg [31:0] addr,
	output reg [31:0] data,
	output reg[2:0] mem_u_b_h_w
);

// tag and data tracking
reg [7:0] q_data;
wire cdb_match_q = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q_data;
wire init_clk_cdb_match = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q_data_in;

// data ready when reservation has data
assign data_ready = busy && q_data == 8'b0;

always @(posedge clk or posedge rst) begin
    if (rst) busy <= 1'b0;
    else if (issue) busy <= 1'b1;
    else if (result_taken) busy <= 1'b0;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        q_data <= 8'b0;
        data <= 32'b0;
    end else if (issue) begin
        if (init_clk_cdb_match) begin
            q_data <= 8'b0;
            data <= cdb[`CDB_DATA_FIELD];
        end else begin
            q_data <= q_data_in;
            data <= data_in;
        end
    end else if (cdb_match_q) begin
        q_data <= 8'b0;
        data <= cdb[`CDB_DATA_FIELD];
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        addr <= 32'b0;
        mem_u_b_h_w <= 3'b0;
    end else if (issue) begin
        addr <= addr_in;
        mem_u_b_h_w <= mem_u_b_h_w_in;
    end
end

endmodule