`timescale 1ns / 1ps

`include "define.vh"

//-----------------------------------------------------------------------------
// Module: RS_generic_line
//  - Holds one instruction until its operands arrive.
//  - q1/q2 = tag fields (0 = already have value).
//  - v1/v2 = data fields.
//  - in rust-like pseudo: enum Operand(NotReady(q), Ready(v));
//  - data_ready goes high when both q1 and q2 are zero.
//  - busy indicates the RS is occupied.
//-----------------------------------------------------------------------------
module RS_generic_line(
    input clk, rst, issue,                // issue = new op arrives
    input FU_result_taken,                // previous result consumed?
    input[40:0] cdb,                      // broadcast bus

    input[7:0]  q1_in, q2_in,             // incoming operand tags
    input[31:0] v1_in, v2_in,             // incoming operand data

    output data_ready,                    // both operands ready
    output reg busy,                      // RS occupied?
    output reg[31:0] v1, v2               // latched operand values
);

// Internal tag registers for operands
reg[7:0] q1, q2;

//-----------------------------------------------------------------------------
// Detect when the CDB tag matches our tag fields
//-----------------------------------------------------------------------------
wire cdb_match_q1 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q1;
wire cdb_match_q2 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q2;

//-----------------------------------------------------------------------------
// During the same cycle as issue, we may already catch a matching CDB.
//-----------------------------------------------------------------------------
wire init_clk_cdb_match_q1 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q1_in;
wire init_clk_cdb_match_q2 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q2_in;

//-----------------------------------------------------------------------------
// data_ready true only when RS is busy AND both tags have been zeroed.
// That means both v1 and v2 hold valid data.
//-----------------------------------------------------------------------------
assign data_ready = busy && q1 == 8'b0 && q2 == 8'b0;

//-----------------------------------------------------------------------------
// busy goes high on issue, low when its FU result is taken.
//-----------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst)            busy <= 1'b0;
    else if (issue)     busy <= 1'b1;
    else if (FU_result_taken) busy <= 1'b0;
end

//-----------------------------------------------------------------------------
// Operand #1 latch logic:
//  - On issue: if CDB already had our tag, grab data immediately.
//  - Else capture tag+value from inputs.
//  - Later, when CDB broadcasts matching tag, grab data.
//-----------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        q1 <= 8'b0; v1 <= 32'b0;
    end else if (issue) begin
        if (init_clk_cdb_match_q1) begin
            q1 <= 8'b0;
            v1 <= cdb[`CDB_DATA_FIELD];
        end else begin
            q1 <= q1_in;
            v1 <= v1_in;
        end
    end else if (cdb_match_q1) begin
        q1 <= 8'b0;
        v1 <= cdb[`CDB_DATA_FIELD];
    end
end

//-----------------------------------------------------------------------------
// Operand #2 latch logic (identical to operand #1).
//-----------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        q2 <= 8'b0; v2 <= 32'b0;
    end else if (issue) begin
        if (init_clk_cdb_match_q2) begin
            q2 <= 8'b0;
            v2 <= cdb[`CDB_DATA_FIELD];
        end else begin
            q2 <= q2_in;
            v2 <= v2_in;
        end
    end else if (cdb_match_q2) begin
        q2 <= 8'b0;
        v2 <= cdb[`CDB_DATA_FIELD];
    end
end

endmodule