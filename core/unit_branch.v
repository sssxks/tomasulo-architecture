`timescale 1ns / 1ps
`include "define.vh"

//----------------------------------------------------------------------------- 
// Module: unit_branch
//  - Single-entry reservation station for branch/jump resolution.
//  - Uses JumpHandle to compute target and condition; broadcasts on CDB.
//----------------------------------------------------------------------------- 
module unit_branch(
    input                  clk,
    input                  rst,
    input                  branch_issue,
    input                  ujump_issue,
    input                  flush,
    input [`NUM_CDBBITS-1:0] cdb,
    input [3:0]             JUMP_op,
    input [`NUM_SRBITS-1:0] q1_in, q2_in,
    input [31:0]            v1_in, v2_in,
    input [31:0]            imm, PC,
    output                  jump_stall,
    output                  cdb_request,
    output[`NUM_CDBBITS-2:0] cdb_out,
    output[`NUM_SRBITS-1:0]  issue_tag
);

    // Reservation station state
    reg               busy;
    reg [`NUM_SRBITS-1:0] q1, q2;
    reg [31:0]        v1, v2, imm_reg, PC_reg;

    // Forwarding matches
    wire cdb_match1 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q1;
    wire cdb_match2 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q2;

    // Data ready when both operands ready
    wire data_ready = busy && q1==0 && q2==0;

    // Issue tagging (only one slot)
    wire rs_issue = issue && !busy;
    wire issue = branch_issue | ujump_issue;
    assign issue_tag = {`FU_BRANCH_TAG, rs_issue, 2'b00} & {`NUM_SRBITS{issue}};
    assign jump_stall = (q1 != 0 && !cdb_match1) || (q2 != 0 && !cdb_match2);

    // Detect if CDB carries operands at issue cycle
    wire init_cdb_match1 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q1_in;
    wire init_cdb_match2 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q2_in;

    // Latch operands, JUMP_op, imm, and PC
    always @(posedge clk or posedge rst) begin
        if (rst || flush) begin
            busy    <= 0;
        end else if (rs_issue) begin
            busy    <= 1;
            q1      <= q1_in;
            q2      <= q2_in;
            v1      <= init_cdb_match1 ? cdb[`CDB_DATA_FIELD] : v1_in;
            v2      <= init_cdb_match2 ? cdb[`CDB_DATA_FIELD] : v2_in;
            imm_reg <= imm;
            PC_reg  <= PC;
        end else if (data_ready) begin
            busy <= 0;
        end
    end

    // Compute jump target and condition
    wire[31:0] PC_jump;
    wire to_jump;
    JumpHandle jh(
        .branch_issue(branch_issue),
        .ujump_issue(ujump_issue),
        .JUMP_op(JUMP_op),
        .q_rs1_in(q1),.q_rs2_in(q2),
        .rs1_data_in(v1),.rs2_data_in(v2),
        .imm(imm_reg),.PC(PC_reg),
        .cdb(cdb),
        .PC_jump(PC_jump),
        .to_jump(to_jump),
        .jump_stall()
    );

    // CDB broadcast logic
    reg result_taken;
    reg[31:0] jump_target_reg;

    always @(negedge clk or posedge rst) begin
        if (rst) begin
            result_taken <= 0;
            jump_target_reg <= 0;
        end else if (data_ready && !result_taken) begin
            result_taken <= 1;
            jump_target_reg <= PC_jump;
        end else if (cdb[`CDB_ON_FIELD] && cdb[`CDB_FU_FIELD] == `FU_BRANCH_TAG) begin
            result_taken <= 0;
        end
    end

    assign cdb_request = result_taken;
    assign cdb_out = {`FU_BRANCH_TAG, 3'b001, jump_target_reg};
endmodule
