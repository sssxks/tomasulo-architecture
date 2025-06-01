`timescale 1ns / 1ps
`include "define.vh"

//-----------------------------------------------------------------------------
// Module: unit_branch
//  - Single-entry reservation station for branch/jump resolution.
//  - Uses JumpHandle to compute target and condition; broadcasts on CDB.
//-----------------------------------------------------------------------------
module unit_branch(
    input clk,
    input rst,
    input branch_issue,        // Specific issue signal for branches
    input ujump_issue,         // Specific issue signal for unconditional jumps
    input flush,
    input cdb_bus_t cdb_i,     // CDB bus struct
    input [3:0] JUMP_op,
    input tagged_data_t op_a_i, // Operand A (rs1)
    input tagged_data_t op_b_i, // Operand B (rs2)
    input [31:0] imm,
    input [31:0] PC,
    output logic jump_stall,   // Stall signal specific to branch unit
    output logic cdb_request,
    output tagged_data_t cdb_data_o,
    output logic[`NUM_SRBITS-1:0] issue_tag // Tag for this issue
);

    // Reservation station state
    reg busy;
    reg [`NUM_SRBITS-1:0] q1, q2; // Tags for operands
    reg [31:0] v1, v2, imm_reg, PC_reg;
    reg [3:0] JUMP_op_reg; // Register for JUMP_op

    // Forwarding matches using cdb_i struct
    wire cdb_match1 = cdb_i.valid && (cdb_i.tag == q1);
    wire cdb_match2 = cdb_i.valid && (cdb_i.tag == q2);

    // Data ready when both operands ready (q1 and q2 are 0)
    wire data_ready = busy && (q1 == `NUM_SRBITS'b0) && (q2 == `NUM_SRBITS'b0);

    // Issue tagging (only one slot for branch unit)
    wire actual_issue = branch_issue | ujump_issue; // Combined issue condition
    wire rs_issue = actual_issue && !busy;
    assign issue_tag = {`FU_BRANCH_TAG, 1'b1, 2'b00} & {`NUM_SRBITS{actual_issue}}; // Hardcoded RS index '1' (3'b100) effectively

    // Stall if operands are not ready and not on CDB this cycle
    assign jump_stall = busy && ((q1 != `NUM_SRBITS'b0 && !cdb_match1) || (q2 != `NUM_SRBITS'b0 && !cdb_match2));

    // Detect if CDB carries operands at issue cycle
    wire init_cdb_match1 = cdb_i.valid && (cdb_i.tag == op_a_i.tag);
    wire init_cdb_match2 = cdb_i.valid && (cdb_i.tag == op_b_i.tag);

    // Latch operands, JUMP_op, imm, and PC
    always @(posedge clk or posedge rst) begin
        if (rst || flush) begin
            busy    <= 1'b0;
            q1      <= `NUM_SRBITS'b0;
            q2      <= `NUM_SRBITS'b0;
            v1      <= 32'b0;
            v2      <= 32'b0;
            imm_reg <= 32'b0;
            PC_reg  <= 32'b0;
            JUMP_op_reg <= 4'b0;
        end else if (rs_issue) begin
            busy    <= 1'b1;
            JUMP_op_reg <= JUMP_op; // Latch JUMP_op
            // Latch rs1
            if (op_a_i.tag == `NUM_SRBITS'b0) begin
                q1 <= `NUM_SRBITS'b0;
                v1 <= op_a_i.val;
            end else if (init_cdb_match1) begin
                q1 <= `NUM_SRBITS'b0;
                v1 <= cdb_i.data;
            end else begin
                q1 <= op_a_i.tag;
                v1 <= op_a_i.val; // Keep original val for potential later use or debug
            end
            // Latch rs2
            if (op_b_i.tag == `NUM_SRBITS'b0) begin
                q2 <= `NUM_SRBITS'b0;
                v2 <= op_b_i.val;
            end else if (init_cdb_match2) begin
                q2 <= `NUM_SRBITS'b0;
                v2 <= cdb_i.data;
            end else begin
                q2 <= op_b_i.tag;
                v2 <= op_b_i.val;
            end
            imm_reg <= imm;
            PC_reg  <= PC;
        end else if (busy) begin // If busy, check for CDB updates
            if (cdb_match1) begin
                v1 <= cdb_i.data;
                q1 <= `NUM_SRBITS'b0;
            end
            if (cdb_match2) begin
                v2 <= cdb_i.data;
                q2 <= `NUM_SRBITS'b0;
            end
        end

        if (data_ready && !cdb_request) begin // Reset busy AFTER data is ready and processed (indicated by cdb_request going high then low)
             // This will be reset by result_taken logic now
        end
    end

    // Compute jump target and condition
    wire[31:0] PC_jump;
    wire to_jump; // This signal indicates if the branch is taken/untaken or jump occurs
    JumpHandle jh(
        .branch_issue(branch_issue && busy), // Pass registered JUMP_op
        .ujump_issue(ujump_issue && busy),   // Pass registered JUMP_op
        .JUMP_op(JUMP_op_reg), // Use latched JUMP_op
        .q_rs1_in(q1),.q_rs2_in(q2), // These are internal RS tags, should be 0 if data_ready
        .rs1_data_in(v1),.rs2_data_in(v2),
        .imm(imm_reg),.PC(PC_reg),
        .cdb({cdb_i.valid, cdb_i.tag, cdb_i.data}), // Pass packed CDB
        .PC_jump(PC_jump),   // Calculated target
        .to_jump(to_jump),   // Actual outcome (taken/not taken)
        .jump_stall()      // jump_stall output of JumpHandle is not used here
    );

    // CDB broadcast logic
    reg result_can_broadcast; // Indicates result is computed and ready for CDB
    reg result_on_cdb;      // Indicates result has been successfully placed on CDB

    always @(posedge clk or posedge rst) begin
        if (rst || flush) begin
            result_can_broadcast <= 1'b0;
            result_on_cdb <= 1'b0;
            busy <= 1'b0; // Ensure busy is reset on flush
        end else begin
            if (data_ready && !result_can_broadcast && !result_on_cdb) begin // Data becomes ready, prepare to broadcast
                result_can_broadcast <= 1'b1;
            end

            if (result_can_broadcast && !result_on_cdb) begin // Attempting to broadcast
                // If CDB accepts the request (simulated by checking if it was this FU that won arbitration)
                // This FU's tag for CDB is {`FU_BRANCH_TAG, 3'b001}
                if (cdb_i.valid && cdb_i.tag == {`FU_BRANCH_TAG, 3'b001}) begin
                    result_on_cdb <= 1'b1;
                    result_can_broadcast <= 1'b0; // Successfully broadcasted
                    busy <= 1'b0; // Free up RS
                end
                // If not granted, result_can_broadcast remains high to retry next cycle.
            end

            if (result_on_cdb && !(cdb_i.valid && cdb_i.tag == {`FU_BRANCH_TAG, 3'b001})) begin
                // If result was on CDB and current CDB cycle is for someone else or invalid, reset flag
                result_on_cdb <= 1'b0;
            end
        end
    end

    assign cdb_request = result_can_broadcast; // Request CDB if result is ready and not yet broadcast
    // The actual target address for JALR is PC_jump. For branches, it's PC_jump if taken, PC+4 if not.
    // The value broadcast on CDB by branch unit is typically the actual next PC after branch resolution.
    // For now, cdb_data_o.val will be PC_jump (calculated target for JAL/JALR, or branch target if taken)
    // The mispredict logic in ROB will use this.
    assign cdb_data_o.tag = {`FU_BRANCH_TAG, 3'b001}; // FU_BRANCH_TAG (5 bits) + RS index (3 bits)
    assign cdb_data_o.val = PC_jump; // Actual calculated target address
    // The 'to_jump' (actual outcome) is implicitly handled by ROB using this target.
    // If it's a branch and not taken, ROB might use PC+4 based on its own info.
    // Or, this unit could send 'to_jump' on CDB if needed, but current cdb_bus_t doesn't support it.

endmodule
