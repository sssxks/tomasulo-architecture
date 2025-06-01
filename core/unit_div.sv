`timescale 1ns / 1ps
`include "define.vh"

module unit_div(
    input clk, rst, issue, flush,
    input cdb_bus_t cdb_i,               // Common Data Bus struct

    input tagged_data_t op_a_i,          // Operand A (tag + data)
    input tagged_data_t op_b_i,          // Operand B (tag + data)

    output logic all_busy,               // all RS full?
    output logic cdb_request,            // ready to broadcast?
    output tagged_data_t cdb_data_o,     // what to broadcast (tag + data)
    output logic[`NUM_SRBITS-1:0] issue_tag // which RS got an issue slot
);

    // Reservation stations for division
    reg rs1_FU_result_taken, rs2_FU_result_taken, rs3_FU_result_taken;
    wire rs1_data_ready, rs2_data_ready, rs3_data_ready;
    wire rs1_busy, rs2_busy, rs3_busy;
    wire[31:0] rs1_v1, rs1_v2, rs2_v1, rs2_v2, rs3_v1, rs3_v2;

    assign all_busy = rs1_busy & rs2_busy & rs3_busy;
    wire rs1_issue = issue & ~rs1_busy;
    wire rs2_issue = issue & rs1_busy & ~rs2_busy;
    wire rs3_issue = issue & rs1_busy & rs2_busy & ~rs3_busy;
    assign issue_tag = {`FU_DIV_TAG, rs1_issue, rs2_issue, rs3_issue} & {`NUM_SRBITS{issue}};

    wire FU_result_taken = cdb_i.valid && (cdb_i.tag[7:3] == `FU_DIV_TAG); // Use cdb_i struct

    RS_generic_line rs1_div(
        .clk(clk), .rst(rst), .issue(rs1_issue), .flush(flush), .FU_result_taken(rs1_FU_result_taken),
        .cdb({cdb_i.valid, cdb_i.tag, cdb_i.data}), // Pass packed CDB vector
        .q1_in(op_a_i.tag), .q2_in(op_b_i.tag),
        .v1_in(op_a_i.val), .v2_in(op_b_i.val),
        .data_ready(rs1_data_ready), .busy(rs1_busy), .v1(rs1_v1), .v2(rs1_v2)
    );

    RS_generic_line rs2_div(
        .clk(clk), .rst(rst), .issue(rs2_issue), .flush(flush), .FU_result_taken(rs2_FU_result_taken),
        .cdb({cdb_i.valid, cdb_i.tag, cdb_i.data}), // Pass packed CDB vector
        .q1_in(op_a_i.tag), .q2_in(op_b_i.tag),
        .v1_in(op_a_i.val), .v2_in(op_b_i.val),
        .data_ready(rs2_data_ready), .busy(rs2_busy), .v1(rs2_v1), .v2(rs2_v2)
    );

    RS_generic_line rs3_div(
        .clk(clk), .rst(rst), .issue(rs3_issue), .flush(flush), .FU_result_taken(rs3_FU_result_taken),
        .cdb({cdb_i.valid, cdb_i.tag, cdb_i.data}), // Pass packed CDB vector
        .q1_in(op_a_i.tag), .q2_in(op_b_i.tag),
        .v1_in(op_a_i.val), .v2_in(op_b_i.val),
        .data_ready(rs3_data_ready), .busy(rs3_busy), .v1(rs3_v1), .v2(rs3_v2)
    );

    // Functional unit control
    reg FU_div_EN;
    reg[2:0] FU_poi; // RS selector: 100 for RS1, 010 for RS2, 001 for RS3
    wire FU_div_finish;
    reg FU_div_finish_reg;
    reg[31:0] ALUA, ALUB;
    wire[31:0] div_out;
    reg[31:0] div_out_reg;

    // Schedule and issue to FU_div
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            FU_poi <= 3'b0;
            ALUA <= 32'b0;
            ALUB <= 32'b0;
            FU_div_EN <= 1'b0;
        end else if (FU_result_taken || FU_poi == 3'b0) begin // Re-evaluate if result taken or no RS selected
            if (rs1_data_ready && (FU_poi == 3'b0 || FU_poi != 3'b100)) begin
                FU_poi <= 3'b100; // Select RS1
                ALUA <= rs1_v1;
                ALUB <= rs1_v2;
                FU_div_EN <= 1'b1;
            end else if (rs2_data_ready && (FU_poi == 3'b0 || FU_poi != 3'b010)) begin
                FU_poi <= 3'b010; // Select RS2
                ALUA <= rs2_v1;
                ALUB <= rs2_v2;
                FU_div_EN <= 1'b1;
            end else if (rs3_data_ready && (FU_poi == 3'b0 || FU_poi != 3'b001)) begin
                FU_poi <= 3'b001; // Select RS3
                ALUA <= rs3_v1;
                ALUB <= rs3_v2;
                FU_div_EN <= 1'b1;
            end else begin
                FU_poi <= 3'b0; // No ready RS or selected one was just taken
                FU_div_EN <= 1'b0;
            end
        // If FU is already running and result not taken, keep EN high (for multi-cycle)
        end else if (FU_div_EN && !FU_div_finish) begin
             // FU_div_EN remains 1'b1;
        end else begin // Default to off if no other condition met
            FU_div_EN <= 1'b0;
        end
    end

    // Division unit
    FU_div div(.clk(clk), .EN(FU_div_EN), .A(ALUA), .B(ALUB), .res(div_out), .finish(FU_div_finish));

    // Capture finish and hold result
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            FU_div_finish_reg <= 1'b0;
            div_out_reg <= 32'b0;
        end else if (FU_div_finish && !FU_result_taken) begin
            FU_div_finish_reg <= 1'b1;
            div_out_reg <= div_out;
        end else if (FU_result_taken) begin
            FU_div_finish_reg <= 1'b0;
        end
    end

    // CDB output
    assign cdb_request = FU_div_finish | FU_div_finish_reg;
    assign cdb_data_o.tag = {`FU_DIV_TAG, FU_poi};
    assign cdb_data_o.val = FU_div_finish_reg ? div_out_reg : div_out;

    // Track result taken for RS
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            rs1_FU_result_taken <= 1'b0;
            rs2_FU_result_taken <= 1'b0;
            rs3_FU_result_taken <= 1'b0;
        end else begin
            // Check if the tag on CDB matches this FU and the specific RS
            rs1_FU_result_taken <= FU_result_taken && (cdb_i.tag[2:0] == 3'b100);
            rs2_FU_result_taken <= FU_result_taken && (cdb_i.tag[2:0] == 3'b010);
            rs3_FU_result_taken <= FU_result_taken && (cdb_i.tag[2:0] == 3'b001);
        end
    end

endmodule