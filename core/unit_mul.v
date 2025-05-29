`timescale 1ns / 1ps

`include "define.vh"

module unit_mul(
	input clk, rst, issue,
	input[`NUM_CDBBITS-1:0]cdb,

	input[7:0]  q1_in, q2_in,
	input[31:0] v1_in, v2_in,

	output all_busy,
	output cdb_request,
	output[`NUM_CDBBITS-2:0] cdb_out,
	output[`NUM_SRBITS-1:0] issue_tag
);

    // Reservation stations for multiply
    reg rs1_FU_result_taken, rs2_FU_result_taken, rs3_FU_result_taken;
    wire rs1_data_ready, rs2_data_ready, rs3_data_ready;
    wire rs1_busy, rs2_busy, rs3_busy;
    wire[31:0] rs1_v1, rs1_v2, rs2_v1, rs2_v2, rs3_v1, rs3_v2;

    assign all_busy = rs1_busy & rs2_busy & rs3_busy;
    wire rs1_issue = issue & ~rs1_busy;
    wire rs2_issue = issue & rs1_busy & ~rs2_busy;
    wire rs3_issue = issue & rs1_busy & rs2_busy & ~rs3_busy;
    assign issue_tag = {`FU_MUL_TAG, rs1_issue, rs2_issue, rs3_issue} & {`NUM_SRBITS{issue}};

    wire FU_result_taken = cdb[`CDB_ON_FIELD] && cdb[`CDB_FU_FIELD] == `FU_MUL_TAG;

    RS_generic_line rs1_mul(.clk(clk),.rst(rst),.issue(rs1_issue),.FU_result_taken(rs1_FU_result_taken),
        .cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),.v1_in(v1_in),.v2_in(v2_in),
        .data_ready(rs1_data_ready),.busy(rs1_busy),.v1(rs1_v1),.v2(rs1_v2));

    RS_generic_line rs2_mul(.clk(clk),.rst(rst),.issue(rs2_issue),.FU_result_taken(rs2_FU_result_taken),
        .cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),.v1_in(v1_in),.v2_in(v2_in),
        .data_ready(rs2_data_ready),.busy(rs2_busy),.v1(rs2_v1),.v2(rs2_v2));

    RS_generic_line rs3_mul(.clk(clk),.rst(rst),.issue(rs3_issue),.FU_result_taken(rs3_FU_result_taken),
        .cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),.v1_in(v1_in),.v2_in(v2_in),
        .data_ready(rs3_data_ready),.busy(rs3_busy),.v1(rs3_v1),.v2(rs3_v2));

    // Functional unit control
    reg FU_mul_EN;
    reg[2:0] FU_poi;
    wire FU_mul_finish;
    reg FU_mul_finish_reg;
    reg[31:0] ALUA, ALUB;
    wire[31:0] mul_out;
    reg[31:0] mul_out_reg;

    // Schedule and issue to FU_mul
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            FU_poi <= 3'b0;
            ALUA <= 32'b0;
            ALUB <= 32'b0;
            FU_mul_EN <= 1'b0;
        end else if (FU_result_taken || FU_poi == 3'b0) begin
            if (rs1_data_ready & ~FU_poi[2]) begin
                FU_poi <= 3'b100;
                ALUA <= rs1_v1;
                ALUB <= rs1_v2;
                FU_mul_EN <= 1'b1;
            end else if (rs2_data_ready & ~FU_poi[1]) begin
                FU_poi <= 3'b010;
                ALUA <= rs2_v1;
                ALUB <= rs2_v2;
                FU_mul_EN <= 1'b1;
            end else if (rs3_data_ready & ~FU_poi[0]) begin
                FU_poi <= 3'b001;
                ALUA <= rs3_v1;
                ALUB <= rs3_v2;
                FU_mul_EN <= 1'b1;
            end else begin
                FU_poi <= 3'b0;
                FU_mul_EN <= 1'b0;
            end
        end else
            FU_mul_EN <= 1'b0;
    end

    // Multiply unit
    FU_mul mul(.clk(clk),.EN(FU_mul_EN),.A(ALUA),.B(ALUB),.res(mul_out),.finish(FU_mul_finish));

    // Capture finish and hold result
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            FU_mul_finish_reg <= 1'b0;
            mul_out_reg <= 32'b0;
        end else if (FU_mul_finish & ~FU_result_taken) begin
            FU_mul_finish_reg <= 1'b1;
            mul_out_reg <= mul_out;
        end else if (FU_result_taken) begin
            FU_mul_finish_reg <= 1'b0;
        end
    end

    // CDB output
    assign cdb_request = FU_mul_finish | FU_mul_finish_reg;
    assign cdb_out = {`FU_MUL_TAG, FU_poi, FU_mul_finish_reg ? mul_out_reg : mul_out};

    // Track result taken for RS
    always @(negedge clk or posedge rst) begin
        if (rst) begin
            rs1_FU_result_taken <= 1'b0;
            rs2_FU_result_taken <= 1'b0;
            rs3_FU_result_taken <= 1'b0;
        end else begin
            rs1_FU_result_taken <= cdb[`CDB_RS_FIELD] == 3'b100 && FU_result_taken;
            rs2_FU_result_taken <= cdb[`CDB_RS_FIELD] == 3'b010 && FU_result_taken;
            rs3_FU_result_taken <= cdb[`CDB_RS_FIELD] == 3'b001 && FU_result_taken;
        end
    end

endmodule