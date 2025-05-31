`timescale 1ns / 1ps
`include "define.vh"

//-----------------------------------------------------------------------------
// Module: unit_ALU
//  - Three RS_ALU_line instances hold pending ALU ops.
//  - Issue logic picks the first free RS.
//  - A small controller picks one ready RS at a time, fires FU_ALU,
//    captures its result, and then announces it on the CDB.
//-----------------------------------------------------------------------------
module unit_ALU(
    input clk, rst, issue,
    input[`NUM_CDBBITS-1:0] cdb,             // Common Data Bus (tag + data)
    input[3:0]  ALUControl_in,               // operation code
    input[7:0]  q1_in, q2_in,                // tags of operands
    input[31:0] v1_in, v2_in,                // literal operands

    output all_busy,                         // all RS full?
    output cdb_request,                      // ready to broadcast?
    output[`NUM_CDBBITS-2:0] cdb_out,        // what to broadcast
    output[`NUM_SRBITS-1:0] issue_tag        // which RS got an issue slot
);

//-----------------------------------------------------------------------------
// Reservation‐station bookkeeping flags:
//    rsX_FU_result_taken: tells RS_X that its result was consumed.
//-----------------------------------------------------------------------------
reg rs1_FU_result_taken, rs2_FU_result_taken, rs3_FU_result_taken;
wire rs1_data_ready, rs2_data_ready, rs3_data_ready;
wire rs1_busy, rs2_busy, rs3_busy;
wire[3:0]  rs1_ALUControl, rs2_ALUControl, rs3_ALUControl;
wire[31:0] rs1_v1, rs1_v2, rs2_v1, rs2_v2, rs3_v1, rs3_v2;

//-----------------------------------------------------------------------------
// Issue logic: pick first free RS in order 1→2→3
//-----------------------------------------------------------------------------
assign all_busy    = rs1_busy & rs2_busy & rs3_busy;
wire rs1_issue     = issue & ~rs1_busy;
wire rs2_issue     = issue &  rs1_busy & ~rs2_busy;
wire rs3_issue     = issue &  rs1_busy &  rs2_busy & ~rs3_busy;
assign issue_tag   = {`FU_ALU_TAG, rs1_issue, rs2_issue, rs3_issue}
                     & {`NUM_SRBITS{issue}};

wire FU_result_taken = cdb[`CDB_ON_FIELD]
                     && cdb[`CDB_FU_FIELD] == `FU_ALU_TAG;

//-----------------------------------------------------------------------------
// Instantiate three generic RS lines for ALU.
// Each tracks its own tag/data readiness and issues into this unit.
//-----------------------------------------------------------------------------
RS_ALU_line rs1_alu(
    .clk(clk), .rst(rst), .issue(rs1_issue),
    .FU_result_taken(rs1_FU_result_taken),
    .cdb(cdb),
    .q1_in(q1_in), .q2_in(q2_in),
    .v1_in(v1_in), .v2_in(v2_in),
    .data_ready(rs1_data_ready), .busy(rs1_busy),
    .v1(rs1_v1), .v2(rs1_v2),
    .ALUControl_in(ALUControl_in), .ALUControl(rs1_ALUControl)
);
RS_ALU_line rs2_alu(.clk(clk),.rst(rst),.issue(rs2_issue),.FU_result_taken(rs2_FU_result_taken),
	.cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),.v1_in(v1_in),.v2_in(v2_in),
	.data_ready(rs2_data_ready),.busy(rs2_busy),.v1(rs2_v1),.v2(rs2_v2),
	.ALUControl_in(ALUControl_in),.ALUControl(rs2_ALUControl));

RS_ALU_line rs3_alu(.clk(clk),.rst(rst),.issue(rs3_issue),.FU_result_taken(rs3_FU_result_taken),
	.cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),.v1_in(v1_in),.v2_in(v2_in),
	.data_ready(rs3_data_ready),.busy(rs3_busy),.v1(rs3_v1),.v2(rs3_v2),
	.ALUControl_in(ALUControl_in),.ALUControl(rs3_ALUControl));

//-----------------------------------------------------------------------------
// Simple round-robin pointer (FU_poi) to round-robin among ready RS.
// ALUA/ALUB hold selected operands, FU_ALUControl selects op.
//-----------------------------------------------------------------------------
reg FU_ALU_EN;
reg[2:0] FU_poi;
reg[3:0]  FU_ALUControl;
reg[31:0] ALUA, ALUB;
wire FU_ALU_finish;
reg  FU_ALU_finish_reg;
wire[31:0] ALUout;
reg[31:0] ALUout_reg;

always @(negedge clk or posedge rst) begin
    if (rst) begin
        FU_poi         <= 3'b000;
        ALUA           <= 0; ALUB <= 0;
        FU_ALUControl  <= 0;
        FU_ALU_EN      <= 0;
    end else if (FU_result_taken || FU_poi == 3'b000) begin
        // pick next ready RS that hasn't been served
        if (rs1_data_ready && ~FU_poi[2]) begin
            FU_poi        <= 3'b100;
            ALUA          <= rs1_v1; ALUB <= rs1_v2;
            FU_ALUControl <= rs1_ALUControl;
            FU_ALU_EN     <= 1;
        end
        else if (rs2_data_ready && ~FU_poi[1]) begin
            FU_poi        <= 3'b010;
            ALUA          <= rs2_v1; ALUB <= rs2_v2;
            FU_ALUControl <= rs2_ALUControl;
            FU_ALU_EN     <= 1;
        end
        else if (rs3_data_ready && ~FU_poi[0]) begin
            FU_poi        <= 3'b001;
            ALUA          <= rs3_v1; ALUB <= rs3_v2;
            FU_ALUControl <= rs3_ALUControl;
            FU_ALU_EN     <= 1;
        end
        else begin
            // nothing ready or already served
            FU_poi    <= 3'b000;
            FU_ALU_EN <= 0;
        end
    end
    else FU_ALU_EN <= 0;
end

//-----------------------------------------------------------------------------
// Core ALU instantiation
//-----------------------------------------------------------------------------
FU_ALU alu(
    .clk(clk), .EN(FU_ALU_EN), .finish(FU_ALU_finish),
    .ALUControl(FU_ALUControl), .ALUA(ALUA), .ALUB(ALUB),
    .res(ALUout), .zero(), .overflow()
);

//-----------------------------------------------------------------------------
// Latch finish signal so we can hold it until the CDB is granted.
//-----------------------------------------------------------------------------
always @(negedge clk or posedge rst) begin
    if (rst) begin
        FU_ALU_finish_reg <= 0;
        ALUout_reg        <= 0;
    end else if (FU_ALU_finish && ~FU_result_taken) begin
        FU_ALU_finish_reg <= 1;
        ALUout_reg        <= ALUout;
    end else if (FU_result_taken) begin
        FU_ALU_finish_reg <= 0;
    end
end

//-----------------------------------------------------------------------------
// Broadcast request + payload onto CDB
//-----------------------------------------------------------------------------
assign cdb_request = FU_ALU_finish | FU_ALU_finish_reg;
assign cdb_out     = { `FU_ALU_TAG,   // FU type
                       FU_poi,         // which RS
                       (FU_ALU_finish_reg ? ALUout_reg : ALUout)
                     };

//-----------------------------------------------------------------------------
// Tell each RS when its result has been taken from the CDB
//-----------------------------------------------------------------------------
always @(negedge clk or posedge rst) begin
	if(rst) begin
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