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
    input clk, rst, issue, flush,
    input cdb_bus_t cdb_i,                   // Common Data Bus
    input[3:0]  ALUControl_in,               // operation code
    input tagged_data_t op_a_i,              // Operand A (tag + data)
    input tagged_data_t op_b_i,              // Operand B (tag + data)

    output logic all_busy,                   // all RS full?
    output logic cdb_request,                // ready to broadcast?
    output tagged_data_t cdb_data_o,         // what to broadcast (tag + data)
    output logic[`NUM_SRBITS-1:0] issue_tag  // which RS got an issue slot
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

wire FU_result_taken = cdb_i.valid && (cdb_i.tag[7:3] == `FU_ALU_TAG); // Adjusted for cdb_bus_t

//-----------------------------------------------------------------------------
// Instantiate three generic RS lines for ALU.
// Each tracks its own tag/data readiness and issues into this unit.
// RS_ALU_line ports need to be adapted from new struct inputs
//-----------------------------------------------------------------------------
RS_ALU_line rs1_alu(
    .clk(clk), .rst(rst), .issue(rs1_issue), .flush(flush),
    .FU_result_taken(rs1_FU_result_taken),
    // Pass CDB fields to RS_ALU_line (assuming it expects individual fields or specific bit slices)
    // This requires RS_ALU_line to be updated or to use an adapter if it expects the old packed cdb.
    // For now, we assume RS_ALU_line.v is not changed yet, so we pass original formatted signals.
    // This implies we need to pass cdb_i.valid, cdb_i.tag, cdb_i.data and let RS_ALU_line internally use define.vh macros
    // Or, more directly, pass the specific bits RS_ALU_line needs based on its internal logic.
    // Let's assume RS_ALU_line is not yet refactored and expects the old packed CDB format for now.
    // This means we need to pass the packed version to it.
    .cdb({cdb_i.valid, cdb_i.tag, cdb_i.data}),
    .q1_in(op_a_i.tag), .q2_in(op_b_i.tag),
    .v1_in(op_a_i.val), .v2_in(op_b_i.val),
    .data_ready(rs1_data_ready), .busy(rs1_busy),
    .v1(rs1_v1), .v2(rs1_v2),
    .ALUControl_in(ALUControl_in), .ALUControl(rs1_ALUControl)
);
RS_ALU_line rs2_alu(
    .clk(clk),.rst(rst),.issue(rs2_issue),.flush(flush),.FU_result_taken(rs2_FU_result_taken),
    .cdb({cdb_i.valid, cdb_i.tag, cdb_i.data}),
    .q1_in(op_a_i.tag), .q2_in(op_b_i.tag), .v1_in(op_a_i.val), .v2_in(op_b_i.val),
    .data_ready(rs2_data_ready),.busy(rs2_busy),.v1(rs2_v1),.v2(rs2_v2),
    .ALUControl_in(ALUControl_in),.ALUControl(rs2_ALUControl)
);

RS_ALU_line rs3_alu(
    .clk(clk),.rst(rst),.issue(rs3_issue),.flush(flush),.FU_result_taken(rs3_FU_result_taken),
    .cdb({cdb_i.valid, cdb_i.tag, cdb_i.data}),
    .q1_in(op_a_i.tag), .q2_in(op_b_i.tag), .v1_in(op_a_i.val), .v2_in(op_b_i.val),
    .data_ready(rs3_data_ready),.busy(rs3_busy),.v1(rs3_v1),.v2(rs3_v2),
    .ALUControl_in(ALUControl_in),.ALUControl(rs3_ALUControl)
);

//-----------------------------------------------------------------------------
// Simple round-robin pointer (FU_poi) to round-robin among ready RS.
// ALUA/ALUB hold selected operands, FU_ALUControl selects op.
//-----------------------------------------------------------------------------
reg FU_ALU_EN;
reg[2:0] FU_poi; // Represents which RS is chosen (100 for RS1, 010 for RS2, 001 for RS3)
reg[3:0]  FU_ALUControl;
reg[31:0] ALUA, ALUB;
wire FU_ALU_finish;
reg  FU_ALU_finish_reg;
wire[31:0] ALUout;
reg[31:0] ALUout_reg;

always @(negedge clk or posedge rst) begin
    if (rst) begin
        FU_poi         <= 3'b000;
        ALUA           <= 32'b0; ALUB <= 32'b0;
        FU_ALUControl  <= 4'b0;
        FU_ALU_EN      <= 1'b0;
    end else if (FU_result_taken || FU_poi == 3'b000) begin // If result taken or no one is selected
        // pick next ready RS that hasn't been served
        if (rs1_data_ready && (FU_poi == 3'b000 || FU_poi != 3'b100)) begin // Check if RS1 ready and not already chosen or if starting
            FU_poi        <= 3'b100; // RS1
            ALUA          <= rs1_v1; ALUB <= rs1_v2;
            FU_ALUControl <= rs1_ALUControl;
            FU_ALU_EN     <= 1'b1;
        end
        else if (rs2_data_ready && (FU_poi == 3'b000 || FU_poi != 3'b010)) begin // Check if RS2 ready and not already chosen or if starting
            FU_poi        <= 3'b010; // RS2
            ALUA          <= rs2_v1; ALUB <= rs2_v2;
            FU_ALUControl <= rs2_ALUControl;
            FU_ALU_EN     <= 1'b1;
        end
        else if (rs3_data_ready && (FU_poi == 3'b000 || FU_poi != 3'b001)) begin // Check if RS3 ready and not already chosen or if starting
            FU_poi        <= 3'b001; // RS3
            ALUA          <= rs3_v1; ALUB <= rs3_v2;
            FU_ALUControl <= rs3_ALUControl;
            FU_ALU_EN     <= 1'b1;
        end
        else begin
            // nothing ready or already served this cycle in a way that allows new selection
            FU_poi    <= 3'b000; // Reset if no one new is picked and previous was taken or none was selected
            FU_ALU_EN <= 1'b0;
        end
    end // Removed 'else FU_ALU_EN <= 0;' to allow FU_ALU_EN to persist if FU is multi-cycle and not taken.
      // FU_ALU_EN is set to 0 if nothing is picked or if FU_result_taken.
      // If FU_ALU_EN is already 1 (meaning FU is busy) and result not taken, it should remain 1.
      // The condition `FU_result_taken || FU_poi == 3'b000` handles when to re-evaluate.
      // If FU is busy (FU_ALU_EN=1) and result not taken, this block is skipped, FU_ALU_EN remains 1.
      // If FU_ALU_finish occurs, then FU_ALU_EN might be reset in the next cycle by FU_result_taken or by not finding a new ready RS.
    else if (FU_ALU_EN && !FU_ALU_finish) begin
        // FU is busy, keep it enabled
    end else begin
        FU_ALU_EN <= 1'b0; // Default to off if no other condition met
    end
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
        FU_ALU_finish_reg <= 1'b0;
        ALUout_reg        <= 32'b0;
    end else if (FU_ALU_finish && ~FU_result_taken) begin // Check FU_result_taken here?
        FU_ALU_finish_reg <= 1'b1;
        ALUout_reg        <= ALUout;
    end else if (FU_result_taken || (FU_ALU_finish_reg && !cdb_request)) begin // If taken, or if it was latched but not requested (e.g. cdb busy)
        FU_ALU_finish_reg <= 1'b0; // Clear if taken or if cdb grant is missed for some reason
    end
end

//-----------------------------------------------------------------------------
// Broadcast request + payload onto CDB
//-----------------------------------------------------------------------------
assign cdb_request = FU_ALU_finish | FU_ALU_finish_reg;
// Constructing cdb_data_o from internal signals
assign cdb_data_o.tag   = {`FU_ALU_TAG, FU_poi}; // FU_ALU_TAG (5 bits), FU_poi (3 bits) = 8 bit tag
assign cdb_data_o.val = (FU_ALU_finish_reg ? ALUout_reg : ALUout);


//-----------------------------------------------------------------------------
// Tell each RS when its result has been taken from the CDB
//-----------------------------------------------------------------------------
always @(negedge clk or posedge rst) begin
	if(rst) begin
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