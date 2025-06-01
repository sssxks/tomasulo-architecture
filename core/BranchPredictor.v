`timescale 1ns / 1ps

// Stub branch predictor with BTB interface
module BranchPredictor(
    input clk,
    input rst,
    input [31:0] pc,
    input update,
    input update_taken,
    input [31:0] update_target,
    output taken,
    output [31:0] target
);

// Predictor tables: 128 entries indexed by PC[8:2]
localparam P_DEPTH = 128;
reg [1:0] ptable [0:P_DEPTH-1];
// Branch Target Buffer: store actual targets
reg [31:0] btb [0:P_DEPTH-1];
// Index from PC bits
wire [6:0] idx = pc[8:2];
// Prediction: taken if MSB of 2-bit counter
assign taken = ptable[idx][1];
// Select predicted target from BTB or PC+4
assign target = taken ? btb[idx] : pc + 32'd4;

integer i;
// Initialize and update counters
always @(posedge clk) begin
    if (rst) begin
        for (i = 0; i < P_DEPTH; i = i + 1) begin
            ptable[i] <= 2'b01; // weakly not taken
            btb[i]    <= 32'd0;
        end
    end else if (update) begin
        if (update_taken) begin
            // increment saturating
            if (ptable[idx] != 2'b11)
                ptable[idx] <= ptable[idx] + 1;
            // update BTB on taken
            btb[idx] <= update_target;
        end else begin
            // decrement saturating
            if (ptable[idx] != 2'b00)
                ptable[idx] <= ptable[idx] - 1;
        end
    end
end

endmodule
