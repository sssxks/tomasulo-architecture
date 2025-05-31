`timescale 1ns / 1ps

// Reorder Buffer (ROB) for speculative commit
// TODO: implement full ROB entry table with allocate, writeback, commit, and recovery

module ROB(
    input clk,
    input rst,
    // Snapshot tags bus for RAT checkpoint
    input [31*`NUM_SRBITS-1:0] alloc_tags_bus,
    // Allocation at issue
    input alloc,
    input [31:0] alloc_pc,
    input alloc_pred_taken,
    input [31:0] alloc_pred_target,
    input [7:0]  alloc_dest_tag,
    // Writeback from CDB: valid + tag + data
    input [40:0] cdb,
    // Commit outputs
    output commit_valid,
    output commit_is_branch,
    output commit_mispredict,
    output commit_actual_taken,
    output [31:0] commit_actual_target,
    output [7:0]  commit_dest_tag,
    output [31:0] commit_value,
    // Restore signals for RAT
    output [3:0] commit_index,
    output [31*`NUM_SRBITS-1:0] commit_tags_bus
);

// ROB parameters and storage
localparam N = 16;
reg [3:0] head, tail;
reg [N-1:0] entry_valid, entry_ready;
reg [31:0] entry_pc      [0:N-1];
reg entry_is_branch       [0:N-1];
reg entry_pred_taken      [0:N-1];
reg [31:0] entry_pred_target [0:N-1];
reg [7:0]  entry_dest_tag [0:N-1];
reg [31:0] entry_value   [0:N-1];
reg entry_actual_taken   [0:N-1];
// RAT snapshot tags
reg [31*`NUM_SRBITS-1:0] entry_tags [0:N-1];

// Commit output registers
reg commit_valid_r;
reg commit_is_branch_r;
reg commit_mispredict_r;
reg [3:0] commit_index_r;
reg commit_actual_taken_r;
reg [31:0] commit_actual_target_r;
reg [7:0]  commit_dest_tag_r;
reg [31:0] commit_value_r;
reg [31*`NUM_SRBITS-1:0] commit_tags_r;

integer i;
always @(posedge clk) begin
    if (rst) begin
        head <= 0; tail <= 0;
        entry_valid <= 0;
        entry_ready <= 0;
        commit_valid_r <= 0;
    end else begin
        // CDB writeback: update matching entry
        if (cdb[40]) begin
            for (i = 0; i < N; i = i + 1) begin
                if (entry_valid[i] && entry_dest_tag[i] == cdb[39:32]) begin
                    entry_value[i] <= cdb[31:0];
                    entry_ready[i] <= 1'b1;
                    // For branch, actual_taken from LSB of value or custom logic
                    entry_actual_taken[i] <= entry_is_branch[i] ? (cdb[31:0] != entry_pred_target[i]) : 1'b0;
                end
            end
        end
        // Allocation on issue
        if (alloc) begin
            entry_valid[tail] <= 1'b1;
            entry_ready[tail] <= 1'b0;
            entry_pc[tail] <= alloc_pc;
            entry_is_branch[tail] <= alloc_pred_taken;
            entry_pred_taken[tail] <= alloc_pred_taken;
            entry_pred_target[tail] <= alloc_pred_target;
            entry_dest_tag[tail] <= alloc_dest_tag;
            entry_tags[tail] <= alloc_tags_bus;
            tail <= tail + 1;
        end
        // Commit oldest entry
        if (entry_valid[head] && entry_ready[head]) begin
            commit_valid_r <= 1'b1;
            commit_is_branch_r <= entry_is_branch[head];
            commit_mispredict_r <= entry_is_branch[head] && (entry_pred_taken[head] != entry_actual_taken[head]);
            commit_actual_taken_r <= entry_actual_taken[head];
            commit_actual_target_r <= entry_value[head];
            commit_dest_tag_r <= entry_dest_tag[head];
            commit_value_r <= entry_value[head];
            commit_index_r <= head;
            commit_tags_r <= entry_tags[head];
            entry_valid[head] <= 1'b0;
            entry_ready[head] <= 1'b0;
            head <= head + 1;
        end else begin
            commit_valid_r <= 1'b0;
        end
    end
end

// Drive outputs
assign commit_valid = commit_valid_r;
assign commit_is_branch = commit_is_branch_r;
assign commit_mispredict = commit_mispredict_r;
assign commit_actual_taken = commit_actual_taken_r;
assign commit_actual_target = commit_actual_target_r;
assign commit_dest_tag = commit_dest_tag_r;
assign commit_value = commit_value_r;
assign commit_index = commit_index_r;
assign commit_tags_bus = commit_tags_r;

endmodule
