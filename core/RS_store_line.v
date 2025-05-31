`timescale 1ns / 1ps

`include "define.vh"

// Reservation station entry for store operations.
// Latches store address/data, handles data forwarding from CDB, and signals when ready.
module RS_store_line(
    input clk, rst, issue,          // clk: system clock; rst: global reset; issue: latch new store
    input result_taken,             // result_taken: free RS when store dispatched
    input [40:0] cdb,               // Common Data Bus: [valid][tag][data] for operand forwarding
    input [7:0] q_data_in,          // Tag for data operand if not available at issue
    input [31:0] addr_in, data_in,  // addr_in: store address; data_in: data operand
    input [2:0] mem_u_b_h_w_in,     // Control for memory width/type (unsigned/byte/half/word)
    output data_ready,              // High when busy and data operand tag cleared
    output reg busy,                // Indicates RS entry is occupied
    output reg [31:0] addr,         // Latched address for store
    output reg [31:0] data,         // Latched/fwd data for store
    output reg [2:0] mem_u_b_h_w    // Latched memory operation control
);

// Internal store data reservation and forwarding
reg [7:0] q_data;                 // Pending tag for data operand

wire cdb_match_q   = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q_data;            // Forward when CDB matches tag
wire init_clk_cdb_match_q = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q_data_in;   // Early forward at issue if tag matches

assign data_ready = busy && q_data == 8'b0;   // Data ready when no pending tag

// Busy control
always @(posedge clk or posedge rst) begin    // Busy flag control
    if (rst) busy <= 1'b0;                    // Clear on reset
    else if (issue) busy <= 1'b1;             // Set on issue
    else if (result_taken) busy <= 1'b0;      // Clear on dispatch
end

// Data operand latch
always @(posedge clk or posedge rst) begin    // Data operand latch/forwarding
    if (rst) begin
        q_data <= 8'b0;                       // Clear tag on reset
        data   <= 32'b0;                      // Clear data on reset
    end else if (issue) begin
        if (init_clk_cdb_match_q) begin       // Immediate data forward at issue
            q_data <= 8'b0;
            data   <= cdb[`CDB_DATA_FIELD];
        end else begin
            q_data <= q_data_in;              // Latch operand tag
            data   <= data_in;                // Latch operand data if available
        end
    end else if (cdb_match_q) begin            // Late forwarding when CDB matches
        q_data <= 8'b0;
        data   <= cdb[`CDB_DATA_FIELD];        // Latch forwarded data
    end
end

// Address and memory control latch
always @(posedge clk or posedge rst) begin    // Address and mem control latch
    if (rst) begin
        addr        <= 32'b0;                 // Clear address on reset
        mem_u_b_h_w <= 3'b0;                  // Clear control on reset
    end else if (issue) begin
        addr        <= addr_in;               // Latch address
        mem_u_b_h_w <= mem_u_b_h_w_in;        // Latch mem control
    end
end

endmodule