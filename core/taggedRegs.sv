`timescale 1ns / 1ps
`include "define.vh"

//-----------------------------------------------------------------------------
// Module: taggedRegs
//  - Tracks register file values and reservation station tags.
//  - Provides read ports combining tag and data, with x0 hardwired to zero.
//  - Updates tags on FU write-back and clears/completes on CDB commit or jump.
//----------------------------------------------------------------------------- 
module taggedRegs(
	input logic clk,
    input logic rst,
    input logic FU_regWrite,
    input logic ujump_wb,
	input logic[4:0] raddr_A, raddr_B, waddr,
	input logic[`NUM_SRBITS-1:0] w_tag,
	input cdb_bus_t cdb_i, // CHANGED
	input logic[31:0] PC,
	output tagged_data_t rdata_A_o, // CHANGED
    output tagged_data_t rdata_B_o, // CHANGED

	// Snapshot & restore ports
	input logic restore,
	input logic[`NUM_SRBITS*31-1:0] restore_tags_bus,
	input logic[3:0] restore_index, // Assuming this index is for ROB, not directly used here for specific tag array index
	output logic[`NUM_SRBITS*31-1:0] all_tags_bus,

	input logic[4:0] Debug_addr,
	output logic[39:0] Debug_regs // Kept as [39:0] for direct compatibility if Test_signal expects it
);
//-----------------------------------------------------------------------------
// Register file and tag storage
//  - register[1:31]: holds 32-bit architectural register values.
//  - tags[1:31]: holds pending source readiness tags (`NUM_SRBITS` bits).
//----------------------------------------------------------------------------- 
	reg [31:0] register [1:31];
	reg[`NUM_SRBITS-1:0] tags [1:31];

//-----------------------------------------------------------------------------
// Read ports
//  - rdata_A_o/B_o: concatenation of tag and register data, zero for x0.
//----------------------------------------------------------------------------- 
	assign rdata_A_o.val = (raddr_A == 0) ? 32'b0 : register[raddr_A];
	assign rdata_A_o.tag = (raddr_A == 0) ? `NUM_SRBITS'b0 : tags[raddr_A];

	assign rdata_B_o.val = (raddr_B == 0) ? 32'b0 : register[raddr_B];
	assign rdata_B_o.tag = (raddr_B == 0) ? `NUM_SRBITS'b0 : tags[raddr_B];

	wire addr_match[1:31];
	wire cdb_match[1:31];

//-----------------------------------------------------------------------------
// Pack current tags into a wide bus for ROB allocation
generate
    genvar gi;
    for (gi = 1; gi < 32; gi = gi + 1) begin : pack_tags
        assign all_tags_bus[(gi*`NUM_SRBITS-1) -: `NUM_SRBITS] = tags[gi];
    end
endgenerate

//-----------------------------------------------------------------------------
// Write-back matching logic
//  - addr_match: when waddr equals register index. 
//  - cdb_match: when CDB broadcasts value, we check if we are the receiver
//               by comparing the tag
//----------------------------------------------------------------------------- 
	genvar i;
	generate
		for(i=1;i<32;i=i+1) begin
			assign addr_match[i] = (waddr == i); // Ensure waddr is not 0 if FU_regWrite or ujump_wb is asserted for waddr
			assign cdb_match[i] = (cdb_i.valid && cdb_i.tag == tags[i] && tags[i] != `NUM_SRBITS'b0); // Match only if tag is valid

//-----------------------------------------------------------------------------
// Per-register update logic
//  - Tag update: on FU_regWrite set new tag, on CDB match or ujump clear.
//  - Value update: on CDB match write data, on ujump write PC+4.
//----------------------------------------------------------------------------- 
			always @(posedge clk or posedge rst) begin
				if(rst) tags[i] <= `NUM_SRBITS'b0;
				else if(FU_regWrite && addr_match[i] && waddr != 0) tags[i] <= w_tag; // Only write if waddr != 0
				else if(cdb_match[i]) tags[i] <= `NUM_SRBITS'b0; // Clear tag on CDB match
                else if(ujump_wb && addr_match[i] && waddr != 0) tags[i] <= `NUM_SRBITS'b0; // JAL/JALR writes PC+4, tag becomes 0
			end

			always @(posedge clk or posedge rst) begin
				if(rst) register[i] <= 32'b0;
				else if(ujump_wb && addr_match[i] && waddr != 0) register[i] <= PC + 4; // JAL/JALR
				else if(cdb_match[i]) register[i] <= cdb_i.data; // Write data from CDB
			end
		end
	endgenerate

//-----------------------------------------------------------------------------
// Restore snapshot logic
// This logic assumes restore_index is not used to select a specific bank of tags,
// but rather that restore_tags_bus contains the full set of 31 tags to restore.
integer ri; // SystemVerilog allows integer for loop variables in generate, but for always blocks, genvar is typical. Here, it's an always block.
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // tags are reset by their individual always blocks above
    end else if (restore) begin
        for (ri = 1; ri < 32; ri = ri + 1) begin
            tags[ri] <= restore_tags_bus[(ri*`NUM_SRBITS-1) -: `NUM_SRBITS];
        end
    end
end

//-----------------------------------------------------------------------------
// Debug port for introspection
//  - Debug_regs: outputs tag+value for Debug_addr, zero for x0.
//----------------------------------------------------------------------------- 
	assign Debug_regs = (Debug_addr == 0) ? 40'b0 : {tags[Debug_addr], register[Debug_addr]};
endmodule