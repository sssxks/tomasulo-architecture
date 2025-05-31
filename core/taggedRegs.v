`timescale 1ns / 1ps

`include "define.vh"

//-----------------------------------------------------------------------------
// Module: taggedRegs
//  - Tracks register file values and reservation station tags.
//  - Provides read ports combining tag and data, with x0 hardwired to zero.
//  - Updates tags on FU write-back and clears/completes on CDB commit or jump.
//----------------------------------------------------------------------------- 
module taggedRegs(
	input clk, rst, FU_regWrite, ujump_wb,
	input[4:0] raddr_A, raddr_B, waddr,
	input[`NUM_SRBITS-1:0] w_tag,
	input[`NUM_CDBBITS-1:0] cdb,
	input[31:0] PC,
	output[`NUM_CDBBITS-2:0] rdata_A, rdata_B,

	input[4:0] Debug_addr,
	output[39:0] Debug_regs
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
//  - rdata_A/B: concatenation of tag and register data, zero for x0.
//----------------------------------------------------------------------------- 
	assign rdata_A = (raddr_A == 0) ? 40'b0 : {tags[raddr_A], register[raddr_A]};
	assign rdata_B = (raddr_B == 0) ? 40'b0 : {tags[raddr_B], register[raddr_B]};

	wire addr_match[1:31];
	wire cdb_match[1:31];

//-----------------------------------------------------------------------------
// Write-back matching logic
//  - addr_match: when waddr equals register index. 
//                (no need to check waddr==0, as implied by FU_regWrite)
//  - cdb_match: when CDB broadcasts value, we check if we are the receiver
//               by comparing the tag
//----------------------------------------------------------------------------- 
	genvar i;
	generate
		for(i=1;i<32;i=i+1) begin
			assign addr_match[i] = waddr == i;
			assign cdb_match[i] = (cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == tags[i]);

//-----------------------------------------------------------------------------
// Per-register update logic
//  - Tag update: on FU_regWrite set new tag, on CDB match or ujump clear.
//  - Value update: on CDB match write data, on ujump write PC+4.
//----------------------------------------------------------------------------- 
			always @(posedge clk or posedge rst) begin
				if(rst) tags[i] <= 8'b0;
				else if(FU_regWrite & addr_match[i]) tags[i] <= w_tag;
				else if(cdb_match[i] | ujump_wb & addr_match[i]) tags[i] <= 8'b0;
			end

			always @(posedge clk or posedge rst) begin
				if(rst) register[i] <= 32'b0;
				else if(ujump_wb & addr_match[i]) register[i] <= PC + 4;
				else if(cdb_match[i]) register[i] <= cdb[`CDB_DATA_FIELD];
			end
		end
	endgenerate

//-----------------------------------------------------------------------------
// Debug port for introspection
//  - Debug_regs: outputs tag+value for Debug_addr, zero for x0.
//----------------------------------------------------------------------------- 
	assign Debug_regs = (Debug_addr == 0) ? 40'b0 : {tags[Debug_addr], register[Debug_addr]};
endmodule