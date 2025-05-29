`timescale 1ns / 1ps

`include "define.vh"

module unit_load_store(
	input clk, rst,
	input[`NUM_CDBBITS-1:0]cdb,
	output cdb_request,
	output[`NUM_CDBBITS-2:0] cdb_out,

	input[31:0] ls_addr_in,
	input[2:0] ls_u_b_h_w_in,

	input load_issue,
	output load_all_busy,
	output[`NUM_SRBITS-1:0] load_issue_tag,

	input store_issue,
	input[7:0]  store_q_data_in,
	input[31:0] store_data_in,
	output store_all_busy,
	output store_conflict_stall
);

	reg[2:0] load_finish;
	reg[2:0] load_from_store [2:0];
	reg[31:0] load_data [2:0];

	wire load_result_taken = cdb[`CDB_ON_FIELD] && cdb[`CDB_FU_FIELD] == `FU_LOAD_TAG;

	wire rs1_load_busy, rs2_load_busy, rs3_load_busy;
	wire rs1_store_busy, rs2_store_busy, rs3_store_busy;
	wire rs1_store_data_ready, rs2_store_data_ready, rs3_store_data_ready;
	reg rs1_load_result_taken, rs2_load_result_taken, rs3_load_result_taken;
	wire rs1_store_result_taken, rs2_store_result_taken, rs3_store_result_taken;



	wire[31:0] rs1_load_addr, rs2_load_addr, rs3_load_addr;
	wire[2:0] rs1_load_u_b_h_w, rs2_load_u_b_h_w, rs3_load_u_b_h_w;

	wire[31:0] rs1_store_addr, rs2_store_addr, rs3_store_addr;
	wire[31:0] rs1_store_data, rs2_store_data, rs3_store_data;
	wire[2:0] rs1_store_u_b_h_w, rs2_store_u_b_h_w, rs3_store_u_b_h_w;

	wire rs1_load_issue = load_issue & ~rs1_load_busy;
	wire rs2_load_issue = load_issue & rs1_load_busy & ~rs2_load_busy;
	wire rs3_load_issue = load_issue & rs1_load_busy & rs2_load_busy & ~rs3_load_busy;
	assign load_all_busy = rs1_load_busy & rs2_load_busy & rs3_load_busy;
	assign load_issue_tag = {`FU_LOAD_TAG, rs1_load_issue, rs2_load_issue, rs3_load_issue} &{`NUM_SRBITS{load_issue}};

	RS_load_line rs1_load(.clk(clk),.rst(rst),.issue(rs1_load_issue),.cdb(cdb),
		.FU_result_taken(rs1_load_result_taken),.addr_in(ls_addr_in),.mem_u_b_h_w_in(ls_u_b_h_w_in),
		.busy(rs1_load_busy),.addr(rs1_load_addr),.mem_u_b_h_w(rs1_load_u_b_h_w));

	RS_load_line rs2_load(.clk(clk),.rst(rst),.issue(rs2_load_issue),.cdb(cdb),
		.FU_result_taken(rs2_load_result_taken),.addr_in(ls_addr_in),.mem_u_b_h_w_in(ls_u_b_h_w_in),
		.busy(rs2_load_busy),.addr(rs2_load_addr),.mem_u_b_h_w(rs2_load_u_b_h_w));

	RS_load_line rs3_load(.clk(clk),.rst(rst),.issue(rs3_load_issue),.cdb(cdb),
		.FU_result_taken(rs3_load_result_taken),.addr_in(ls_addr_in),.mem_u_b_h_w_in(ls_u_b_h_w_in),
		.busy(rs3_load_busy),.addr(rs3_load_addr),.mem_u_b_h_w(rs3_load_u_b_h_w));



	wire rs1_s_addr_match = rs1_store_busy && ls_addr_in == rs1_store_addr;
	wire rs2_s_addr_match = rs2_store_busy && ls_addr_in == rs2_store_addr;
	wire rs3_s_addr_match = rs3_store_busy && ls_addr_in == rs3_store_addr;
	wire no_s_addr_match = ~rs1_s_addr_match & ~rs2_s_addr_match & ~rs3_s_addr_match;

	wire rs1_store_issue = store_issue & no_s_addr_match;
	wire rs2_store_issue = store_issue & no_s_addr_match & rs1_store_busy ;
	wire rs3_store_issue = store_issue & no_s_addr_match & rs1_store_busy & rs2_store_busy;
	assign store_all_busy = rs1_store_busy & rs2_store_busy & rs3_store_busy;
	assign store_conflict_stall = rs1_s_addr_match | rs2_s_addr_match | rs3_s_addr_match;

	RS_store_line rs1_store(.clk(clk),.rst(rst),.issue(rs1_store_issue),.result_taken(rs1_store_result_taken),
		.cdb(cdb),.q_data_in(store_q_data_in),.addr_in(ls_addr_in),.data_in(store_data_in),
		.mem_u_b_h_w_in(ls_u_b_h_w_in),.data_ready(rs1_store_data_ready),
		.busy(rs1_store_busy),.addr(rs1_store_addr),.data(rs1_store_data),.mem_u_b_h_w(rs1_store_u_b_h_w));

	RS_store_line rs2_store(.clk(clk),.rst(rst),.issue(rs2_store_issue),.result_taken(rs2_store_result_taken),
		.cdb(cdb),.q_data_in(store_q_data_in),.addr_in(ls_addr_in),.data_in(store_data_in),
		.mem_u_b_h_w_in(ls_u_b_h_w_in),.data_ready(rs2_store_data_ready),
		.busy(rs2_store_busy),.addr(rs2_store_addr),.data(rs2_store_data),.mem_u_b_h_w(rs2_store_u_b_h_w));

	RS_store_line rs3_store(.clk(clk),.rst(rst),.issue(rs3_store_issue),.result_taken(rs3_store_result_taken),
		.cdb(cdb),.q_data_in(store_q_data_in),.addr_in(ls_addr_in),.data_in(store_data_in),
		.mem_u_b_h_w_in(ls_u_b_h_w_in),.data_ready(rs3_store_data_ready),
		.busy(rs3_store_busy),.addr(rs3_store_addr),.data(rs3_store_data),.mem_u_b_h_w(rs3_store_u_b_h_w));



	wire rs1_load_out = rs1_load_busy & ~load_finish[2] & ~|load_from_store[2];
	wire rs2_load_out = ~rs1_load_out & rs2_load_busy & ~load_finish[1] & ~|load_from_store[1];
	wire rs3_load_out = ~rs2_load_out & rs3_load_busy & ~load_finish[0] & ~|load_from_store[0];
	wire rs1_store_out = ~rs3_load_out & rs1_store_data_ready;
	wire rs2_store_out = ~rs1_store_out & rs2_store_data_ready;
	wire rs3_store_out = ~rs2_store_out & rs3_store_data_ready;

	assign rs1_store_result_taken = rs1_store_out;
	assign rs2_store_result_taken = rs2_store_out;
	assign rs3_store_result_taken = rs3_store_out;



	wire rs1_load_on_cdb = load_finish[2];
	wire rs2_load_on_cdb = ~load_finish[2] & load_finish[1];
	wire rs3_load_on_cdb = ~load_finish[2] & ~load_finish[1] & load_finish[0];
	assign cdb_request = |load_finish;
	assign cdb_out = {`FU_LOAD_TAG, rs1_load_on_cdb, rs2_load_on_cdb, rs3_load_on_cdb,
					rs1_load_on_cdb ? load_data[2] : rs2_load_on_cdb ? load_data[1] : load_data[0]};

	wire[2:0]mem_bhw;
	wire[31:0] mem_addr, mem_store_data, mem_load_data;
	wire mem_w = rs1_store_out | rs2_store_out | rs3_store_out;

	tristate #(32) ts_addr1(.dout(mem_addr),.din(rs1_load_addr),.en(rs1_load_out));
	tristate #(32) ts_addr2(.dout(mem_addr),.din(rs2_load_addr),.en(rs2_load_out));
	tristate #(32) ts_addr3(.dout(mem_addr),.din(rs3_load_addr),.en(rs3_load_out));
	tristate #(32) ts_addr4(.dout(mem_addr),.din(rs1_store_addr),.en(rs1_store_out));
	tristate #(32) ts_addr5(.dout(mem_addr),.din(rs2_store_addr),.en(rs2_store_out));
	tristate #(32) ts_addr6(.dout(mem_addr),.din(rs3_store_addr),.en(rs3_store_out));

	tristate #(3) ts_bhw1(.dout(mem_bhw),.din(rs1_load_u_b_h_w),.en(rs1_load_out));
	tristate #(3) ts_bhw2(.dout(mem_bhw),.din(rs2_load_u_b_h_w),.en(rs2_load_out));
	tristate #(3) ts_bhw3(.dout(mem_bhw),.din(rs3_load_u_b_h_w),.en(rs3_load_out));
	tristate #(3) ts_bhw4(.dout(mem_bhw),.din(rs1_store_u_b_h_w),.en(rs1_store_out));
	tristate #(3) ts_bhw5(.dout(mem_bhw),.din(rs2_store_u_b_h_w),.en(rs2_store_out));
	tristate #(3) ts_bhw6(.dout(mem_bhw),.din(rs3_store_u_b_h_w),.en(rs3_store_out));

	tristate #(32) ts_din1(.dout(mem_store_data),.din(rs1_store_data),.en(rs1_store_out));
	tristate #(32) ts_din2(.dout(mem_store_data),.din(rs2_store_data),.en(rs2_store_out));
	tristate #(32) ts_din3(.dout(mem_store_data),.din(rs3_store_data),.en(rs3_store_out));

	RAM_B ram(.clka(clk),.addra(mem_addr),.dina(mem_store_data),.wea(mem_w),
        .douta(mem_load_data),.mem_u_b_h_w(mem_bhw));


	always@(posedge clk or posedge rst) begin
		if(rst) begin
			load_from_store[2] <= 3'b0;
			load_from_store[1] <= 3'b0;
			load_from_store[0] <= 3'b0;
		end
		else begin
			if(rs1_load_issue)
				load_from_store[2] <= {rs1_s_addr_match, rs2_s_addr_match, rs3_s_addr_match};
			else if(load_finish[2])
				load_from_store[2] <= 3'b0;

			if(rs2_load_issue)
				load_from_store[1] <= {rs1_s_addr_match, rs2_s_addr_match, rs3_s_addr_match};
			else if(load_finish[1])
				load_from_store[1] <= 3'b0;

			if(rs3_load_issue)
				load_from_store[0] <= {rs1_s_addr_match, rs2_s_addr_match, rs3_s_addr_match};
			else if(load_finish[0])
				load_from_store[0] <= 3'b0;
		end
	end

	always@(negedge clk or posedge rst) begin
		if(rst) begin
			rs1_load_result_taken <= 1'b0;
			rs2_load_result_taken <= 1'b0;
			rs3_load_result_taken <= 1'b0;
		end
		else begin
			rs1_load_result_taken <= cdb[`CDB_RS_FIELD] == 3'b100 && load_result_taken;
			rs2_load_result_taken <= cdb[`CDB_RS_FIELD] == 3'b010 && load_result_taken;
			rs3_load_result_taken <= cdb[`CDB_RS_FIELD] == 3'b001 && load_result_taken;
		end
	end

	always@(negedge clk or posedge rst) begin
		if(rst)
			load_finish <= 3'b0;
		else begin
			if(rs1_load_out |
				load_from_store[2][2] & rs1_store_data_ready |
				load_from_store[2][1] & rs2_store_data_ready |
				load_from_store[2][0] & rs3_store_data_ready)
				load_finish[2] <= 1'b1;
			else if(cdb[`CDB_RS_FIELD] == 3'b100 && load_result_taken)
				load_finish[2] <= 1'b0;

			if(rs2_load_out |
				load_from_store[1][2] & rs1_store_data_ready |
				load_from_store[1][1] & rs2_store_data_ready |
				load_from_store[1][0] & rs3_store_data_ready)
				load_finish[1] <= 1'b1;
			else if(cdb[`CDB_RS_FIELD] == 3'b010 && load_result_taken)
				load_finish[1] <= 1'b0;

			if(rs3_load_out |
				load_from_store[0][2] & rs1_store_data_ready |
				load_from_store[0][1] & rs2_store_data_ready |
				load_from_store[0][0] & rs3_store_data_ready)
				load_finish[0] <= 1'b1;
			else if(cdb[`CDB_RS_FIELD] == 3'b001 && load_result_taken)
				load_finish[0] <= 1'b0;
		end
	end

	always@(negedge clk or posedge rst) begin
		if(rst)
			load_data[2] <= 32'b0;
		else begin
			if(rs1_load_out)
				load_data[2] <= mem_load_data;
			else if(load_from_store[2][2] & rs1_store_data_ready)
				load_data[2] <= rs1_store_data;
			else if(load_from_store[2][1] & rs2_store_data_ready)
				load_data[2] <= rs2_store_data;
			else if(load_from_store[2][0] & rs2_store_data_ready)
				load_data[2] <= rs3_store_data;
		end
	end

	always@(negedge clk or posedge rst) begin
		if(rst)
			load_data[1] <= 32'b0;
		else begin
			if(rs2_load_out)
				load_data[1] <= mem_load_data;
			else if(load_from_store[1][2] & rs1_store_data_ready)
				load_data[1] <= rs1_store_data;
			else if(load_from_store[1][1] & rs2_store_data_ready)
				load_data[1] <= rs2_store_data;
			else if(load_from_store[1][0] & rs2_store_data_ready)
				load_data[1] <= rs3_store_data;
		end
	end

	always@(negedge clk or posedge rst) begin
		if(rst)
			load_data[0] <= 32'b0;
		else begin
			if(rs3_load_out)
				load_data[0] <= mem_load_data;
			else if(load_from_store[0][2] & rs1_store_data_ready)
				load_data[0] <= rs1_store_data;
			else if(load_from_store[0][1] & rs2_store_data_ready)
				load_data[0] <= rs2_store_data;
			else if(load_from_store[0][0] & rs2_store_data_ready)
				load_data[0] <= rs3_store_data;
		end
	end
endmodule