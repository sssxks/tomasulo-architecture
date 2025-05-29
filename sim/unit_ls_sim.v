`timescale 1ns / 1ps

module unit_ls_sim;

	reg clk, rst;
	wire[40:0]cdb;

	wire ls_cdb_request;
	reg ALU_cdb_request;
	wire[39:0] ls_cdb_out;
	reg[39:0] ALU_cdb_out;

	reg load_issue;
	reg[31:0] ls_addr_in;

	reg store_issue;
	reg[7:0]  store_q_data_in;
	reg[31:0] store_data_in;

	unit_load_store unit(.clk(clk),.rst(rst),.cdb(cdb),.cdb_request(ls_cdb_request),
		.cdb_out(ls_cdb_out),.load_issue(load_issue),.ls_addr_in(ls_addr_in),
		.ls_u_b_h_w_in(3'b010),.load_all_busy(),.load_issue_tag(),
		.store_issue(store_issue),.store_q_data_in(store_q_data_in),
		.store_all_busy(),.store_conflict_stall());

	common_data_bus cd_b(.clk(clk),.rst(rst),
		.ALU_cdb_request(ALU_cdb_request),.mul_cdb_request(1'b0),.div_cdb_request(1'b0),.ls_cdb_request(ls_cdb_request),
		.ALU_cdb_in(ALU_cdb_out),.mul_cdb_in(40'b0),.div_cdb_in(40'b0),.ls_cdb_in(ls_cdb_out),.cdb(cdb));


	initial begin
		clk = 1;
		rst = 1;
		load_issue = 0;
		ls_addr_in = 0;
		store_issue = 0;
		store_q_data_in = 0;
		store_data_in = 0;
		ALU_cdb_request = 0;
		ALU_cdb_out = 0;

		#2;
		rst = 0;
		load_issue = 1;

		#10;
		load_issue = 0;
		store_issue = 1;
		ls_addr_in = 4;
		store_q_data_in = 8'b10000_001;

		#10;
		store_data_in = 0;
		store_q_data_in = 8'b10000_010;

		#10;
		store_issue = 0;
		load_issue = 1;

		#10;
		ls_addr_in = 8;

		#10;
		load_issue = 0;

		#30;
		ALU_cdb_request = 1;
		ALU_cdb_out = 40'h81_0000_0050;

		#10;
		ALU_cdb_request = 0;

	end

	always #5 clk = ~clk;

endmodule