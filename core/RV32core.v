`timescale 1ns / 1ps

module  RV32core(
		input debug_en,  // debug enable
		input debug_step,  // debug step clock
		input [6:0] debug_addr,  // debug address
		output[39:0] debug_data,  // debug data
		input clk,  // main clock
		input rst   // synchronous reset
	);

	wire debug_clk;
	wire[39:0] debug_regs;
	reg[39:0] Test_signal;
	assign debug_data = |debug_addr[6:5] ? Test_signal : debug_regs;

	debug_clk clock(.clk(clk),.debug_en(debug_en),.debug_step(debug_step),.debug_clk(debug_clk));

	wire ls_addr_unresolved, struct_hazard, waw_hazard, normal_stall, to_jump, jump_stall;

	wire[31:0] PC_IF, next_PC_IF, PCp4_IF, PC_jump, inst_IF;

	wire ALUSrcA, ALUSrcB, FU_regWrite;
	wire ALU_issue, mul_issue, div_issue, load_issue, store_issue, ujump_issue, branch_issue;
	wire[2:0] ImmSel;
	wire[3:0] ALU_op, JUMP_op;
	wire[31:0] inst_ID, PC_ID, Imm, ls_addr;
	wire[39:0] rs1_data, rs2_data, ALUA, ALUB;

	wire ALU_cdb_request, mul_cdb_request, div_cdb_request, load_cdb_request;
	wire ALU_all_busy, mul_all_busy, div_all_busy, load_all_busy, store_all_busy, store_conflict_stall;
	wire[7:0] ALU_issue_tag, mul_issue_tag, div_issue_tag, load_issue_tag, reg_w_tag;
	wire[39:0] ALU_cdb_in, mul_cdb_in, div_cdb_in, load_cdb_in;

	wire[40:0] cdb;


	// IF
	REG32 REG_PC(.clk(debug_clk),.rst(rst),.CE(~normal_stall & ~jump_stall),.D(next_PC_IF),.Q(PC_IF));

	add_32 add_IF(.a(PC_IF),.b(32'd4),.c(PCp4_IF));

	MUX2T1_32 mux_IF(.I0(PCp4_IF),.I1(PC_jump),.s(to_jump),.o(next_PC_IF));

	ROM_D inst_rom(.a(PC_IF[8:2]),.spo(inst_IF));


	//Issue
	REG_ID reg_ID(.clk(debug_clk),.rst(rst),.EN(~normal_stall & ~jump_stall),
		.flush(to_jump & ~jump_stall),.PCOUT(PC_IF),.IR(inst_IF),

		.IR_ID(inst_ID),.PCurrent_ID(PC_ID),.valid());

	DecodeIssue issue(.inst(inst_ID),.ujump_issue(ujump_issue),.branch_issue(branch_issue),
		.ALU_issue(ALU_issue),.mul_issue(mul_issue),.div_issue(div_issue),.load_issue(load_issue),.store_issue(store_issue),
		.ImmSel(ImmSel),.ALU_op(ALU_op),.JUMP_op(JUMP_op),.ALUSrcA(ALUSrcA),.ALUSrcB(ALUSrcB),.FU_regWrite(FU_regWrite));

	ImmGen imm_gen(.ImmSel(ImmSel),.inst_field(inst_ID),.Imm_out(Imm));

	tristate #(8) alu_tag(.dout(reg_w_tag),.din(ALU_issue_tag),.en(ALU_issue));
	tristate #(8) mul_tag(.dout(reg_w_tag),.din(mul_issue_tag),.en(mul_issue));
	tristate #(8) div_tag(.dout(reg_w_tag),.din(div_issue_tag),.en(div_issue));
	tristate #(8) load_tag(.dout(reg_w_tag),.din(load_issue_tag),.en(load_issue));

	taggedRegs tregs(.clk(debug_clk),.rst(rst),.FU_regWrite(FU_regWrite & ~normal_stall),.ujump_wb(ujump_issue & ~jump_stall),
		.raddr_A(inst_ID[19:15]),.rdata_A(rs1_data),.raddr_B(inst_ID[24:20]),.rdata_B(rs2_data),
		.waddr(inst_ID[11:7]),.w_tag(reg_w_tag),.cdb(cdb),.PC(PC_ID),
		.Debug_addr(debug_addr[4:0]),.Debug_regs(debug_regs));

	MUX2T1_40 mux_imm_ALU_A(.I0(rs1_data),.I1({8'd0,PC_ID}),.s(ALUSrcA),.o(ALUA));

	MUX2T1_40 mux_imm_ALU_B(.I0(rs2_data),.I1({8'd0,Imm}),.s(ALUSrcB),.o(ALUB));

	JumpHandle jh(.branch_issue(branch_issue),.ujump_issue(ujump_issue),.JUMP_op(JUMP_op),
		.q_rs1_in(rs1_data[39:32]),.q_rs2_in(rs2_data[39:32]),.rs1_data_in(rs1_data[31:0]),.rs2_data_in(rs2_data[31:0]),
		.imm(Imm),.PC(PC_ID),.cdb(cdb),.PC_jump(PC_jump),.to_jump(to_jump),.jump_stall(jump_stall));

	add_32 add_addr(.a(rs1_data[31:0]),.b(Imm),.c(ls_addr));

	assign ls_addr_unresolved = |rs1_data[39:32] & (load_issue | store_issue);

	assign struct_hazard =  ALU_issue & ALU_all_busy |
							mul_issue & mul_all_busy |
							div_issue & div_all_busy |
							load_issue & load_all_busy |
							store_issue & store_all_busy;
	
	assign waw_hazard = store_issue & store_conflict_stall;

	assign normal_stall = ls_addr_unresolved | struct_hazard | waw_hazard;


	// FU
	unit_ALU alu(.clk(debug_clk),.rst(rst),.issue(ALU_issue & ~normal_stall),.cdb(cdb),.ALUControl_in(ALU_op),
		.q1_in(ALUA[39:32]),.v1_in(ALUA[31:0]),.q2_in(ALUB[39:32]),.v2_in(ALUB[31:0]),
		.all_busy(ALU_all_busy),.cdb_request(ALU_cdb_request),.cdb_out(ALU_cdb_in),.issue_tag(ALU_issue_tag));

	unit_mul mul(.clk(debug_clk),.rst(rst),.issue(mul_issue & ~normal_stall),.cdb(cdb),
		.q1_in(rs1_data[39:32]),.v1_in(rs1_data[31:0]),.q2_in(rs2_data[39:32]),.v2_in(rs2_data[31:0]),
		.all_busy(mul_all_busy),.cdb_request(mul_cdb_request),.cdb_out(mul_cdb_in),.issue_tag(mul_issue_tag));

	unit_div div(.clk(debug_clk),.rst(rst),.issue(div_issue & ~normal_stall),.cdb(cdb),
		.q1_in(rs1_data[39:32]),.v1_in(rs1_data[31:0]),.q2_in(rs2_data[39:32]),.v2_in(rs2_data[31:0]),
		.all_busy(div_all_busy),.cdb_request(div_cdb_request),.cdb_out(div_cdb_in),.issue_tag(div_issue_tag));


	unit_load_store ls(.clk(debug_clk),.rst(rst),.cdb(cdb),.cdb_request(load_cdb_request),.cdb_out(load_cdb_in),
		.ls_addr_in(ls_addr),.ls_u_b_h_w_in(inst_ID[14:12]),.store_issue(store_issue & ~normal_stall),
		.store_q_data_in(rs2_data[39:32]),.store_data_in(rs2_data[31:0]),.store_all_busy(store_all_busy),
		.load_issue(load_issue & ~normal_stall),.load_all_busy(load_all_busy),.load_issue_tag(load_issue_tag),
		.store_conflict_stall(store_conflict_stall));


	// CDB
	common_data_bus bus(.clk(debug_clk),.rst(rst),.cdb(cdb),
		.ALU_cdb_request(ALU_cdb_request),.ALU_cdb_in(ALU_cdb_in),
		.mul_cdb_request(mul_cdb_request),.mul_cdb_in(mul_cdb_in),
		.div_cdb_request(div_cdb_request),.div_cdb_in(div_cdb_in),
		.ls_cdb_request(load_cdb_request),.ls_cdb_in(load_cdb_in));



	always @* begin
		case (debug_addr)
			32: Test_signal = PC_IF;
			33: Test_signal = inst_IF;
			34: Test_signal = PC_ID;
			35: Test_signal = inst_ID;
			36: Test_signal = ls_addr_unresolved;
			37: Test_signal = struct_hazard;
			38: Test_signal = normal_stall;
			39: Test_signal = jump_stall;

			40: Test_signal = to_jump;
			41: Test_signal = ALUSrcA;
			42: Test_signal = ALUSrcB;
			43: Test_signal = FU_regWrite;
			44: Test_signal = ALU_issue;
			45: Test_signal = mul_issue;
			46: Test_signal = div_issue;
			47: Test_signal = load_issue;

			48: Test_signal = store_issue;
			49: Test_signal = ujump_issue;
			50: Test_signal = branch_issue;
			51: Test_signal = ImmSel;
			52: Test_signal = ALU_op;
			53: Test_signal = JUMP_op;
			54: Test_signal = Imm;
			55: Test_signal = ls_addr;

			56: Test_signal = inst_ID[19:15];
			57: Test_signal = rs1_data;
			58: Test_signal = inst_ID[24:20];
			59: Test_signal = rs2_data;
			60: Test_signal = ALUA;
			61: Test_signal = ALUB;
			62: Test_signal = inst_ID[11:7];
			63: Test_signal = PC_jump;

			64: Test_signal = ALU_cdb_request;
			65: Test_signal = mul_cdb_request;
			66: Test_signal = div_cdb_request;
			67: Test_signal = load_cdb_request;
			68: Test_signal = ALU_cdb_in;
			69: Test_signal = mul_cdb_in;
			70: Test_signal = div_cdb_in;
			71: Test_signal = load_cdb_in;

			72: Test_signal = ALU_issue_tag;
			73: Test_signal = mul_issue_tag;
			74: Test_signal = div_issue_tag;
			75: Test_signal = load_issue_tag;
			76: Test_signal = reg_w_tag;
			77: Test_signal = cdb[40] ? 40'h11111_11111 : 40'h0;
			78: Test_signal = cdb[39:0];	

			default: Test_signal = 40'h8AA55_AA558;
		endcase
	end

endmodule