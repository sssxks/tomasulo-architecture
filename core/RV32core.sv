`timescale 1ns / 1ps
`include "define.vh"

//---------------------------------------------------------
// RV32core: Tomasulo-based 32-bit RISC-V processor core
// Implements dynamic scheduling with register renaming
// Stages: IF (fetch), ID (decode/issue), EX (execute via
// multiple functional units), and WB (via common data bus)
//---------------------------------------------------------

module  RV32core(
		// Debug interface signals
		input debug_en,       // Enable debug mode
		input debug_step,     // Single-step clock when in debug mode
		input [6:0] debug_addr,  // Select internal signal to observe
		output[39:0] debug_data, // Multiplexed debug output data
		
		// Core clock and reset
		input clk,            // System clock
		input rst             // Synchronous reset
	);

	// Instantiate structs
	debug_if_t dbg_if;
	if_stage_t if_stage;
	id_stage_t id_stage;
	dispatch_signals_t dispatch_signals; // Changed name to avoid conflict with module instance 'dispatch'
	hazard_stall_ctrl_t hazard_ctrl;
	branch_pred_ctrl_t bp_ctrl;
	tagged_data_t rs1_operand;
	tagged_data_t rs2_operand;
	tagged_data_t alu_op_a;
	tagged_data_t alu_op_b;
	cdb_fu_request_t cdb_requests;
	cdb_fu_input_t cdb_inputs;
	cdb_bus_t cdb_bus;
	rs_status_t rs_status;
	rs_issue_tags_t rs_tags;

	// Debug interface logic
	// wire dbg_if.debug_clk;        // Gated clock (normal or debug stepping) // Part of dbg_if
	// wire[39:0] dbg_if.debug_regs; // Register file values for debug // Part of dbg_if
	reg[39:0] Test_signal; // Internal signals for debug
	
	// Assign module ports to/from dbg_if struct
	assign dbg_if.debug_en = debug_en;
	assign dbg_if.debug_step = debug_step;
	assign dbg_if.debug_addr = debug_addr;
	assign debug_data = dbg_if.debug_data;

	// Select between register file values or other internal signals
	// assign debug_data = |dbg_if.debug_addr[6:5] ? Test_signal : dbg_if.debug_regs; // Will be dbg_if.debug_regs
	assign dbg_if.debug_data = |dbg_if.debug_addr[6:5] ? Test_signal : dbg_if.debug_regs;


	// Clock control module for debug stepping
	debug_clk clock(.clk(clk),.debug_en(dbg_if.debug_en),.debug_step(dbg_if.debug_step),.debug_clk(dbg_if.debug_clk));

		// ----- Hazard and stall control signals -----
	// wire ls_addr_unresolved;  // Load/store address depends on pending result // Part of hazard_ctrl
	// wire struct_hazard;       // No free reservation stations in target functional unit // Part of hazard_ctrl
	// wire waw_hazard;          // Write-after-write hazard (store conflicts) // Part of hazard_ctrl
	// wire normal_stall;        // Stall due to hazards // Part of hazard_ctrl
	// wire to_jump;             // Branch/jump taken flag // Part of hazard_ctrl
	// wire jump_stall;          // Stall due to unresolved branch/jump // Part of hazard_ctrl

	// Branch predictor and fetch control signals
	// wire pred_taken;            // Branch predictor taken output // Part of bp_ctrl
	// wire [31:0] pred_target;    // Branch predictor target output // Part of bp_ctrl
	// wire use_misp;              // Mispredict redirect control // Part of bp_ctrl
	// wire [31:0] fetch_pc_pred;  // Predicted next PC before mispredict // Part of bp_ctrl

	// ----- Instruction Fetch (IF) stage signals -----
	// logic [31:0] if_stage.pc;          // Current program counter // Part of if_stage
	// wire[31:0] if_stage.inst;        // Fetched instruction // Part of if_stage

	// ----- Decode and control signals -----
	// wire id_stage.ALUSrcA;              // ALU input A source select (0=rs1, 1=PC) // Part of id_stage
	// wire id_stage.ALUSrcB;              // ALU input B source select (0=rs2, 1=immediate) // Part of id_stage
	// wire id_stage.FU_regWrite;          // Register file write enable // Part of id_stage
	
	// Instruction type dispatch signals
	// wire dispatch_signals.ALU_issue;            // ALU operation to issue // Part of dispatch_signals
	// wire dispatch_signals.mul_issue;            // Multiply operation to issue // Part of dispatch_signals
	// wire dispatch_signals.div_issue;            // Division operation to issue // Part of dispatch_signals
	// wire dispatch_signals.load_issue;           // Load operation to issue // Part of dispatch_signals
	// wire dispatch_signals.store_issue;          // Store operation to issue // Part of dispatch_signals
	// wire dispatch_signals.ujump_issue;          // Unconditional jump to issue // Part of dispatch_signals
	// wire dispatch_signals.branch_issue;         // Branch operation to issue // Part of dispatch_signals
	
	// Operation control fields
	// wire[2:0] id_stage.ImmSel;          // Immediate format select // Part of id_stage
	// wire[3:0] id_stage.ALU_op;          // ALU operation code // Part of id_stage
	// wire[3:0] id_stage.JUMP_op;         // Jump/branch operation code // Part of id_stage
	
	// ----- Instruction Decode (ID) stage signals -----
	// wire[31:0] id_stage.inst;        // Decoded instruction // Part of id_stage
	// wire[31:0] id_stage.pc;          // PC in ID stage // Part of id_stage
	// wire[31:0] id_stage.imm;            // Sign-extended immediate // Part of id_stage
	// wire[31:0] id_stage.ls_addr;        // Load/store address // Part of id_stage
	
	// Register values with tags (39:32=tag, 31:0=value)
	// wire[39:0] rs1_data;       // rs1 register data with tag // Now rs1_operand (tagged_data_t)
	// wire[39:0] rs2_data;       // rs2 register data with tag // Now rs2_operand (tagged_data_t)
	// wire[39:0] ALUA;           // ALU input A with tag // Now alu_op_a (tagged_data_t)
	// wire[39:0] ALUB;           // ALU input B with tag // Now alu_op_b (tagged_data_t)

	// ----- Common Data Bus (CDB) interface signals -----
	// Request signals from functional units
	// wire ALU_cdb_request;      // ALU requesting CDB access // cdb_requests.alu
	// wire mul_cdb_request;      // Multiplier requesting CDB access // cdb_requests.mul
	// wire div_cdb_request;      // Divider requesting CDB access // cdb_requests.div
	// wire load_cdb_request;     // Load unit requesting CDB access // cdb_requests.load
	// wire branch_cdb_request;   // Branch unit requesting CDB access // cdb_requests.branch
	
	// Reservation station status signals
	// wire ALU_all_busy;         // All ALU reservation stations busy // rs_status.ALU_all_busy
	// wire mul_all_busy;         // All multiply reservation stations busy // rs_status.mul_all_busy
	// wire div_all_busy;         // All divide reservation stations busy // rs_status.div_all_busy
	// wire load_all_busy;        // All load reservation stations busy // rs_status.load_all_busy
	// wire store_all_busy;       // All store reservation stations busy // rs_status.store_all_busy
	// wire store_conflict_stall; // Store conflicts with pending load/store // rs_status.store_conflict_stall
	
	// Reservation station tags and result data
	// wire[7:0] ALU_issue_tag;   // Tag assigned to ALU operation // rs_tags.ALU
	// wire[7:0] mul_issue_tag;   // Tag assigned to multiply operation // rs_tags.mul
	// wire[7:0] div_issue_tag;   // Tag assigned to divide operation // rs_tags.div
	// wire[7:0] load_issue_tag;  // Tag assigned to load operation // rs_tags.load
	// wire[7:0] branch_issue_tag; // Tag assigned to branch operation // rs_tags.branch
	// wire[7:0] reg_w_tag;       // Tag assigned to destination register // rs_tags.reg_w_tag
	
	// Data outputs from functional units to CDB
	// wire[39:0] ALU_cdb_in;     // ALU result with tag // cdb_inputs.alu (tagged_data_t)
	// wire[39:0] mul_cdb_in;     // Multiply result with tag // cdb_inputs.mul (tagged_data_t)
	// wire[39:0] div_cdb_in;     // Divide result with tag // cdb_inputs.div (tagged_data_t)
	// wire[39:0] load_cdb_in;    // Load result with tag // cdb_inputs.load (tagged_data_t)
	// wire[39:0] branch_cdb_in;  // Branch result with tag // cdb_inputs.branch (tagged_data_t)

	// Common Data Bus - 41 bits: valid bit + 8-bit tag + 32-bit data
	// wire[40:0] cdb;            // [40]=valid, [39:32]=tag, [31:0]=data // Now cdb_bus (cdb_bus_t)


	// ROB integration
	logic rob_alloc;
	logic [31:0] rob_alloc_pc;
	logic rob_alloc_pred_taken;
	logic [31:0] rob_alloc_pred_target;
	logic [7:0] rob_alloc_dest_tag;
	wire [31*`NUM_SRBITS-1:0] rob_commit_tags_bus;
	wire [31*`NUM_SRBITS-1:0] all_tags_bus;

	// Struct for ROB commit signals
	typedef struct packed {
		logic valid;
		logic is_branch;
		logic mispredict;
		logic actual_taken;
		logic [31:0] actual_target;
		logic [7:0] dest_tag;
		logic [31:0] value;
		logic [3:0] index;
		logic [31*`NUM_SRBITS-1:0] tags_bus;
	} rob_commit_t;
	rob_commit_t rob_commit;

	// Compute ROB allocation signals
	always_comb begin
		rob_alloc            = dispatch_signals.ALU_issue | dispatch_signals.mul_issue | dispatch_signals.div_issue | dispatch_signals.load_issue | dispatch_signals.store_issue | dispatch_signals.branch_issue | dispatch_signals.ujump_issue;
		rob_alloc_pc         = id_stage.pc;
		rob_alloc_pred_taken = dispatch_signals.branch_issue & bp_ctrl.pred_taken;
		rob_alloc_pred_target= bp_ctrl.pred_target;
		rob_alloc_dest_tag   = rs_tags.reg_w_tag;
	end

	// Pack ROB commit outputs into struct
	always_comb begin
		rob_commit.valid         = rob_commit_valid;
		rob_commit.is_branch     = rob_commit_is_branch;
		rob_commit.mispredict    = rob_commit_mispredict;
		rob_commit.actual_taken  = rob_commit_actual_taken;
		rob_commit.actual_target = rob_commit_actual_target;
		rob_commit.dest_tag      = rob_commit_dest_tag;
		rob_commit.value         = rob_commit_value;
		rob_commit.index         = rob_commit_index;
		rob_commit.tags_bus      = rob_commit_tags_bus;
	end

	// Instantiate ROB with RAT checkpoints
	ROB rob0(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.flush(bp_ctrl.use_misp),             // Flush ROB entries on mispredict
		.alloc_tags_bus(all_tags_bus),
		.alloc(rob_alloc),
		.alloc_pc(rob_alloc_pc),
		.alloc_pred_taken(rob_alloc_pred_taken),
		.alloc_pred_target(rob_alloc_pred_target),
		.alloc_dest_tag(rs_tags.reg_w_tag),
		.cdb({cdb_bus.valid, cdb_bus.tag, cdb_bus.data}),
		.commit_valid(rob_commit.valid),
		.commit_is_branch(rob_commit.is_branch),
		.commit_mispredict(rob_commit.mispredict),
		.commit_actual_taken(rob_commit.actual_taken),
		.commit_actual_target(rob_commit.actual_target),
		.commit_dest_tag(rob_commit.dest_tag),
		.commit_value(rob_commit.value),
		.commit_index(rob_commit.index),
		.commit_tags_bus(rob_commit.tags_bus)
	);

	// Instantiate branch predictor
	BranchPredictor bp0(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.pc(if_stage.pc),
		.update(rob_commit.valid & rob_commit.is_branch),
		.update_taken(rob_commit.actual_taken),
		.update_target(rob_commit.actual_target),
		.taken(bp_ctrl.pred_taken),
		.target(bp_ctrl.pred_target)
	);

	// ----- Instruction Fetch (IF) Stage -----
	// Program Counter update logic using SystemVerilog always_ff
	always_ff @(posedge dbg_if.debug_clk or posedge rst) begin
		if (rst) begin
			if_stage.pc <= 32'd0;
		end else if (~hazard_ctrl.normal_stall) begin
			if (rob_commit.valid & rob_commit.mispredict)
				if_stage.pc <= rob_commit.actual_target;
			else if (bp_ctrl.pred_taken)
				if_stage.pc <= bp_ctrl.pred_target;
			else
				if_stage.pc <= if_stage.pc + 32'd4;
		end
	end

	// Instruction memory (ROM) - fetches instruction at current PC
	// Uses PC[8:2] as word address (PC is byte-addressed)
	ROM_D inst_rom(
		.a(if_stage.pc[8:2]),                    // Word address
		.spo(if_stage.inst)                      // Instruction output
	);


	// ----- Instruction Decode/Issue (ID) Stage -----
	// Pipeline register between IF and ID stages
	REG_ID reg_ID(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.EN(~hazard_ctrl.normal_stall),  // Only update when no stalls
		.flush(bp_ctrl.use_misp),     // Flush on mispredict
		.PCOUT(if_stage.pc),                     // PC from IF stage
		.IR(if_stage.inst),                      // Instruction from IF stage
		.IR_ID(id_stage.inst),                   // Instruction to ID stage
		.PCurrent_ID(id_stage.pc),               // PC to ID stage
		.valid()                           // Valid bit (unused)
	);

	// Instruction decoder - determines operation type and control signals
	DecodeDispatch dispatch(
		.inst(id_stage.inst),                    // Instruction to decode
		// Operation type outputs
		.ujump_dispatch(dispatch_signals.ujump_issue),       // Unconditional jump
		.branch_dispatch(dispatch_signals.branch_issue),     // Conditional branch
		.ALU_dispatch(dispatch_signals.ALU_issue),          // ALU operation
		.mul_dispatch(dispatch_signals.mul_issue),          // Multiply operation
		.div_dispatch(dispatch_signals.div_issue),          // Divide operation
		.load_dispatch(dispatch_signals.load_issue),        // Load operation
		.store_dispatch(dispatch_signals.store_issue),      // Store operation
		// Control signal outputs
		.ImmSel(id_stage.ImmSel),                   // Immediate format select
		.ALU_op(id_stage.ALU_op),                   // ALU operation code
		.JUMP_op(id_stage.JUMP_op),                 // Jump/branch operation code
		.ALUSrcA(id_stage.ALUSrcA),                // ALU input A source select
		.ALUSrcB(id_stage.ALUSrcB),                // ALU input B source select
		.FU_regWrite(id_stage.FU_regWrite)          // Register write enable
	);

	// Immediate generator - sign extends immediates based on instruction format
	ImmGen imm_gen(
		.ImmSel(id_stage.ImmSel),                   // Format select from decoder
		.inst_field(id_stage.inst),              // Instruction fields
		.Imm_out(id_stage.imm)                      // Generated immediate value
	);

	// ----- Register Renaming Logic -----
	// Tri-state buffers to select which tag drives the register write tag
	// Only one issue signal should be active at a time
	tristate #(8) alu_tag_driver(.dout(rs_tags.reg_w_tag), .din(rs_tags.ALU), .en(dispatch_signals.ALU_issue));
	tristate #(8) mul_tag_driver(.dout(rs_tags.reg_w_tag), .din(rs_tags.mul), .en(dispatch_signals.mul_issue));
	tristate #(8) div_tag_driver(.dout(rs_tags.reg_w_tag), .din(rs_tags.div), .en(dispatch_signals.div_issue));
	tristate #(8) load_tag_driver(.dout(rs_tags.reg_w_tag), .din(rs_tags.load), .en(dispatch_signals.load_issue));
	tristate #(8) branch_tag_driver(.dout(rs_tags.reg_w_tag), .din(rs_tags.branch), .en(dispatch_signals.branch_issue));

	// Register file with register renaming and checkpoint support
	taggedRegs tregs(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		// Control inputs
		.FU_regWrite(id_stage.FU_regWrite & ~hazard_ctrl.normal_stall), // Only rename when no stall
		.ujump_wb(dispatch_signals.ujump_issue & ~hazard_ctrl.jump_stall),      // Jump writeback
		// Register read ports
		.raddr_A(id_stage.inst[19:15]),          // rs1 address
		.rdata_A_o(rs1_operand), // rs1 data with tag
		.raddr_B(id_stage.inst[24:20]),          // rs2 address
		.rdata_B_o(rs2_operand), // rs2 data with tag
		// Register write port
		.waddr(id_stage.inst[11:7]),             // rd address
		.w_tag(rs_tags.reg_w_tag),                 // Tag for renaming
		.cdb_i(cdb_bus), // Common data bus for updates
		.PC(id_stage.pc),                        // PC for JAL/JALR link address
		// Debug interface
		.Debug_addr(dbg_if.debug_addr[4:0]),      // Debug register select
		.Debug_regs(dbg_if.debug_regs),
		// RAT checkpoint/restore
		.all_tags_bus(all_tags_bus),
		.restore(rob_commit.valid & rob_commit.mispredict),
		.restore_index(rob_commit.index),
		.restore_tags_bus(rob_commit.tags_bus)
	);

	// ----- ALU Input Selection -----
	// Multiplexer for ALU input A: rs1 or PC
	MUX2T1_40 mux_imm_ALU_A(
		.I0({rs1_operand.tag, rs1_operand.val}), // rs1 value with tag
		.I1({8'd0,id_stage.pc}),                 // PC with zero tag
		.s(id_stage.ALUSrcA),                       // Select control
		.o({alu_op_a.tag, alu_op_a.val})       // Selected output
	);

	// Multiplexer for ALU input B: rs2 or immediate
	MUX2T1_40 mux_imm_ALU_B(
		.I0({rs2_operand.tag, rs2_operand.val}), // rs2 value with tag
		.I1({8'd0,id_stage.imm}),                   // Immediate with zero tag
		.s(id_stage.ALUSrcB),                       // Select control
		.o({alu_op_b.tag, alu_op_b.val})       // Selected output
	);

	// ----- Branch/Jump Handling -----
	// Handles branch/jump target calculation and condition evaluation
	// Removed JumpHandle instantiation

	// Load/Store unit - handles memory operations
	unit_load_store ls(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.flush(bp_ctrl.use_misp),                  // Add flush on mispredict
		.cdb_i(cdb_bus), // CHANGED: Connect to cdb_bus_t struct directly
		.cdb_request(cdb_requests.load),     // Request to broadcast load result
		.load_cdb_data_o(cdb_inputs.load), // CHANGED: Connect to tagged_data_t struct for load output
		// Memory operation details
		.ls_addr_in(id_stage.ls_addr),               // Calculated memory address
		.ls_u_b_h_w_in(id_stage.inst[14:12]),     // Size/sign: byte/half/word, signed/unsigned
		// Store operation signals
		.store_issue(dispatch_signals.store_issue & ~hazard_ctrl.normal_stall),  // Issue store operation
		.store_data_i(rs2_operand),        // CHANGED: Connect to tagged_data_t struct for store data
		.store_all_busy(rs_status.store_all_busy),    // All store buffer entries busy
		// Load operation signals
		.load_issue(dispatch_signals.load_issue & ~hazard_ctrl.normal_stall),    // Issue load operation
		.load_all_busy(rs_status.load_all_busy),      // All load reservation stations busy
		.load_issue_tag(rs_tags.load),    // Tag assigned to load operation
		// Hazard detection
		.store_conflict_stall(rs_status.store_conflict_stall)  // Store conflicts with pending load/store
	);

	// Branch unit - handles branch operations
	unit_branch br(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.branch_issue(dispatch_signals.branch_issue & ~hazard_ctrl.normal_stall),
		.ujump_issue(dispatch_signals.ujump_issue & ~hazard_ctrl.normal_stall),
		.flush(bp_ctrl.use_misp),
		.cdb_i(cdb_bus), // Connect to cdb_bus_t struct
		.JUMP_op(id_stage.JUMP_op),
		.op_a_i(rs1_operand), // Connect to tagged_data_t struct for op A (rs1)
		.op_b_i(rs2_operand), // Connect to tagged_data_t struct for op B (rs2)
		.imm(id_stage.imm),
		.PC(id_stage.pc),
		.jump_stall(hazard_ctrl.jump_stall), // This is an output from unit_branch
		.cdb_request(cdb_requests.branch),
		.cdb_data_o(cdb_inputs.branch), // Connect to tagged_data_t struct for cdb output
		.issue_tag(rs_tags.branch)
	);

	// ----- Load/Store Address Calculation -----
	// Adder for load/store address: base + offset
	add_32 add_addr(
		.a(rs1_operand.val),               // Base address (rs1)
		.b(id_stage.imm),                           // Offset (immediate)
		.c(id_stage.ls_addr)                        // Calculated address
	);

	// ----- Hazard Detection Logic -----
	// Load/store address depends on a pending result (tag != 0)
	assign hazard_ctrl.ls_addr_unresolved = |rs1_operand.tag & (dispatch_signals.load_issue | dispatch_signals.store_issue);

	// Structural hazard: all reservation stations of required type are busy
	assign hazard_ctrl.struct_hazard =  (dispatch_signals.ALU_issue & rs_status.ALU_all_busy) |
							(dispatch_signals.mul_issue & rs_status.mul_all_busy) |
							(dispatch_signals.div_issue & rs_status.div_all_busy) |
							(dispatch_signals.load_issue & rs_status.load_all_busy) |
							(dispatch_signals.store_issue & rs_status.store_all_busy);
	
	// Write-after-write hazard: store conflicts with pending store
	assign hazard_ctrl.waw_hazard = dispatch_signals.store_issue & rs_status.store_conflict_stall;

	// Normal stall: any hazard that prevents instruction issue
	assign hazard_ctrl.normal_stall = hazard_ctrl.ls_addr_unresolved | hazard_ctrl.struct_hazard | hazard_ctrl.waw_hazard;


	// ----- Functional Units (Execution Stage) -----
	// Each functional unit contains reservation stations and execution logic
	// ALU unit - handles arithmetic, logic, and shift operations
	unit_ALU alu(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.issue(dispatch_signals.ALU_issue & ~hazard_ctrl.normal_stall),  // Only issue when no stalls
		.flush(bp_ctrl.use_misp),  // Add flush on mispredict
		.cdb_i(cdb_bus), // Connect to the cdb_bus_t struct directly
		.ALUControl_in(id_stage.ALU_op),            // ALU operation type
		// Input operands with tags
		.op_a_i(alu_op_a),             // Connect to tagged_data_t struct for op A
		.op_b_i(alu_op_b),             // Connect to tagged_data_t struct for op B
		// Status and result signals
		.all_busy(rs_status.ALU_all_busy),   // All reservation stations busy
		.cdb_request(cdb_requests.alu),     // Request to broadcast result
		.cdb_data_o(cdb_inputs.alu),       // Connect to tagged_data_t struct for cdb output
		.issue_tag(rs_tags.ALU)           // Tag assigned to this operation
	);

	// Multiplication unit - handles multiply operations
	unit_mul mul(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.issue(dispatch_signals.mul_issue & ~hazard_ctrl.normal_stall),  // Only issue when no stalls
		.flush(bp_ctrl.use_misp),  // Add flush on mispredict
		.cdb_i(cdb_bus),                         // Common Data Bus struct
		// Input operands with tags (directly from register file)
		.op_a_i(rs1_operand),                // Operand A (tag + data)
		.op_b_i(rs2_operand),                // Operand B (tag + data)
		// Status and result signals
		.all_busy(rs_status.mul_all_busy),   // All reservation stations busy
		.cdb_request(cdb_requests.mul),     // Request to broadcast result
		.cdb_data_o(cdb_inputs.mul),       // Multiply result data (tag + data)
		.issue_tag(rs_tags.mul)           // Tag assigned to this operation
	);

	// Division unit - handles divide operations
	unit_div div(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.issue(dispatch_signals.div_issue & ~hazard_ctrl.normal_stall),  // Only issue when no stalls
		.flush(bp_ctrl.use_misp),  // Add flush on mispredict
		.cdb_i(cdb_bus),                         // Common Data Bus struct
		// Input operands with tags (directly from register file)
		.op_a_i(rs1_operand),                // Operand A (tag + data)
		.op_b_i(rs2_operand),                // Operand B (tag + data)
		// Status and result signals
		.all_busy(rs_status.div_all_busy),   // All reservation stations busy
		.cdb_request(cdb_requests.div),     // Request to broadcast result
		.cdb_data_o(cdb_inputs.div),       // Divide result data (tag + data)
		.issue_tag(rs_tags.div)           // Tag assigned to this operation
	);

	// ----- Common Data Bus (CDB) -----
	// Arbitrates between multiple functional units requesting to broadcast results
	common_data_bus bus(
		.clk(dbg_if.debug_clk),
		.rst(rst),
		.ALU_cdb_request(cdb_requests.alu),
		.mul_cdb_request(cdb_requests.mul),
		.div_cdb_request(cdb_requests.div),
		.ls_cdb_request(cdb_requests.load),
		.branch_cdb_request(cdb_requests.branch),
		.ALU_cdb_in({cdb_inputs.alu.tag, cdb_inputs.alu.val}),
		.mul_cdb_in({cdb_inputs.mul.tag, cdb_inputs.mul.val}),
		.div_cdb_in({cdb_inputs.div.tag, cdb_inputs.div.val}),
		.ls_cdb_in({cdb_inputs.load.tag, cdb_inputs.load.val}),
		.branch_cdb_in({cdb_inputs.branch.tag, cdb_inputs.branch.val}),
		.cdb({cdb_bus.valid, cdb_bus.tag, cdb_bus.data})
	);



	// ----- Debug Signal Multiplexer -----
	// Selects internal signals for observation based on debug_addr
	always_comb begin
		case (dbg_if.debug_addr)
			// ----- IF stage signals (32-39) -----
			32: Test_signal = {8'd0, if_stage.pc};                // Program counter
			33: Test_signal = {8'd0, if_stage.inst};              // Fetched instruction
			34: Test_signal = {8'd0, id_stage.pc};                // ID stage PC
			35: Test_signal = {8'd0, id_stage.inst};              // Decoded instruction
			36: Test_signal = {39'd0, hazard_ctrl.ls_addr_unresolved};    // Load/store address hazard
			37: Test_signal = {39'd0, hazard_ctrl.struct_hazard};         // Structural hazard
			38: Test_signal = {39'd0, hazard_ctrl.normal_stall};          // Pipeline stall
			39: Test_signal = {39'd0, hazard_ctrl.jump_stall};            // Branch/jump stall

			// ----- Control signals (40-47) -----
			40: Test_signal = {39'd0, hazard_ctrl.to_jump};               // Branch/jump taken flag
			41: Test_signal = {39'd0, id_stage.ALUSrcA};               // ALU input A select
			42: Test_signal = {39'd0, id_stage.ALUSrcB};               // ALU input B select
			43: Test_signal = {39'd0, id_stage.FU_regWrite};           // Register write enable
			44: Test_signal = {39'd0, dispatch_signals.ALU_issue};             // ALU issue flag
			45: Test_signal = {39'd0, dispatch_signals.mul_issue};             // Multiply issue flag
			46: Test_signal = {39'd0, dispatch_signals.div_issue};             // Divide issue flag
			47: Test_signal = {39'd0, dispatch_signals.load_issue};            // Load issue flag

			// ----- More control and data signals (48-55) -----
			48: Test_signal = {39'd0, dispatch_signals.store_issue};           // Store issue flag
			49: Test_signal = {39'd0, dispatch_signals.ujump_issue};           // Unconditional jump flag
			50: Test_signal = {39'd0, dispatch_signals.branch_issue};          // Branch issue flag
			51: Test_signal = {36'd0, id_stage.ImmSel};                // Immediate format select
			52: Test_signal = {35'd0, id_stage.ALU_op};                // ALU operation code
			53: Test_signal = {35'd0, id_stage.JUMP_op};               // Jump operation code
			54: Test_signal = {8'd0, id_stage.imm};                   // Generated immediate
			55: Test_signal = {8'd0, id_stage.ls_addr};               // Load/store address

			// ----- Register and operand values (56-63) -----
			56: Test_signal = {35'd0, id_stage.inst[19:15]};        // rs1 address field
			57: Test_signal = {rs1_operand.tag, rs1_operand.val};   // rs1 value with tag
			58: Test_signal = {35'd0, id_stage.inst[24:20]};        // rs2 address field
			59: Test_signal = {rs2_operand.tag, rs2_operand.val};   // rs2 value with tag
			60: Test_signal = {alu_op_a.tag, alu_op_a.val};         // ALU input A
			61: Test_signal = {alu_op_b.tag, alu_op_b.val};         // ALU input B
			62: Test_signal = {35'd0, id_stage.inst[11:7]};         // rd address field
			63: Test_signal = branch_cdb_in;         // Branch unit output to CDB (includes jump target address)

			// ----- CDB request signals (64-71) -----
			64: Test_signal = {39'd0, cdb_requests.alu};       // ALU requesting CDB
			65: Test_signal = {39'd0, cdb_requests.mul};       // Multiply requesting CDB
			66: Test_signal = {39'd0, cdb_requests.div};       // Divide requesting CDB
			67: Test_signal = {39'd0, cdb_requests.load};      // Load requesting CDB
			68: Test_signal = {cdb_inputs.alu.tag, cdb_inputs.alu.val}; // ALU result
			69: Test_signal = {cdb_inputs.mul.tag, cdb_inputs.mul.val}; // Multiply result
			70: Test_signal = {cdb_inputs.div.tag, cdb_inputs.div.val}; // Divide result
			71: Test_signal = {cdb_inputs.load.tag, cdb_inputs.load.val}; // Load result

			// ----- Tag and CDB values (72-78) -----
			72: Test_signal = {32'd0, rs_tags.ALU};         // ALU operation tag
			73: Test_signal = {32'd0, rs_tags.mul};         // Multiply operation tag
			74: Test_signal = {32'd0, rs_tags.div};         // Divide operation tag
			75: Test_signal = {32'd0, rs_tags.load};        // Load operation tag
			76: Test_signal = {32'd0, rs_tags.reg_w_tag};   // Register write tag
			77: Test_signal = {39'd0, cdb_bus.valid} ? 40'h11111_11111 : 40'h0;  // CDB valid bit
			78: Test_signal = {cdb_bus.tag, cdb_bus.data};  // CDB tag and data

			// Default pattern for invalid addresses
			default: Test_signal = 40'h8AA55_AA558;   // Recognizable pattern
		endcase
	end

endmodule