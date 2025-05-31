`timescale 1ns / 1ps

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

	// Debug interface logic
	wire debug_clk;        // Gated clock (normal or debug stepping)
	wire[39:0] debug_regs; // Register file values for debug
	reg[39:0] Test_signal; // Internal signals for debug
	
	// Select between register file values or other internal signals
	assign debug_data = |debug_addr[6:5] ? Test_signal : debug_regs;

	// Clock control module for debug stepping
	debug_clk clock(.clk(clk),.debug_en(debug_en),.debug_step(debug_step),.debug_clk(debug_clk));

		// ----- Hazard and stall control signals -----
	wire ls_addr_unresolved;  // Load/store address depends on pending result
	wire struct_hazard;       // No free reservation stations in target functional unit
	wire waw_hazard;          // Write-after-write hazard (store conflicts)
	wire normal_stall;        // Stall due to hazards
	wire to_jump;             // Branch/jump taken flag
	wire jump_stall;          // Stall due to unresolved branch/jump

	// ----- Instruction Fetch (IF) stage signals -----
	wire[31:0] PC_IF;          // Current program counter
	wire[31:0] next_PC_IF;     // Next program counter value
	wire[31:0] PCp4_IF;        // PC+4 (sequential next address)
	wire[31:0] PC_jump;        // Jump/branch target address
	wire[31:0] inst_IF;        // Fetched instruction

	// ----- Decode and control signals -----
	wire ALUSrcA;              // ALU input A source select (0=rs1, 1=PC)
	wire ALUSrcB;              // ALU input B source select (0=rs2, 1=immediate)
	wire FU_regWrite;          // Register file write enable
	
	// Instruction type dispatch signals
	wire ALU_issue;            // ALU operation to issue
	wire mul_issue;            // Multiply operation to issue
	wire div_issue;            // Division operation to issue
	wire load_issue;           // Load operation to issue
	wire store_issue;          // Store operation to issue
	wire ujump_issue;          // Unconditional jump to issue
	wire branch_issue;         // Branch operation to issue
	
	// Operation control fields
	wire[2:0] ImmSel;          // Immediate format select
	wire[3:0] ALU_op;          // ALU operation code
	wire[3:0] JUMP_op;         // Jump/branch operation code
	
	// ----- Instruction Decode (ID) stage signals -----
	wire[31:0] inst_ID;        // Decoded instruction
	wire[31:0] PC_ID;          // PC in ID stage
	wire[31:0] Imm;            // Sign-extended immediate
	wire[31:0] ls_addr;        // Load/store address
	
	// Register values with tags (39:32=tag, 31:0=value)
	wire[39:0] rs1_data;       // rs1 register data with tag
	wire[39:0] rs2_data;       // rs2 register data with tag
	wire[39:0] ALUA;           // ALU input A with tag
	wire[39:0] ALUB;           // ALU input B with tag

	// ----- Common Data Bus (CDB) interface signals -----
	// Request signals from functional units
	wire ALU_cdb_request;      // ALU requesting CDB access
	wire mul_cdb_request;      // Multiplier requesting CDB access
	wire div_cdb_request;      // Divider requesting CDB access
	wire load_cdb_request;     // Load unit requesting CDB access
	
	// Reservation station status signals
	wire ALU_all_busy;         // All ALU reservation stations busy
	wire mul_all_busy;         // All multiply reservation stations busy
	wire div_all_busy;         // All divide reservation stations busy
	wire load_all_busy;        // All load reservation stations busy
	wire store_all_busy;       // All store reservation stations busy
	wire store_conflict_stall; // Store conflicts with pending load/store
	
	// Reservation station tags and result data
	wire[7:0] ALU_issue_tag;   // Tag assigned to ALU operation
	wire[7:0] mul_issue_tag;   // Tag assigned to multiply operation
	wire[7:0] div_issue_tag;   // Tag assigned to divide operation
	wire[7:0] load_issue_tag;  // Tag assigned to load operation
	wire[7:0] reg_w_tag;       // Tag assigned to destination register
	
	// Data outputs from functional units to CDB
	wire[39:0] ALU_cdb_in;     // ALU result with tag
	wire[39:0] mul_cdb_in;     // Multiply result with tag
	wire[39:0] div_cdb_in;     // Divide result with tag
	wire[39:0] load_cdb_in;    // Load result with tag

	// Common Data Bus - 41 bits: valid bit + 8-bit tag + 32-bit data
	wire[40:0] cdb;            // [40]=valid, [39:32]=tag, [31:0]=data


	// ----- Instruction Fetch (IF) Stage -----
	// Program Counter register - only updates when no stalls are active
	REG32 REG_PC(
		.clk(debug_clk),
		.rst(rst),
		.CE(~normal_stall & ~jump_stall),  // Enable only when no stalls
		.D(next_PC_IF),                    // Next PC value
		.Q(PC_IF)                         // Current PC output
	);

	// PC+4 adder for sequential execution
	add_32 add_IF(
		.a(PC_IF),
		.b(32'd4),
		.c(PCp4_IF)                       // PC+4 result
	);

	// Multiplexer to select between PC+4 and jump target
	MUX2T1_32 mux_IF(
		.I0(PCp4_IF),                      // Sequential next PC
		.I1(PC_jump),                      // Jump/branch target
		.s(to_jump),                       // Select jump if branch taken
		.o(next_PC_IF)                     // Selected next PC
	);

	// Instruction memory (ROM) - fetches instruction at current PC
	// Uses PC[8:2] as word address (PC is byte-addressed)
	ROM_D inst_rom(
		.a(PC_IF[8:2]),                    // Word address
		.spo(inst_IF)                      // Instruction output
	);


	// ----- Instruction Decode/Issue (ID) Stage -----
	// Pipeline register between IF and ID stages
	REG_ID reg_ID(
		.clk(debug_clk),
		.rst(rst),
		.EN(~normal_stall & ~jump_stall),  // Only update when no stalls
		.flush(to_jump & ~jump_stall),     // Clear on taken branch/jump
		.PCOUT(PC_IF),                     // PC from IF stage
		.IR(inst_IF),                      // Instruction from IF stage
		.IR_ID(inst_ID),                   // Instruction to ID stage
		.PCurrent_ID(PC_ID),               // PC to ID stage
		.valid()                           // Valid bit (unused)
	);

	// Instruction decoder - determines operation type and control signals
	DecodeDispatch dispatch(
		.inst(inst_ID),                    // Instruction to decode
		// Operation type outputs
		.ujump_dispatch(ujump_issue),       // Unconditional jump
		.branch_dispatch(branch_issue),     // Conditional branch
		.ALU_dispatch(ALU_issue),          // ALU operation
		.mul_dispatch(mul_issue),          // Multiply operation
		.div_dispatch(div_issue),          // Divide operation
		.load_dispatch(load_issue),        // Load operation
		.store_dispatch(store_issue),      // Store operation
		// Control signal outputs
		.ImmSel(ImmSel),                   // Immediate format select
		.ALU_op(ALU_op),                   // ALU operation code
		.JUMP_op(JUMP_op),                 // Jump/branch operation code
		.ALUSrcA(ALUSrcA),                // ALU input A source select
		.ALUSrcB(ALUSrcB),                // ALU input B source select
		.FU_regWrite(FU_regWrite)          // Register write enable
	);

	// Immediate generator - sign extends immediates based on instruction format
	ImmGen imm_gen(
		.ImmSel(ImmSel),                   // Format select from decoder
		.inst_field(inst_ID),              // Instruction fields
		.Imm_out(Imm)                      // Generated immediate value
	);

	// ----- Register Renaming Logic -----
	// Tri-state buffers to select which tag drives the register write tag
	// Only one issue signal should be active at a time
	tristate #(8) alu_tag(.dout(reg_w_tag), .din(ALU_issue_tag), .en(ALU_issue));
	tristate #(8) mul_tag(.dout(reg_w_tag), .din(mul_issue_tag), .en(mul_issue));
	tristate #(8) div_tag(.dout(reg_w_tag), .din(div_issue_tag), .en(div_issue));
	tristate #(8) load_tag(.dout(reg_w_tag), .din(load_issue_tag), .en(load_issue));

	// Register file with register renaming support
	taggedRegs tregs(
		.clk(debug_clk),
		.rst(rst),
		// Control inputs
		.FU_regWrite(FU_regWrite & ~normal_stall), // Only rename when no stall
		.ujump_wb(ujump_issue & ~jump_stall),      // Jump writeback
		// Register read ports
		.raddr_A(inst_ID[19:15]),          // rs1 address
		.rdata_A(rs1_data),                // rs1 data with tag
		.raddr_B(inst_ID[24:20]),          // rs2 address
		.rdata_B(rs2_data),                // rs2 data with tag
		// Register write port
		.waddr(inst_ID[11:7]),             // rd address
		.w_tag(reg_w_tag),                 // Tag for renaming
		.cdb(cdb),                         // Common data bus for updates
		.PC(PC_ID),                        // PC for JAL/JALR link address
		// Debug interface
		.Debug_addr(debug_addr[4:0]),      // Debug register select
		.Debug_regs(debug_regs)            // Debug register output
	);

	// ----- ALU Input Selection -----
	// Multiplexer for ALU input A: rs1 or PC
	MUX2T1_40 mux_imm_ALU_A(
		.I0(rs1_data),                     // rs1 value with tag
		.I1({8'd0,PC_ID}),                 // PC with zero tag
		.s(ALUSrcA),                       // Select control
		.o(ALUA)                           // Selected output
	);

	// Multiplexer for ALU input B: rs2 or immediate
	MUX2T1_40 mux_imm_ALU_B(
		.I0(rs2_data),                     // rs2 value with tag
		.I1({8'd0,Imm}),                   // Immediate with zero tag
		.s(ALUSrcB),                       // Select control
		.o(ALUB)                           // Selected output
	);

	// ----- Branch/Jump Handling -----
	// Handles branch/jump target calculation and condition evaluation
	JumpHandle jh(
		.branch_issue(branch_issue),        // Branch instruction flag
		.ujump_issue(ujump_issue),          // Unconditional jump flag
		.JUMP_op(JUMP_op),                 // Branch/jump operation type
		// Register values with tags for condition checking
		.q_rs1_in(rs1_data[39:32]),        // rs1 tag
		.q_rs2_in(rs2_data[39:32]),        // rs2 tag
		.rs1_data_in(rs1_data[31:0]),      // rs1 value
		.rs2_data_in(rs2_data[31:0]),      // rs2 value
		.imm(Imm),                         // Immediate for offset
		.PC(PC_ID),                        // Current PC
		.cdb(cdb),                         // Common data bus for updates
		// Outputs
		.PC_jump(PC_jump),                 // Calculated jump target
		.to_jump(to_jump),                 // Take jump flag
		.jump_stall(jump_stall)            // Stall due to unresolved branch
	);

	// ----- Load/Store Address Calculation -----
	// Adder for load/store address: base + offset
	add_32 add_addr(
		.a(rs1_data[31:0]),               // Base address (rs1)
		.b(Imm),                           // Offset (immediate)
		.c(ls_addr)                        // Calculated address
	);

	// ----- Hazard Detection Logic -----
	// Load/store address depends on a pending result (tag != 0)
	assign ls_addr_unresolved = |rs1_data[39:32] & (load_issue | store_issue);

	// Structural hazard: all reservation stations of required type are busy
	assign struct_hazard =  ALU_issue & ALU_all_busy |
							mul_issue & mul_all_busy |
							div_issue & div_all_busy |
							load_issue & load_all_busy |
							store_issue & store_all_busy;
	
	// Write-after-write hazard: store conflicts with pending store
	assign waw_hazard = store_issue & store_conflict_stall;

	// Normal stall: any hazard that prevents instruction issue
	assign normal_stall = ls_addr_unresolved | struct_hazard | waw_hazard;


	// ----- Functional Units (Execution Stage) -----
	// Each functional unit contains reservation stations and execution logic
	// ALU unit - handles arithmetic, logic, and shift operations
	unit_ALU alu(
		.clk(debug_clk),
		.rst(rst),
		.issue(ALU_issue & ~normal_stall),  // Only issue when no stalls
		.cdb(cdb),                         // Common data bus for operand updates
		.ALUControl_in(ALU_op),            // ALU operation type
		// Input operands with tags
		.q1_in(ALUA[39:32]),               // Input A tag
		.v1_in(ALUA[31:0]),                // Input A value
		.q2_in(ALUB[39:32]),               // Input B tag
		.v2_in(ALUB[31:0]),                // Input B value
		// Status and result signals
		.all_busy(ALU_all_busy),           // All reservation stations busy
		.cdb_request(ALU_cdb_request),     // Request to broadcast result
		.cdb_out(ALU_cdb_in),              // Result data for CDB
		.issue_tag(ALU_issue_tag)           // Tag assigned to this operation
	);

	// Multiplication unit - handles multiply operations
	unit_mul mul(
		.clk(debug_clk),
		.rst(rst),
		.issue(mul_issue & ~normal_stall),  // Only issue when no stalls
		.cdb(cdb),                         // Common data bus for operand updates
		// Input operands with tags (directly from register file)
		.q1_in(rs1_data[39:32]),           // Input A tag
		.v1_in(rs1_data[31:0]),            // Input A value
		.q2_in(rs2_data[39:32]),           // Input B tag
		.v2_in(rs2_data[31:0]),            // Input B value
		// Status and result signals
		.all_busy(mul_all_busy),           // All reservation stations busy
		.cdb_request(mul_cdb_request),     // Request to broadcast result
		.cdb_out(mul_cdb_in),              // Result data for CDB
		.issue_tag(mul_issue_tag)           // Tag assigned to this operation
	);

	// Division unit - handles divide operations
	unit_div div(
		.clk(debug_clk),
		.rst(rst),
		.issue(div_issue & ~normal_stall),  // Only issue when no stalls
		.cdb(cdb),                         // Common data bus for operand updates
		// Input operands with tags (directly from register file)
		.q1_in(rs1_data[39:32]),           // Input A tag
		.v1_in(rs1_data[31:0]),            // Input A value
		.q2_in(rs2_data[39:32]),           // Input B tag
		.v2_in(rs2_data[31:0]),            // Input B value
		// Status and result signals
		.all_busy(div_all_busy),           // All reservation stations busy
		.cdb_request(div_cdb_request),     // Request to broadcast result
		.cdb_out(div_cdb_in),              // Result data for CDB
		.issue_tag(div_issue_tag)           // Tag assigned to this operation
	);

	// Load/Store unit - handles memory operations
	unit_load_store ls(
		.clk(debug_clk),
		.rst(rst),
		.cdb(cdb),                         // Common data bus for operand updates
		.cdb_request(load_cdb_request),     // Request to broadcast load result
		.cdb_out(load_cdb_in),              // Load result data for CDB
		// Memory operation details
		.ls_addr_in(ls_addr),               // Calculated memory address
		.ls_u_b_h_w_in(inst_ID[14:12]),     // Size/sign: byte/half/word, signed/unsigned
		// Store operation signals
		.store_issue(store_issue & ~normal_stall),  // Issue store operation
		.store_q_data_in(rs2_data[39:32]),  // Store data tag
		.store_data_in(rs2_data[31:0]),     // Store data value
		.store_all_busy(store_all_busy),    // All store buffer entries busy
		// Load operation signals
		.load_issue(load_issue & ~normal_stall),    // Issue load operation
		.load_all_busy(load_all_busy),      // All load reservation stations busy
		.load_issue_tag(load_issue_tag),    // Tag assigned to load operation
		// Hazard detection
		.store_conflict_stall(store_conflict_stall)  // Store conflicts with pending load/store
	);


	// ----- Common Data Bus (CDB) -----
	// Arbitrates between multiple functional units requesting to broadcast results
	common_data_bus bus(
		.clk(debug_clk),
		.rst(rst),
		.cdb(cdb),                         // Output: combined CDB signal
		// ALU unit interface
		.ALU_cdb_request(ALU_cdb_request),  // ALU requesting broadcast
		.ALU_cdb_in(ALU_cdb_in),           // ALU result data
		// Multiply unit interface
		.mul_cdb_request(mul_cdb_request),  // Multiplier requesting broadcast
		.mul_cdb_in(mul_cdb_in),           // Multiply result data
		// Divide unit interface
		.div_cdb_request(div_cdb_request),  // Divider requesting broadcast
		.div_cdb_in(div_cdb_in),           // Divide result data
		// Load/Store unit interface
		.ls_cdb_request(load_cdb_request),  // Load unit requesting broadcast
		.ls_cdb_in(load_cdb_in)            // Load result data
	);



	// ----- Debug Signal Multiplexer -----
	// Selects internal signals for observation based on debug_addr
	always @* begin
		case (debug_addr)
			// ----- IF stage signals (32-39) -----
			32: Test_signal = PC_IF;                // Program counter
			33: Test_signal = inst_IF;              // Fetched instruction
			34: Test_signal = PC_ID;                // ID stage PC
			35: Test_signal = inst_ID;              // Decoded instruction
			36: Test_signal = ls_addr_unresolved;    // Load/store address hazard
			37: Test_signal = struct_hazard;         // Structural hazard
			38: Test_signal = normal_stall;          // Pipeline stall
			39: Test_signal = jump_stall;            // Branch/jump stall

			// ----- Control signals (40-47) -----
			40: Test_signal = to_jump;               // Branch/jump taken flag
			41: Test_signal = ALUSrcA;               // ALU input A select
			42: Test_signal = ALUSrcB;               // ALU input B select
			43: Test_signal = FU_regWrite;           // Register write enable
			44: Test_signal = ALU_issue;             // ALU issue flag
			45: Test_signal = mul_issue;             // Multiply issue flag
			46: Test_signal = div_issue;             // Divide issue flag
			47: Test_signal = load_issue;            // Load issue flag

			// ----- More control and data signals (48-55) -----
			48: Test_signal = store_issue;           // Store issue flag
			49: Test_signal = ujump_issue;           // Unconditional jump flag
			50: Test_signal = branch_issue;          // Branch issue flag
			51: Test_signal = ImmSel;                // Immediate format select
			52: Test_signal = ALU_op;                // ALU operation code
			53: Test_signal = JUMP_op;               // Jump operation code
			54: Test_signal = Imm;                   // Generated immediate
			55: Test_signal = ls_addr;               // Load/store address

			// ----- Register and operand values (56-63) -----
			56: Test_signal = inst_ID[19:15];        // rs1 address field
			57: Test_signal = rs1_data;              // rs1 value with tag
			58: Test_signal = inst_ID[24:20];        // rs2 address field
			59: Test_signal = rs2_data;              // rs2 value with tag
			60: Test_signal = ALUA;                  // ALU input A
			61: Test_signal = ALUB;                  // ALU input B
			62: Test_signal = inst_ID[11:7];         // rd address field
			63: Test_signal = PC_jump;               // Jump target address

			// ----- CDB request signals (64-71) -----
			64: Test_signal = ALU_cdb_request;       // ALU requesting CDB
			65: Test_signal = mul_cdb_request;       // Multiply requesting CDB
			66: Test_signal = div_cdb_request;       // Divide requesting CDB
			67: Test_signal = load_cdb_request;      // Load requesting CDB
			68: Test_signal = ALU_cdb_in;            // ALU result
			69: Test_signal = mul_cdb_in;            // Multiply result
			70: Test_signal = div_cdb_in;            // Divide result
			71: Test_signal = load_cdb_in;           // Load result

			// ----- Tag and CDB values (72-78) -----
			72: Test_signal = ALU_issue_tag;         // ALU operation tag
			73: Test_signal = mul_issue_tag;         // Multiply operation tag
			74: Test_signal = div_issue_tag;         // Divide operation tag
			75: Test_signal = load_issue_tag;        // Load operation tag
			76: Test_signal = reg_w_tag;             // Register write tag
			77: Test_signal = cdb[40] ? 40'h11111_11111 : 40'h0;  // CDB valid bit
			78: Test_signal = cdb[39:0];            // CDB tag and data

			// Default pattern for invalid addresses
			default: Test_signal = 40'h8AA55_AA558;   // Recognizable pattern
		endcase
	end

endmodule