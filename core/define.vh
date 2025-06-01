`define NUM_RS     3
`define NUM_FU     6      // increased from 5 to add branch unit
`define NUM_SRBITS 8
`define NUM_CDBBITS 41

`define CDB_ON_FIELD   40
`define CDB_TAG_FIELD  39:32
`define CDB_FU_FIELD   39:35
`define CDB_RS_FIELD   34:32
`define CDB_DATA_FIELD 31:0

`define FU_ALU_TAG   5'b10000
`define FU_MUL_TAG   5'b01000
`define FU_DIV_TAG   5'b00100
`define FU_LOAD_TAG  5'b00010
`define FU_STORE_TAG 5'b00001
`define FU_BRANCH_TAG 5'b00011    // branch/jump unit tag

typedef struct packed {
    logic debug_en;
    logic debug_step;
    logic [6:0] debug_addr;
    logic [39:0] debug_data; // This is an output from the module, consider if it fits here or stays separate
    logic debug_clk;
    logic [39:0] debug_regs;
} debug_if_t;

typedef struct packed {
    logic [31:0] pc;
    logic [31:0] inst;
} if_stage_t;

typedef struct packed {
    logic [31:0] pc;
    logic [31:0] inst;
    logic [31:0] imm;
    logic [31:0] ls_addr;
    logic ALUSrcA;
    logic ALUSrcB;
    logic FU_regWrite;
    logic [2:0] ImmSel;
    logic [3:0] ALU_op;
    logic [3:0] JUMP_op;
} id_stage_t;

typedef struct packed {
    logic ALU_issue;
    logic mul_issue;
    logic div_issue;
    logic load_issue;
    logic store_issue;
    logic ujump_issue;
    logic branch_issue;
} dispatch_signals_t;

typedef struct packed {
    logic [7:0] tag;
    logic [31:0] val;
} tagged_data_t;

typedef struct packed {
    logic ls_addr_unresolved;
    logic struct_hazard;
    logic waw_hazard;
    logic normal_stall;
    logic to_jump;
    logic jump_stall;
} hazard_stall_ctrl_t;

typedef struct packed {
    logic pred_taken;
    logic [31:0] pred_target;
    logic use_misp;
    logic [31:0] fetch_pc_pred;
} branch_pred_ctrl_t;

typedef struct packed {
    logic alu;
    logic mul;
    logic div;
    logic load;
    logic branch;
} cdb_fu_request_t;

typedef struct packed {
    tagged_data_t alu;
    tagged_data_t mul;
    tagged_data_t div;
    tagged_data_t load;
    tagged_data_t branch;
} cdb_fu_input_t;

typedef struct packed {
    logic valid;
    logic [7:0] tag;
    logic [31:0] data;
} cdb_bus_t;

typedef struct packed {
    logic ALU_all_busy;
    logic mul_all_busy;
    logic div_all_busy;
    logic load_all_busy;
    logic store_all_busy;
    logic store_conflict_stall;
} rs_status_t;

typedef struct packed {
    logic [7:0] ALU;
    logic [7:0] mul;
    logic [7:0] div;
    logic [7:0] load;
    logic [7:0] branch;
    logic [7:0] reg_w_tag; // This is the selected write tag
} rs_issue_tags_t;
