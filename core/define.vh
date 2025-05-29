`define NUM_RS     3
`define NUM_FU     5
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
