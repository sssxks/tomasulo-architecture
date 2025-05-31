# Tomasulo 架构代码详细分析

本文档提供了 Tomasulo 架构实现的详细逐行分析，按照指令生命周期进行组织。

## 1. 常量和信号定义 (define.vh)

首先分析 `define.vh` 文件，它定义了整个架构中使用的关键常量和信号字段：

```verilog
`define NUM_RS     3        // 每种功能单元的保留站数量
`define NUM_FU     5        // 功能单元类型数量
`define NUM_SRBITS 8        // 标记位宽
`define NUM_CDBBITS 41      // 公共数据总线位宽

// CDB 字段定义
`define CDB_ON_FIELD   40   // CDB 有效位
`define CDB_TAG_FIELD  39:32 // 标记字段
`define CDB_FU_FIELD   39:35 // 功能单元类型字段
`define CDB_RS_FIELD   34:32 // 保留站编号字段
`define CDB_DATA_FIELD 31:0  // 数据字段

// 功能单元类型标记
`define FU_ALU_TAG   5'b10000  // ALU 功能单元标记
`define FU_MUL_TAG   5'b01000  // 乘法功能单元标记
`define FU_DIV_TAG   5'b00100  // 除法功能单元标记
`define FU_LOAD_TAG  5'b00010  // 加载功能单元标记
`define FU_STORE_TAG 5'b00001  // 存储功能单元标记
```

这些定义为整个架构提供了基础配置，包括保留站数量、功能单元类型、标记位宽、公共数据总线结构等。

## 2. 顶层模块 (RV32core.v)

顶层模块 `RV32core` 集成了整个处理器的所有组件，按照指令生命周期分析其结构：

```verilog
module RV32core(
    input debug_en,         // 调试使能
    input debug_step,       // 调试步进时钟
    input [6:0] debug_addr, // 调试地址
    output[39:0] debug_data,// 调试数据
    input clk,              // 主时钟
    input rst               // 同步复位
);
```

顶层模块端口包括调试接口和基本时钟/复位信号。

### 2.1 调试和时钟控制

```verilog
wire debug_clk;
wire[39:0] debug_regs;
reg[39:0] Test_signal;
assign debug_data = |debug_addr[6:5] ? Test_signal : debug_regs;

debug_clk clock(.clk(clk),.debug_en(debug_en),.debug_step(debug_step),.debug_clk(debug_clk));
```

这部分实现了调试时钟控制和调试数据选择逻辑。

### 2.2 流水线控制信号

```verilog
wire ls_addr_unresolved, struct_hazard, waw_hazard, normal_stall, to_jump, jump_stall;
```

这些信号用于控制流水线停顿和跳转：
- `ls_addr_unresolved`: 加载/存储地址未解析
- `struct_hazard`: 结构冒险
- `waw_hazard`: 写后写冒险
- `normal_stall`: 普通停顿
- `to_jump`: 跳转信号
- `jump_stall`: 跳转停顿

### 2.3 取指阶段 (IF)

```verilog
wire[31:0] PC_IF, next_PC_IF, PCp4_IF, PC_jump, inst_IF;

// IF
REG32 REG_PC(.clk(debug_clk),.rst(rst),.CE(~normal_stall & ~jump_stall),.D(next_PC_IF),.Q(PC_IF));

add_32 add_IF(.a(PC_IF),.b(32'd4),.c(PCp4_IF));

MUX2T1_32 mux_IF(.I0(PCp4_IF),.I1(PC_jump),.s(to_jump),.o(next_PC_IF));

ROM_D inst_rom(.a(PC_IF[8:2]),.spo(inst_IF));
```

取指阶段实现：
1. `REG_PC`: PC寄存器，在无停顿时更新为下一条指令地址
2. `add_IF`: PC+4加法器，计算顺序执行的下一条指令地址
3. `mux_IF`: PC选择器，根据跳转信号选择PC+4或跳转地址
4. `inst_rom`: 指令存储器，根据PC读取指令

### 2.4 译码/发射阶段 (ID/Issue)

```verilog
wire ALUSrcA, ALUSrcB, FU_regWrite;
wire ALU_issue, mul_issue, div_issue, load_issue, store_issue, ujump_issue, branch_issue;
wire[2:0] ImmSel;
wire[3:0] ALU_op, JUMP_op;
wire[31:0] inst_ID, PC_ID, Imm, ls_addr;
wire[39:0] rs1_data, rs2_data, ALUA, ALUB;

//Issue
REG_ID reg_ID(.clk(debug_clk),.rst(rst),.EN(~normal_stall & ~jump_stall),
    .flush(to_jump & ~jump_stall),.PCOUT(PC_IF),.IR(inst_IF),
    .IR_ID(inst_ID),.PCurrent_ID(PC_ID),.valid());

DecodeIssue issue(.inst(inst_ID),.ujump_issue(ujump_issue),.branch_issue(branch_issue),
    .ALU_issue(ALU_issue),.mul_issue(mul_issue),.div_issue(div_issue),.load_issue(load_issue),.store_issue(store_issue),
    .ImmSel(ImmSel),.ALU_op(ALU_op),.JUMP_op(JUMP_op),.ALUSrcA(ALUSrcA),.ALUSrcB(ALUSrcB),.FU_regWrite(FU_regWrite));

ImmGen imm_gen(.ImmSel(ImmSel),.inst_field(inst_ID),.Imm_out(Imm));
```

译码/发射阶段实现：
1. `reg_ID`: IF/ID流水线寄存器，在无停顿时传递PC和指令
2. `issue`: 指令译码和发射单元，解析指令类型并生成控制信号
3. `imm_gen`: 立即数生成器，根据指令类型生成立即数

### 2.5 标记分配和寄存器访问

```verilog
tristate #(8) alu_tag(.dout(reg_w_tag),.din(ALU_issue_tag),.en(ALU_issue));
tristate #(8) mul_tag(.dout(reg_w_tag),.din(mul_issue_tag),.en(mul_issue));
tristate #(8) div_tag(.dout(reg_w_tag),.din(div_issue_tag),.en(div_issue));
tristate #(8) load_tag(.dout(reg_w_tag),.din(load_issue_tag),.en(load_issue));

taggedRegs tregs(.clk(debug_clk),.rst(rst),.FU_regWrite(FU_regWrite & ~normal_stall),.ujump_wb(ujump_issue & ~jump_stall),
    .raddr_A(inst_ID[19:15]),.rdata_A(rs1_data),.raddr_B(inst_ID[24:20]),.rdata_B(rs2_data),
    .waddr(inst_ID[11:7]),.w_tag(reg_w_tag),.cdb(cdb),.PC(PC_ID),
    .Debug_addr(debug_addr[4:0]),.Debug_regs(debug_regs));
```

标记分配和寄存器访问实现：
1. 三态缓冲器用于根据指令类型选择适当的标记
2. `taggedRegs`: 带标记的寄存器文件，支持读取源操作数和写入目标寄存器

### 2.6 操作数准备

```verilog
MUX2T1_40 mux_imm_ALU_A(.I0(rs1_data),.I1({8'd0,PC_ID}),.s(ALUSrcA),.o(ALUA));
MUX2T1_40 mux_imm_ALU_B(.I0(rs2_data),.I1({8'd0,Imm}),.s(ALUSrcB),.o(ALUB));
```

操作数准备实现：
1. `mux_imm_ALU_A`: 选择ALU的A输入（寄存器值或PC）
2. `mux_imm_ALU_B`: 选择ALU的B输入（寄存器值或立即数）

### 2.7 跳转处理

```verilog
JumpHandle jh(.branch_issue(branch_issue),.ujump_issue(ujump_issue),.JUMP_op(JUMP_op),
    .q_rs1_in(rs1_data[39:32]),.q_rs2_in(rs2_data[39:32]),.rs1_data_in(rs1_data[31:0]),.rs2_data_in(rs2_data[31:0]),
    .imm(Imm),.PC(PC_ID),.cdb(cdb),.PC_jump(PC_jump),.to_jump(to_jump),.jump_stall(jump_stall));

add_32 add_addr(.a(rs1_data[31:0]),.b(Imm),.c(ls_addr));
```

跳转处理实现：
1. `jh`: 跳转处理单元，处理条件分支和无条件跳转
2. `add_addr`: 计算加载/存储指令的地址

### 2.8 停顿控制

```verilog
assign ls_addr_unresolved = |rs1_data[39:32] & (load_issue | store_issue);

assign struct_hazard =  ALU_issue & ALU_all_busy |
                        mul_issue & mul_all_busy |
                        div_issue & div_all_busy |
                        load_issue & load_all_busy |
                        store_issue & store_all_busy;

assign waw_hazard = store_issue & store_conflict_stall;

assign normal_stall = ls_addr_unresolved | struct_hazard | waw_hazard;
```

停顿控制逻辑：
1. `ls_addr_unresolved`: 加载/存储地址未解析时停顿
2. `struct_hazard`: 功能单元全部忙碌时停顿
3. `waw_hazard`: 存储冲突时停顿
4. `normal_stall`: 综合以上三种情况的停顿信号

### 2.9 功能单元 (FU)

```verilog
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
```

功能单元实现：
1. `unit_ALU`: ALU功能单元，处理算术逻辑运算
2. `unit_mul`: 乘法功能单元
3. `unit_div`: 除法功能单元
4. `unit_load_store`: 加载/存储功能单元

### 2.10 公共数据总线 (CDB)

```verilog
// CDB
common_data_bus bus(.clk(debug_clk),.rst(rst),.cdb(cdb),
    .ALU_cdb_request(ALU_cdb_request),.ALU_cdb_in(ALU_cdb_in),
    .mul_cdb_request(mul_cdb_request),.mul_cdb_in(mul_cdb_in),
    .div_cdb_request(div_cdb_request),.div_cdb_in(div_cdb_in),
    .ls_cdb_request(load_cdb_request),.ls_cdb_in(load_cdb_in));
```

公共数据总线实现：
- `bus`: 公共数据总线，仲裁各功能单元的结果并广播

### 2.11 调试信号

```verilog
always @* begin
    case (debug_addr)
        32: Test_signal = PC_IF;
        33: Test_signal = inst_IF;
        // ... 更多调试信号
        default: Test_signal = 40'h8AA55_AA558;
    endcase
end
```

调试信号选择逻辑，根据调试地址选择不同的内部信号进行观察。

## 3. 译码和发射 (DecodeIssue.v)

译码和发射模块负责解析指令并生成控制信号：

```verilog
module DecodeIssue(
    input[31:0] inst,
    output ALU_issue, mul_issue, div_issue, load_issue, store_issue,
    output ujump_issue, branch_issue,
    output[2:0] ImmSel,
    output[3:0] ALU_op,
    output[3:0] JUMP_op,
    output ALUSrcA,
    output ALUSrcB,
    output FU_regWrite
);
```

### 3.1 指令字段提取

```verilog
// instruction field
wire [6:0] funct7 = inst[31:25];
wire [2:0] funct3 = inst[14:12];
wire [6:0] opcode = inst[6:0];
wire [4:0] rd = inst[11:7];
wire [4:0] rs1 = inst[19:15];
wire [4:0] rs2 = inst[24:20];
```

从指令中提取各个字段。

### 3.2 指令类型识别

```verilog
// type specification
wire Rop = opcode == 7'b0110011;
wire Iop = opcode == 7'b0010011;
wire Bop = opcode == 7'b1100011;
wire Lop = opcode == 7'b0000011;
wire Sop = opcode == 7'b0100011;

wire funct7_0  = funct7 == 7'h0;
wire funct7_1  = funct7 == 7'h1;
wire funct7_32 = funct7 == 7'h20;

wire funct3_0 = funct3 == 3'h0;
// ... 更多funct3识别
```

识别指令类型和功能码。

### 3.3 具体指令识别

```verilog
wire ADD  = Rop & funct3_0 & funct7_0;
wire SUB  = Rop & funct3_0 & funct7_32;
// ... 更多R型指令

wire MUL    = Rop & funct3_0 & funct7_1;
// ... 更多乘除法指令

wire ADDI  = Iop & funct3_0;
// ... 更多I型指令

wire BEQ = Bop & funct3_0;
// ... 更多B型指令

wire LB =  Lop & funct3_0;
// ... 更多加载指令

wire SB = Sop & funct3_0;
// ... 更多存储指令

wire LUI   = opcode == 7'b0110111;
wire AUIPC = opcode == 7'b0010111;

wire JAL  =  opcode == 7'b1101111;
wire JALR = (opcode == 7'b1100111) && funct3_0;
```

识别所有具体指令。

### 3.4 发射信号生成

```verilog
wire R_valid = AND | OR | ADD | XOR | SLL | SRL | SRA | SUB | SLT | SLTU
    | MUL | MULH | MULHSU | MULHU | DIV | DIVU | REM | REMU;
wire I_valid = ANDI | ORI | ADDI | XORI | SLLI | SRLI | SRAI | SLTI | SLTIU;
wire B_valid = BEQ | BNE | BLT | BGE | BLTU | BGEU;
wire L_valid = LW | LH | LB | LHU | LBU;
wire S_valid = SW | SH | SB;

assign ALU_issue = (AND | OR | ADD | XOR | SLL | SRL | SRA | SUB | SLT | SLTU
    | I_valid | LUI | AUIPC) & |rd;
assign mul_issue = (MUL | MULH | MULHSU | MULHU) & |rd;
assign div_issue = (DIV | DIVU | REM | REMU) & |rd;
assign load_issue = L_valid & |rd;
assign store_issue = S_valid;
assign branch_issue = B_valid;
assign ujump_issue = JAL | JALR;
```

生成各类指令的发射信号。

### 3.5 立即数类型选择

```verilog
localparam Imm_type_I = 3'b001;
localparam Imm_type_B = 3'b010;
localparam Imm_type_J = 3'b011;
localparam Imm_type_S = 3'b100;
localparam Imm_type_U = 3'b101;
assign ImmSel = {3{JALR | L_valid | I_valid}} & Imm_type_I |
                {3{B_valid}}                  & Imm_type_B |
                {3{JAL}}                      & Imm_type_J |
                {3{S_valid}}                  & Imm_type_S |
                {3{LUI | AUIPC}}              & Imm_type_U ;
```

根据指令类型选择立即数格式。

### 3.6 跳转操作码生成

```verilog
localparam JUMP_BEQ  = 4'b0_001;
// ... 更多跳转操作码
assign JUMP_op ={4{BEQ}}  & JUMP_BEQ  |
                {4{BNE}}  & JUMP_BNE  |
                // ... 更多跳转类型
                {4{JALR}} & JUMP_JALR ;
```

生成跳转操作码。

### 3.7 ALU操作码生成

```verilog
localparam ALU_ADD  = 4'b0001;
// ... 更多ALU操作码
assign ALU_op = {4{ADD | ADDI | AUIPC}} & ALU_ADD  |
                {4{SUB}}                & ALU_SUB  |
                // ... 更多ALU操作
                {4{LUI}}                & ALU_Bout ;
```

生成ALU操作码。

### 3.8 操作数选择和寄存器写使能

```verilog
assign ALUSrcA = AUIPC;
assign ALUSrcB = I_valid | LUI | AUIPC;
assign FU_regWrite = ALU_issue | mul_issue | div_issue | load_issue;
```

生成操作数选择信号和寄存器写使能信号。

## 4. 立即数生成 (ImmGen.v)

立即数生成模块负责从指令中提取并扩展立即数：

```verilog
module ImmGen(
    input  wire [2:0] ImmSel,
    input  wire [31:0] inst_field,
    output[31:0] Imm_out
);
```

### 4.1 立即数类型定义

```verilog
parameter Imm_type_I = 3'b001;
parameter Imm_type_B = 3'b010;
parameter Imm_type_J = 3'b011;
parameter Imm_type_S = 3'b100;
parameter Imm_type_U = 3'b101;

wire I = ImmSel == Imm_type_I;
wire B = ImmSel == Imm_type_B;
wire J = ImmSel == Imm_type_J;
wire S = ImmSel == Imm_type_S;
wire U = ImmSel == Imm_type_U;
```

定义并识别各种立即数类型。

### 4.2 立即数提取和符号扩展

```verilog
wire[31:0] Imm_I = {{20{inst_field[31]}}, inst_field[31:20]};
wire[31:0] Imm_B = {{20{inst_field[31]}}, inst_field[7], inst_field[30:25], inst_field[11:8], 1'b0};
wire[31:0] Imm_J = {{12{inst_field[31]}}, inst_field[19:12], inst_field[20], inst_field[30:21],1'b0};
wire[31:0] Imm_S = {{20{inst_field[31]}}, inst_field[31:25], inst_field[11:7]};
wire[31:0] Imm_U = {inst_field[31:12], 12'b0};
```

根据RISC-V规范从指令中提取并符号扩展各类立即数。

### 4.3 立即数选择

```verilog
assign Imm_out = {32{I}} & Imm_I |
                 {32{B}} & Imm_B |
                 {32{J}} & Imm_J |
                 {32{S}} & Imm_S |
                 {32{U}} & Imm_U ;
```

根据立即数类型选择最终输出的立即数。

## 5. 带标记的寄存器文件 (taggedRegs.v)

带标记的寄存器文件是Tomasulo算法的核心组件之一：

```verilog
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
```

### 5.1 寄存器和标记存储

```verilog
reg [31:0] register [1:31];
reg[`NUM_SRBITS-1:0] tags [1:31];
```

存储32个寄存器的值和对应的标记。

### 5.2 寄存器读取

```verilog
assign rdata_A = (raddr_A == 0) ? 40'b0 : {tags[raddr_A], register[raddr_A]};
assign rdata_B = (raddr_B == 0) ? 40'b0 : {tags[raddr_B], register[raddr_B]};
```

读取寄存器值和标记，x0寄存器始终返回0。

### 5.3 标记和寄存器更新

```verilog
wire addr_match[1:31];
wire cdb_match[1:31];

genvar i;
generate
    for(i=1;i<32;i=i+1) begin
        assign addr_match[i] = waddr == i;
        assign cdb_match[i] = (cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == tags[i]);

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
```

标记和寄存器更新逻辑：
1. 复位时清零
2. 发射指令时设置标记
3. CDB匹配或无条件跳转时清除标记
4. 无条件跳转时写入PC+4
5. CDB匹配时写入CDB数据

### 5.4 调试接口

```verilog
assign Debug_regs = (Debug_addr == 0) ? 40'b0 : {tags[Debug_addr], register[Debug_addr]};
```

提供调试接口读取寄存器值和标记。

## 6. 跳转处理 (JumpHandle.v)

跳转处理模块负责处理条件分支和无条件跳转：

```verilog
module JumpHandle(
    input branch_issue, ujump_issue,
    input[3:0]JUMP_op,
    input[`NUM_SRBITS-1:0] q_rs1_in, q_rs2_in,
    input[31:0] rs1_data_in, rs2_data_in, imm, PC,
    input[`NUM_CDBBITS-1:0] cdb,
    output[31:0] PC_jump,
    output to_jump,
    output jump_stall
);
```

### 6.1 跳转类型识别

```verilog
wire JALR = ujump_issue & JUMP_op[3];
wire JAL = ujump_issue & ~JUMP_op[3];
```

识别JALR和JAL指令。

### 6.2 操作数前递

```verilog
wire cdb_match1 = cdb[`CDB_ON_FIELD] && q_rs1_in == cdb[`CDB_TAG_FIELD];
wire cdb_match2 = cdb[`CDB_ON_FIELD] && q_rs2_in == cdb[`CDB_TAG_FIELD];

wire[31:0] rs1_data = cdb_match1 ? cdb[`CDB_DATA_FIELD] : rs1_data_in;
wire[31:0] rs2_data = cdb_match2 ? cdb[`CDB_DATA_FIELD] : rs2_data_in;
```

从CDB获取最新的操作数值。

### 6.3 条件比较和跳转地址计算

```verilog
wire cmp_res;
cmp_32 cmp(.a(rs1_data),.b(rs2_data),.ctrl(JUMP_op[2:0]),.c(cmp_res));

add_32 a(.a(JALR ? rs1_data : PC),.b(imm),.c(PC_jump));
```

条件比较和跳转地址计算：
1. `cmp`: 比较两个操作数
2. `a`: 计算跳转地址（JALR使用rs1+imm，其他使用PC+imm）

### 6.4 跳转控制

```verilog
assign jump_stall = |q_rs1_in & ~cdb_match1 & (branch_issue | JALR) |
                    |q_rs2_in & ~cdb_match2 & branch_issue;

assign to_jump = ujump_issue | branch_issue & cmp_res;
```

跳转控制逻辑：
1. `jump_stall`: 操作数未就绪时停顿
2. `to_jump`: 无条件跳转或条件满足时跳转

## 7. 公共数据总线 (common_data_bus.v)

公共数据总线负责仲裁和广播功能单元的结果：

```verilog
module common_data_bus(
    input clk, rst,
    input ALU_cdb_request, mul_cdb_request, div_cdb_request, ls_cdb_request,
    input[39:0] ALU_cdb_in, mul_cdb_in, div_cdb_in, ls_cdb_in,
    output reg[40:0] cdb
);
```

### 7.1 仲裁逻辑

```verilog
// Arbitration: prioritize ALU > mul > div > ld/st
always @(posedge clk or posedge rst) begin
    if (rst) begin
        cdb <= {1'b0, 40'b0};
    end else if (ALU_cdb_request) begin
        cdb <= {1'b1, ALU_cdb_in};
    end else if (mul_cdb_request) begin
        cdb <= {1'b1, mul_cdb_in};
    end else if (div_cdb_request) begin
        cdb <= {1'b1, div_cdb_in};
    end else if (ls_cdb_request) begin
        cdb <= {1'b1, ls_cdb_in};
    end else begin
        cdb <= {1'b0, 40'b0};
    end
end
```

仲裁逻辑按优先级选择功能单元的结果：ALU > 乘法 > 除法 > 加载/存储。

## 8. 保留站 (RS_*.v)

### 8.1 通用保留站行 (RS_generic_line.v)

通用保留站行是所有保留站的基础：

```verilog
module RS_generic_line(
    input clk, rst, issue,
    input FU_result_taken,
    input[40:0] cdb,
    input[7:0]  q1_in, q2_in,
    input[31:0] v1_in, v2_in,
    output data_ready,
    output reg busy,
    output reg[31:0] v1, v2
);
```

#### 8.1.1 标记和数据存储

```verilog
reg[7:0]  q1, q2;
```

存储两个操作数的标记。

#### 8.1.2 CDB匹配逻辑

```verilog
wire cdb_match_q1 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q1;
wire cdb_match_q2 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q2;

wire init_clk_cdb_match_q1 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q1_in;
wire init_clk_cdb_match_q2 = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q2_in;
```

检测CDB是否匹配当前标记或输入标记。

#### 8.1.3 数据就绪信号

```verilog
assign data_ready = busy && q1 == 8'b0 && q2 == 8'b0;
```

当保留站忙且两个操作数都就绪时，数据就绪。

#### 8.1.4 忙状态控制

```verilog
always@(posedge clk or posedge rst) begin
    if(rst) busy <= 1'b0;
    else if(issue) busy <= 1'b1;
    else if(FU_result_taken) busy <= 1'b0;
end
```

忙状态控制：发射时置忙，结果被取走时清忙。

#### 8.1.5 操作数更新

```verilog
always@(posedge clk or posedge rst) begin
    if(rst) begin
        q1 <= 8'b0;
        v1 <= 32'b0;
    end
    else if(issue) begin
        if(init_clk_cdb_match_q1) begin
            q1 <= 8'b0;
            v1 <= cdb[`CDB_DATA_FIELD];
        end
        else begin
            q1 <= q1_in;
            v1 <= v1_in;
        end
    end
    else if(cdb_match_q1) begin
        q1 <= 8'b0;
        v1 <= cdb[`CDB_DATA_FIELD];
    end
end
```

操作数更新逻辑（q2类似）：
1. 复位时清零
2. 发射时，如果CDB匹配输入标记，则直接使用CDB数据
3. 否则，使用输入标记和数据
4. 非发射时，如果CDB匹配当前标记，则更新数据并清除标记

### 8.2 ALU保留站行 (RS_ALU_line.v)

ALU保留站行扩展了通用保留站行，增加了ALU控制信号：

```verilog
module RS_ALU_line(
    input clk, rst, issue,
    input FU_result_taken,
    input[40:0] cdb,
    input[3:0] ALUControl_in,
    input[7:0]  q1_in, q2_in,
    input[31:0] v1_in, v2_in,
    output data_ready,
    output busy,
    output reg[3:0] ALUControl,
    output [31:0] v1, v2
);
```

#### 8.2.1 通用保留站实例化

```verilog
RS_generic_line rs_g(.clk(clk),.rst(rst),.issue(issue),.FU_result_taken(FU_result_taken),
    .cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),.v1_in(v1_in),.v2_in(v2_in),
    .data_ready(data_ready),.busy(busy),.v1(v1),.v2(v2));
```

实例化通用保留站行。

#### 8.2.2 ALU控制信号存储

```verilog
always@(posedge clk or posedge rst)begin
    if(rst) ALUControl <= 4'b0;
    else if(issue) ALUControl <= ALUControl_in;
end
```

存储ALU控制信号。

### 8.3 加载保留站行 (RS_load_line.v)

加载保留站行存储加载指令的地址和类型：

```verilog
module RS_load_line(
    input clk, rst, issue,
    input FU_result_taken,
    input[40:0] cdb,
    input[31:0] addr_in,
    input[2:0] mem_u_b_h_w_in,
    output reg busy,
    output reg [31:0] addr,
    output reg[2:0] mem_u_b_h_w
);
```

#### 8.3.1 状态和数据存储

```verilog
always @(posedge clk or posedge rst) begin
    if (rst) begin
        busy <= 1'b0;
        addr <= 32'b0;
        mem_u_b_h_w <= 3'b0;
    end else if (issue) begin
        busy <= 1'b1;
        addr <= addr_in;
        mem_u_b_h_w <= mem_u_b_h_w_in;
    end else if (FU_result_taken) begin
        busy <= 1'b0;
    end
end
```

存储加载指令的状态、地址和类型。

### 8.4 存储保留站行 (RS_store_line.v)

存储保留站行存储存储指令的地址、数据和类型：

```verilog
module RS_store_line(
    input clk, rst, issue,
    input result_taken,
    input[40:0] cdb,
    input[7:0]  q_data_in,
    input[31:0] addr_in, data_in,
    input[2:0] mem_u_b_h_w_in,
    output data_ready,
    output reg busy,
    output reg [31:0] addr,
    output reg [31:0] data,
    output reg[2:0] mem_u_b_h_w
);
```

#### 8.4.1 标记和数据跟踪

```verilog
reg [7:0] q_data;
wire cdb_match_q = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q_data;
wire init_clk_cdb_match = cdb[`CDB_ON_FIELD] && cdb[`CDB_TAG_FIELD] == q_data_in;
```

跟踪数据标记和CDB匹配。

#### 8.4.2 数据就绪信号

```verilog
assign data_ready = busy && q_data == 8'b0;
```

当保留站忙且数据就绪时，数据就绪。

#### 8.4.3 忙状态控制

```verilog
always @(posedge clk or posedge rst) begin
    if (rst) busy <= 1'b0;
    else if (issue) busy <= 1'b1;
    else if (result_taken) busy <= 1'b0;
end
```

忙状态控制：发射时置忙，结果被取走时清忙。

#### 8.4.4 数据更新

```verilog
always @(posedge clk or posedge rst) begin
    if (rst) begin
        q_data <= 8'b0;
        data <= 32'b0;
    end else if (issue) begin
        if (init_clk_cdb_match) begin
            q_data <= 8'b0;
            data <= cdb[`CDB_DATA_FIELD];
        end else begin
            q_data <= q_data_in;
            data <= data_in;
        end
    end else if (cdb_match_q) begin
        q_data <= 8'b0;
        data <= cdb[`CDB_DATA_FIELD];
    end
end
```

数据更新逻辑：
1. 复位时清零
2. 发射时，如果CDB匹配输入标记，则直接使用CDB数据
3. 否则，使用输入标记和数据
4. 非发射时，如果CDB匹配当前标记，则更新数据并清除标记

#### 8.4.5 地址和类型更新

```verilog
always @(posedge clk or posedge rst) begin
    if (rst) begin
        addr <= 32'b0;
        mem_u_b_h_w <= 3'b0;
    end else if (issue) begin
        addr <= addr_in;
        mem_u_b_h_w <= mem_u_b_h_w_in;
    end
end
```

地址和类型更新逻辑。

## 9. 功能单元 (FU_*.v)

### 9.1 ALU功能单元 (FU_ALU.v)

ALU功能单元执行算术逻辑运算：

```verilog
module FU_ALU(
    input clk, EN,
    input[3:0] ALUControl,
    input[31:0] ALUA, ALUB,
    output[31:0] res,
    output zero, overflow,
    output finish
);
```

#### 9.1.1 状态控制

```verilog
reg state;
reg[3:0] Control;
reg[31:0] A, B;
assign finish = state == 1'b1;
initial begin
    state = 0;
    A = 0;
    B = 0;
    Control = 0;
end

always@(posedge clk) begin
    if(EN & ~state) begin // state == 0
        A <= ALUA;
        B <= ALUB;
        Control <= ALUControl;
        state <= 1;
    end
    else state <= 0;
end
```

状态控制：使能时锁存操作数和控制信号，并在下一个周期完成计算。

#### 9.1.2 ALU操作

```verilog
localparam ALU_ADD  = 4'b0001;
// ... 更多ALU操作码

wire[4:0] shamt = B[4:0];
wire[32:0] res_subu = {1'b0,A} - {1'b0,B};

wire[31:0] res_ADD  = A + B;
// ... 更多ALU操作结果

wire add_of = A[31] & B[31] & ~res_ADD[31] | // neg + neg = pos
              ~A[31] & ~B[31] & res_ADD[31]; // pos + pos = neg
wire sub_of = ~A[31] & B[31] & res_SUB[31] | // pos - neg = neg
              A[31] & ~B[31] & ~res_SUB[31]; // neg - pos = pos

// ... 更多ALU操作结果
```

计算各种ALU操作的结果和溢出标志。

#### 9.1.3 结果选择

```verilog
wire ADD  = Control == ALU_ADD ;
// ... 更多操作选择

assign zero = ~|res;

assign overflow = (Control == ALU_ADD && add_of) |
                (Control == ALU_SUB && sub_of);

assign res = {32{ADD }} & res_ADD  |
             // ... 更多结果选择
             {32{Bout}} & res_Bout ;
```

根据控制信号选择最终结果。

### 9.2 乘法功能单元 (FU_mul.v)

乘法功能单元执行乘法运算：

```verilog
module FU_mul(
    input clk, EN,
    input[31:0] A, B,
    output[31:0] res,
    output finish
);
```

#### 9.2.1 状态控制

```verilog
reg[6:0] state;
assign finish = state[0] == 1'b1;
initial begin
    state = 0;
end

reg[31:0] A_reg, B_reg;

always@(posedge clk) begin
    if(EN & ~|state) begin
        A_reg <= A;
        B_reg <= B;
        state <= 7'b100_0000;
    end
    else state <= {1'b0, state[6:1]};
end
```

状态控制：使能时锁存操作数，并在6个周期后完成计算。

#### 9.2.2 乘法计算

```verilog
wire[63:0] mulres;
mult_gen_0 mul(.CLK(clk),.A(A_reg),.B(B_reg),.P(mulres));
assign res = mulres[31:0];
```

使用乘法器IP核计算乘法结果。

### 9.3 除法功能单元 (FU_div.v)

除法功能单元执行除法运算：

```verilog
module FU_div(
    input clk, EN,
    input[31:0] A, B,
    output[31:0] res,
    output finish
);
```

#### 9.3.1 状态控制

```verilog
wire res_valid;
wire[63:0] divres;

reg state;
assign finish = res_valid & state;
initial begin
    state = 0;
end

reg A_valid, B_valid;
reg[31:0] A_reg, B_reg;

always@(posedge clk) begin
    if(EN & ~state) begin  // state == 0
        A_reg <= A;
        B_reg <= B;
        A_valid <= 1;
        B_valid <= 1;
        state <= 1;
    end
    else if(res_valid) begin
        A_valid <= 0;
        B_valid <= 0;
        state <= 0;
    end
end
```

状态控制：使能时锁存操作数，并在结果有效时完成计算。

#### 9.3.2 除法计算

```verilog
div_gen_0 div(.aclk(clk),
    .s_axis_dividend_tvalid(A_valid),
    .s_axis_dividend_tdata(A_reg),
    .s_axis_divisor_tvalid(B_valid),
    .s_axis_divisor_tdata(B_reg),
    .m_axis_dout_tvalid(res_valid),
    .m_axis_dout_tdata(divres)
);

assign res = divres[63:32];
```

使用除法器IP核计算除法结果。

## 10. 功能单元控制 (unit_*.v)

### 10.1 ALU功能单元控制 (unit_ALU.v)

ALU功能单元控制管理ALU保留站和功能单元：

```verilog
module unit_ALU(
    input clk, rst, issue,
    input[`NUM_CDBBITS-1:0]cdb,
    input[3:0] ALUControl_in,
    input[7:0]  q1_in, q2_in,
    input[31:0] v1_in, v2_in,
    output all_busy,
    output cdb_request,
    output[`NUM_CDBBITS-2:0] cdb_out,
    output[`NUM_SRBITS-1:0] issue_tag
);
```

#### 10.1.1 保留站管理

```verilog
reg rs1_FU_result_taken, rs2_FU_result_taken, rs3_FU_result_taken;
wire rs1_data_ready, rs2_data_ready, rs3_data_ready;
wire rs1_busy, rs2_busy, rs3_busy;
wire[3:0] rs1_ALUControl, rs2_ALUControl, rs3_ALUControl;
wire[31:0] rs1_v1, rs1_v2, rs2_v1, rs2_v2, rs3_v1, rs3_v2;

assign all_busy = rs1_busy & rs2_busy & rs3_busy;
wire rs1_issue = issue & ~rs1_busy;
wire rs2_issue = issue & rs1_busy & ~rs2_busy;
wire rs3_issue = issue & rs1_busy & rs2_busy & ~rs3_busy;
assign issue_tag = {`FU_ALU_TAG, rs1_issue, rs2_issue, rs3_issue} & {`NUM_SRBITS{issue}};

wire FU_result_taken = cdb[`CDB_ON_FIELD] && cdb[`CDB_FU_FIELD] == `FU_ALU_TAG;
```

保留站管理：
1. 跟踪三个保留站的状态
2. 生成全忙信号
3. 根据忙状态分配发射
4. 生成发射标记
5. 检测结果是否被取走

#### 10.1.2 保留站实例化

```verilog
RS_ALU_line rs1_alu(.clk(clk),.rst(rst),.issue(rs1_issue),.FU_result_taken(rs1_FU_result_taken),
    .cdb(cdb),.q1_in(q1_in),.q2_in(q2_in),.v1_in(v1_in),.v2_in(v2_in),
    .data_ready(rs1_data_ready),.busy(rs1_busy),.v1(rs1_v1),.v2(rs1_v2),
    .ALUControl_in(ALUControl_in),.ALUControl(rs1_ALUControl));

// ... rs2_alu和rs3_alu类似
```

实例化三个ALU保留站行。

#### 10.1.3 功能单元调度

```verilog
reg FU_ALU_EN;
reg[2:0] FU_poi;

wire FU_ALU_finish;
reg FU_ALU_finish_reg;
reg[3:0]FU_ALUControl;
reg[31:0] ALUA, ALUB;
wire[31:0] ALUout;
reg[31:0] ALUout_reg;

always@(negedge clk or posedge rst) begin
    if(rst) begin
        FU_poi <= 3'b0;
        ALUA <= 32'b0;
        ALUB <= 32'b0;
        FU_ALUControl <= 4'b0;
        FU_ALU_EN <= 1'b0;
    end

    else if(FU_result_taken || FU_poi == 3'b0) begin
        if(rs1_data_ready & ~FU_poi[2]) begin
            FU_poi <= 3'b100;
            ALUA <= rs1_v1;
            ALUB <= rs1_v2;
            FU_ALUControl <= rs1_ALUControl;
            FU_ALU_EN <= 1'b1;
        end
        else if(rs2_data_ready & ~FU_poi[1]) begin
            // ... 类似rs1
        end
        else if(rs3_data_ready & ~FU_poi[0]) begin
            // ... 类似rs1
        end
        else begin
            FU_poi <= 3'b0;
            FU_ALU_EN <= 1'b0;
        end
    end

    else FU_ALU_EN <= 1'b0;
end
```

功能单元调度：
1. 复位时清零
2. 结果被取走或无操作时，检查保留站数据就绪情况
3. 按优先级选择就绪的保留站
4. 设置功能单元使能和操作数

#### 10.1.4 功能单元实例化

```verilog
FU_ALU alu(.clk(clk),.EN(FU_ALU_EN),.finish(FU_ALU_finish),
    .ALUControl(FU_ALUControl),.ALUA(ALUA),.ALUB(ALUB),.res(ALUout),
    .zero(),.overflow());
```

实例化ALU功能单元。

#### 10.1.5 结果处理

```verilog
always@(negedge clk or posedge rst)begin
    if(rst) begin
        FU_ALU_finish_reg <= 1'b0;
        ALUout_reg <= 32'b0;
    end
    else if(FU_ALU_finish & ~FU_result_taken) begin
        FU_ALU_finish_reg <= 1'b1;
        ALUout_reg <= ALUout;
    end
    else if(FU_result_taken) begin
        FU_ALU_finish_reg <= 1'b0;
    end
end

assign cdb_request = FU_ALU_finish | FU_ALU_finish_reg;
assign cdb_out = {`FU_ALU_TAG, FU_poi, FU_ALU_finish_reg ? ALUout_reg : ALUout};
```

结果处理：
1. 复位时清零
2. 计算完成且结果未被取走时，锁存结果
3. 结果被取走时，清除完成标志
4. 生成CDB请求和输出数据

#### 10.1.6 保留站结果处理

```verilog
always@(negedge clk or posedge rst) begin
    if(rst) begin
        rs1_FU_result_taken <= 1'b0;
        rs2_FU_result_taken <= 1'b0;
        rs3_FU_result_taken <= 1'b0;
    end
    else begin
        rs1_FU_result_taken <= cdb[`CDB_RS_FIELD] == 3'b100 && FU_result_taken;
        rs2_FU_result_taken <= cdb[`CDB_RS_FIELD] == 3'b010 && FU_result_taken;
        rs3_FU_result_taken <= cdb[`CDB_RS_FIELD] == 3'b001 && FU_result_taken;
    end
end
```

保留站结果处理：根据CDB上的保留站编号设置结果被取走标志。

### 10.2 乘法和除法功能单元控制 (unit_mul.v, unit_div.v)

乘法和除法功能单元控制与ALU功能单元控制类似，主要区别在于功能单元的实例化和操作。

### 10.3 加载/存储功能单元控制 (unit_load_store.v)

加载/存储功能单元控制管理加载和存储保留站：

```verilog
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
```

加载/存储功能单元控制比其他功能单元控制更复杂，因为它需要处理加载和存储之间的依赖关系。

## 11. 内存访问 (RAM_B.v, ROM_D.v)

### 11.1 数据内存 (RAM_B.v)

数据内存存储程序数据：

```verilog
module RAM_B(
    input [31:0] addra,
    input clka,      // normal clock
    input[31:0] dina,
    input wea, 
    output[31:0] douta,
    input[2:0] mem_u_b_h_w
);
```

#### 11.1.1 内存存储

```verilog
reg[7:0] data[0:127];

initial begin
    $readmemh("ram.hex", data);
end
```

使用字节数组存储内存数据，初始化时从文件加载。

#### 11.1.2 写入逻辑

```verilog
always @ (negedge clka) begin
    if (wea & ~|addra[31:7]) begin
        data[addra[6:0]] <= dina[7:0];
        if(mem_u_b_h_w[0] | mem_u_b_h_w[1])
            data[addra[6:0] + 1] <= dina[15:8];
        if(mem_u_b_h_w[1]) begin
            data[addra[6:0] + 2] <= dina[23:16];
            data[addra[6:0] + 3] <= dina[31:24];
        end
    end
end
```

写入逻辑：根据地址和类型写入1、2或4个字节。

#### 11.1.3 读取逻辑

```verilog
assign douta = addra[31:7] ? 32'b0 :
    mem_u_b_h_w[1] ? {data[addra[6:0] + 3], data[addra[6:0] + 2],
                data[addra[6:0] + 1], data[addra[6:0]]} :
    mem_u_b_h_w[0] ? {mem_u_b_h_w[2] ? 16'b0 : {16{data[addra[6:0] + 1][7]}},
                data[addra[6:0] + 1], data[addra[6:0]]} :
    {mem_u_b_h_w[2] ? 24'b0 : {24{data[addra[6:0]][7]}}, data[addra[6:0]]};
```

读取逻辑：根据地址和类型读取1、2或4个字节，并进行符号扩展或零扩展。

### 11.2 指令内存 (ROM_D.v)

指令内存存储程序指令：

```verilog
module ROM_D(
    input [6:0] a,
    output [31:0] spo
);
```

指令内存实现较为简单，从文件加载指令并根据地址读取。

## 12. 指令生命周期总结

基于以上分析，Tomasulo架构中的指令生命周期可以总结如下：

### 12.1 取指阶段

1. PC寄存器提供当前指令地址
2. 指令内存根据PC读取指令
3. PC更新为下一条指令地址（PC+4或跳转地址）

### 12.2 译码/发射阶段

1. 指令被传递到译码/发射单元
2. 译码单元解析指令类型和操作码
3. 生成控制信号和立即数
4. 读取源寄存器值和标记
5. 检查功能单元是否可用
6. 如果可用，分配保留站并设置目标寄存器标记
7. 如果不可用，停顿流水线

### 12.3 执行阶段

1. 保留站等待操作数就绪
2. 操作数就绪后，功能单元调度器选择就绪的保留站
3. 功能单元执行操作
4. 执行完成后，请求使用公共数据总线

### 12.4 写回阶段

1. 公共数据总线仲裁器选择一个功能单元的结果
2. 结果通过公共数据总线广播
3. 保留站和寄存器文件监听公共数据总线
4. 匹配的保留站更新操作数
5. 匹配的寄存器更新值并清除标记

### 12.5 特殊情况

1. 跳转指令：在译码阶段计算跳转条件和目标地址，如果跳转，更新PC并刷新流水线
2. 加载/存储指令：需要计算地址并访问数据内存
3. 结构冒险：功能单元全部忙碌时停顿流水线
4. 数据冒险：通过标记和保留站自动处理
5. 控制冒险：通过跳转预测和投机执行处理

## 13. 总结

Tomasulo架构是一种经典的乱序执行处理器架构，通过保留站、标记和公共数据总线实现指令级并行。本分析详细解读了一个RISC-V Tomasulo架构实现的每个模块和每一行代码，按照指令生命周期组织，展示了从取指到写回的完整流程。

该实现包括以下关键组件：
1. 顶层控制逻辑
2. 译码和发射单元
3. 带标记的寄存器文件
4. 各类保留站
5. 多种功能单元
6. 公共数据总线
7. 内存访问单元
8. 跳转处理单元

通过这些组件的协同工作，实现了高效的乱序执行，提高了处理器的性能。
