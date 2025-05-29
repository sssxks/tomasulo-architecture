`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date:    16:35:20 09/25/2017
// Design Name:
// Module Name:    vga_debug
// Project Name:
// Target Devices:
// Tool versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module VGA_TESTP(
	input clk,
	input clk25,
//	input [9:0] PCol,                                      //改了变量名（建议与VGA文本模式统一，增加VGA外设。但修改问题太多，暂时恢复）
//	input [9:0] PRow,                                      //改了变量�??
	input SWO15,                                           //pipeline 新增 反汇编与指令切换
	input SWO14,                                           //存储数据翻页�??2021 Modify :增加显示两帧ROM数据
	input SWO13,                                           //ROM、RAM切换
	input [39:0] Debug_data,

	output[3:0] Red,
	output[3:0] Green,
	output[3:0] Blue,
	output VSYNC,
	output HSYNC,

//	output reg [11:0]dout,
	output [6:0] Debug_addr);

reg [8*89-1:0] Headline="Zhejiang University Computer Organization Experimental SOC Test Environment (With RISC-V)";
reg [39:0] data_buf [0:3];
reg [7:0] ascii_code;
reg [8*5:0] strdata;
reg [11:0]dout;
wire pixel;
wire [9:0] PRow, PCol;

wire  [9:0] row_addr =  PRow - 10'd35;     // pixel ram row addr
wire  [9:0] col_addr =  PCol- 10'd143;    // pixel ram col addr

wire should_latch_debug_data = (PCol < 10'd143) && (PCol[2:0] == 3'b000) && (row_addr[3:0] == 4'b0000);

wire [4:0] char_index_row = row_addr[8:4] - 3;
wire [6:0] char_index_col = (PCol < 10'd143) ? 0 : (col_addr / 8 + 1);
wire [1:0] char_page = char_index_col / 20;
wire [4:0] char_index_in_page = char_index_col % 20;
//wire [2:0] char_index_in_reg_buf = 7 - (char_index_in_page - 9);               //没有使用   2021 Modify 注释�??

//    assign dout = pixel ? {4'b1111, {4{~Debug_addr[5]}}, {4{~Debug_addr[6]}}} : 12'b1100_0000_0000;         //Debug_addr[5]     12'b111111111111
reg flag;
	always @* begin                                                            // 2021 Modify 区域分色方便观察
		if(pixel)
			if(flag)dout = 12'b0000_0000_1111;
			else
			case(Debug_addr[6:5])
				2'b00:   dout = 12'b1111_1111_1111;
				2'b01:   dout = 12'b0000_1111_1111;
				2'b10:   dout = 12'b0000_0111_1111;
				default: dout = 12'b0000_0111_1111;
			endcase
		else if(SWO15 && ((row_addr[9:4] == 19 && col_addr[9:3] > 21 && col_addr[9:3] < 38)
					|| (row_addr[9:4] == 20 && col_addr[9:3] > 25 && col_addr[9:3] < 42)
					|| (row_addr[9:4] == 21 && col_addr[9:3] > 30 && col_addr[9:3] < 47)
					|| (row_addr[9:4] == 22 && col_addr[9:3] > 35 && col_addr[9:3] < 52)
					|| (row_addr[9:4] == 23 && col_addr[9:3] > 40 && col_addr[9:3] < 58)))
				dout = 12'b0100_0000_0000;
		else dout = 12'b0100_0000_0000;
	end

assign Debug_addr = {char_index_row , PCol[4:3]};
wire[7:0] current_display_reg_addr = {1'b0, char_index_row, char_page};

wire [19*8-1:0] inst_if, inst_id, inst_exe, inst_mem, inst_wb;

	assign inst_if  = "Reserve";
	assign inst_id  = "Reserve";
	assign inst_exe = "Reserve";
	assign inst_mem = "Reserve";
	assign inst_wb  = "Reserve";

always @(posedge clk) begin                                         //2021 Modify
	if (should_latch_debug_data) begin
		data_buf[Debug_addr[1:0]] <= Debug_data;
	end
end
always @(posedge clk) begin
	flag <=0;                                                    //2021 Modify：动态显示定�??
	if ((row_addr < 1 * 16) || (row_addr >= 480 - 1 * 16))
		ascii_code <= " ";
	else if(row_addr[9:4] <= 2)
		ascii_code <= row_addr[9:4] == 1 ? (col_addr[9:3] > 13 && col_addr[9:3] < 68 ) ? Headline[(89 - col_addr[9:3] +13)*8 +:8] : " "
										:  (col_addr[9:3] > 23 && col_addr[9:3] < 58 ) ? Headline[(34 - col_addr[9:3] +23)*8 +:8] : " ";
	else begin
		if(SWO15 && row_addr[9:4] >= 19 && row_addr[9:4] <= 23 && col_addr[9:3] > 21 && col_addr[9:3] < 60) begin
			if(SWO15 && row_addr[9:4] == 19 && col_addr[9:3] > 21 && col_addr[9:3] < 40)begin
				ascii_code <= inst_if[(38 - col_addr[9:3] +2)*8 +:8] ;
				flag <= 1;
			end
			else if(SWO15 && row_addr[9:4] == 20 && col_addr[9:3] > 25 && col_addr[9:3] < 44)begin
				ascii_code <= inst_id[(42 - col_addr[9:3] +2)*8 +:8] ;
				flag <= 1;
			end
			else if(SWO15 && row_addr[9:4] == 21 && col_addr[9:3] > 30 && col_addr[9:3] < 49)begin
				ascii_code <= inst_exe[(47 - col_addr[9:3] +2)*8 +:8] ;
				flag <= 1;
			end
			else if(SWO15 && row_addr[9:4] == 22 && col_addr[9:3] > 35 && col_addr[9:3] < 54)begin
				ascii_code <= inst_mem[(52 - col_addr[9:3] +2)*8 +:8] ;
				flag <= 1;
			end
			else if(SWO15 && row_addr[9:4] == 23 && col_addr[9:3] > 41 && col_addr[9:3] < 60)begin
				ascii_code <= inst_wb[(58 - col_addr[9:3] +2)*8 +:8] ;
				flag <= 1;
			end
			else ascii_code <= " ";
		end
			else if (col_addr[2:0] == 3'b111) begin     //--------------
				if ((char_index_in_page >= 2) && (char_index_in_page <= 6)) begin
					ascii_code <= strdata[(6 - char_index_in_page) * 8 +: 8];
				end
			else if ((char_index_in_page >= 8) && (char_index_in_page <= 17)) begin
					ascii_code <=  num2str(data_buf[char_page][(17 - char_index_in_page) * 4  +: 4]);
				end else ascii_code <= " ";
			end
		else ascii_code <= ascii_code;
	end
end

	wire [8*5:0] MEMADDRSTR = SWO13 ? "RAM" : "COD";                                   //切换RAM/ROM地址显示标志
	always @(posedge clk) begin                                                                 //2021 Modify ,后期还需要调�??
		case (current_display_reg_addr[7:0])
			0:  strdata <= "x00:";
			1:  strdata <= "x01:";
			2:  strdata <= "x02:";
			3:  strdata <= "x03:";
			4:  strdata <= "x04:";
			5:  strdata <= "x05:";
			6:  strdata <= "x06:";
			7:  strdata <= "x07:";

			8:  strdata <= "x08:";
			9:  strdata <= "x09:";
			10: strdata <= "x10:";
			11: strdata <= "x11:";
			12: strdata <= "x12:";
			13: strdata <= "x13:";
			14: strdata <= "x14:";
			15: strdata <= "x15:";

			16: strdata <= "x16:";
			17: strdata <= "x17:";
			18: strdata <= "x18:";
			19: strdata <= "x19:";
			20: strdata <= "x20:";
			21: strdata <= "x21:";
			22: strdata <= "x22:";
			23: strdata <= "x23:";

			24: strdata <= "x24:";
			25: strdata <= "x25:";
			26: strdata <= "x26:";
			27: strdata <= "x27:";
			28: strdata <= "x28:";
			29: strdata <= "x29:";
			30: strdata <= "x30:";
			31: strdata <= "x31:";

			32: strdata <= "PC_IF";
			33: strdata <= "instF";
			34: strdata <= "PC_ID";
			35: strdata <= "instD";
			36: strdata <= "lsAdU";
			37: strdata <= "StrHz";
			38: strdata <= "NmStl";
			39: strdata <= "JpStl";

			40: strdata <= "toJmp";
			41: strdata <= "SrcA";
			42: strdata <= "SrcB";
			43: strdata <= "regWr";
			44: strdata <= "aluIs";
			45: strdata <= "mulIs";
			46: strdata <= "divIs";
			47: strdata <= "loaIs";

			48: strdata <= "StoIs";
			49: strdata <= "uJpIs";
			50: strdata <= "BrnIs";
			51: strdata <= "ImmSl";
			52: strdata <= "ALUop";
			53: strdata <= "JMPop";
			54: strdata <= "Imm";
			55: strdata <= "lsAdr";

			56: strdata <= "rs1";
			57: strdata <= "rs1dt";
			58: strdata <= "rs2";
			59: strdata <= "rs2dt";
			60: strdata <= "ALUA";
			61: strdata <= "ALUB";
			62: strdata <= "rd";
			63: strdata <= "PCJmp";

			64: strdata <= "A-req";
			65: strdata <= "M-req";
			66: strdata <= "D-req";
			67: strdata <= "L-req";
			68: strdata <= "A-cdb";
			69: strdata <= "M-cdb";
			70: strdata <= "D-cdb";
			71: strdata <= "L-cdb";

			72: strdata <= "AIsTg";
			73: strdata <= "MIsTg";
			74: strdata <= "DIsTg";
			75: strdata <= "LIsTg";
			76: strdata <= "w_tag";
			77: strdata <= "CDBon";
			78: strdata <= "*CDB*";			

			default: strdata <= "RESRV";
		endcase
end


FONT8_16 FONT_8X16 (                               //后续修改为标准字�??
	.clk(clk),
	.ascii_code(ascii_code[6:0]),
	.row(row_addr[3:0]),
	.col(col_addr[2:0]),
	.row_of_pixels(pixel)
);

	function [7:0] num2str;
		input [3:0] number;
		begin
			if (number < 10)
				num2str = "0" + number;
			else
				num2str = "A" - 10 + number;
		end
	endfunction


		vga     U12(.clk(clk25),
					.rst(1'b0),
					.Din(dout),
					.PCol(PCol),
					.PRow(PRow),
					.R(Red),
					.G(Green),
					.B(Blue),
					.VS(VSYNC),
					.HS(HSYNC),
					.rdn(),
					.vgaclk());

endmodule
