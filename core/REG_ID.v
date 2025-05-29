`timescale 1ns / 1ps

module    REG_ID(
	input clk,
	input rst,
	input EN,
	input flush,
	input [31:0] PCOUT,
	input [31:0] IR,

	output reg[31:0] IR_ID,
	output reg[31:0] PCurrent_ID,
	output reg valid
);
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            IR_ID <= 32'h00000013;                            //复位清零s
            PCurrent_ID <= 32'h00000000;                     //复位清零
            valid <= 0;
        end
        else if(flush) begin
            PCurrent_ID <= PCOUT;                       //正常取指,传送下一流水级译码
            IR_ID <= 32'h00000013;              //IR waiting for Control Hazards i清s除指令并暂停
            valid <= 0;
        end
        else if(EN)begin
            IR_ID <= IR;                       //正常取指,传送下一流水级译码
            PCurrent_ID <= PCOUT;             //当前取指PC地址，Branch/Junp指令计算目标地址用(非PC+4)
            valid <= 1;
        end
    end
endmodule