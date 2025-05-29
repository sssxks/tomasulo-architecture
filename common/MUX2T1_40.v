`timescale 1ns / 1ps

module MUX2T1_40(input[39:0]I0,
				 input[39:0]I1,
				 input s,
				 output[39:0]o

    );
    assign o = s ? I1 : I0;
endmodule
