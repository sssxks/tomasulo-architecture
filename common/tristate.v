`timescale 1ns / 1ps

module tristate
	#(parameter SIZE = 32)(
	output[SIZE-1:0] dout,
	input[SIZE-1:0] din,
	input en
);

	genvar i;
	generate
		for (i = 0; i<SIZE; i=i+1) begin
			bufif1(dout[i],din[i],en);
		end
	endgenerate
endmodule