`timescale 1ns / 1ps

// Testbench for branch mispredict flush and RAT restore
module branch_restore_tb;
    reg clk, rst;

    // Instantiate core with debug disabled
    RV32core core(
        .debug_en(1'b0),
        .debug_step(1'b0),
        .debug_addr(7'b0),
        .debug_data(),
        .clk(clk),
        .rst(rst)
    );

    // Clock generator
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        // Reset
        rst = 1; #20;
        rst = 0;
        // Run until a mispredict commit occurs
        wait (core.rob0.commit_mispredict == 1);
        #1;
        $display("*** Mispredict Commit Detected ***");
        $display("Commit Index: %0d", core.rob0.commit_index);
        $display("Restored RAT tags: %h", core.rob0.commit_tags_bus);
        $display("Current RAT tags:  %h", core.tregs.all_tags_bus);
        $finish;
    end
endmodule
