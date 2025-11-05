//============================================================
// Testbench: tb_cache_memory.v
// Description: Testbench for cache_memory module
// Compatible with Cadence Xcelium / Incisive
//============================================================

`timescale 1ns / 1ps

module tb_cache_memory;

    // Signals
    reg clk;
    reg rst;
    reg [31:0] addr;
    reg [31:0] write_data;
    reg mem_read;
    reg mem_write;
    wire [31:0] read_data;
    wire hit;

    // DUT instantiation
    cache_memory uut (
        .clk(clk),
        .rst(rst),
        .addr(addr),
        .write_data(write_data),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .read_data(read_data),
        .hit(hit)
    );

    // Clock generation: 10 ns period → 100 MHz
    initial clk = 0;
    always #5 clk = ~clk;

    // Main stimulus
    initial begin
        $display("================================================");
        $display("         CACHE MEMORY TEST STARTED              ");
        $display("================================================");

        // Initialize inputs
        rst = 1'b1;
        addr = 32'b0;
        write_data = 32'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;

        // Apply reset
        #20;
        rst = 1'b0;

        // 1️⃣ Write to address 0x00000040 (should miss first, then allocate)
        #10;
        addr = 32'h0000_0040;
        write_data = 32'hAABB_CCDD;
        mem_write = 1'b1;
        #10 mem_write = 1'b0;
        #10;
        $display("[%0t ns] WRITE @0x%h  Data=0x%h  Hit=%b",
                 $time, addr, write_data, hit);

        // 2️⃣ Read same address (should hit)
        #10;
        addr = 32'h0000_0040;
        mem_read = 1'b1;
        #10 mem_read = 1'b0;
        #10;
        $display("[%0t ns] READ  @0x%h  Data=0x%h  Hit=%b",
                 $time, addr, read_data, hit);

        // 3️⃣ Read a different address (should miss)
        #10;
        addr = 32'h0000_1040;
        mem_read = 1'b1;
        #10 mem_read = 1'b0;
        #10;
        $display("[%0t ns] READ  @0x%h  Data=0x%h  Hit=%b",
                 $time, addr, read_data, hit);

        $display("================================================");
        $display("         CACHE MEMORY TEST FINISHED             ");
        $display("================================================");

        #50;
        $finish;
    end

    // Continuous monitor for signals
    initial begin
        $monitor("[%0t ns] CLK=%b RST=%b READ=%b WRITE=%b ADDR=0x%h DATA_OUT=0x%h HIT=%b",
                 $time, clk, rst, mem_read, mem_write, addr, read_data, hit);
    end

endmodule
