`timescale 1ns / 1ps

module cache_memory #(
    parameter CACHE_SIZE = 4096,
    parameter BLOCK_SIZE = 16,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire clk,
    input  wire rst,

    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire [DATA_WIDTH-1:0] write_data,
    input  wire mem_read,
    input  wire mem_write,
    output reg  [DATA_WIDTH-1:0] read_data,
    output reg  hit
);

    localparam NUM_BLOCKS  = CACHE_SIZE / BLOCK_SIZE;
    localparam INDEX_BITS  = $clog2(NUM_BLOCKS);
    localparam OFFSET_BITS = $clog2(BLOCK_SIZE);
    localparam TAG_BITS    = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;

    wire [TAG_BITS-1:0]   tag     = addr[ADDR_WIDTH-1 : ADDR_WIDTH - TAG_BITS];
    wire [INDEX_BITS-1:0] index   = addr[OFFSET_BITS +: INDEX_BITS];
    wire [OFFSET_BITS-1:0] offset = addr[OFFSET_BITS-1:0];
    wire [1:0] word_offset = offset[3:2]; // 4 bytes per word

    // Cache arrays
    reg [TAG_BITS-1:0] tag_array [0:NUM_BLOCKS-1];
    reg valid_array [0:NUM_BLOCKS-1];
    reg [DATA_WIDTH-1:0] data_array [0:NUM_BLOCKS-1][0:(BLOCK_SIZE/DATA_WIDTH)-1];

    integer i;

    // Reset
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < NUM_BLOCKS; i = i + 1) begin
                valid_array[i] <= 1'b0;
                tag_array[i]   <= {TAG_BITS{1'b0}};
            end
            hit <= 0;
            read_data <= 0;
        end else begin
            // Default outputs
            hit <= 0;

            // Write-through + write-allocate
            if (mem_write) begin
                valid_array[index] <= 1'b1;
                tag_array[index]   <= tag;
                data_array[index][word_offset] <= write_data;
                hit <= valid_array[index] && (tag_array[index] == tag);
            end

            // Read
            else if (mem_read) begin
                if (valid_array[index] && (tag_array[index] == tag)) begin
                    hit <= 1;
                    read_data <= data_array[index][word_offset];
                end else begin
                    hit <= 0;
                    read_data <= 32'hDEADBEEF;
                end
            end
        end
    end
endmodule
