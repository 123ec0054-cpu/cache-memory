Cache Memory Implementation in Verilog


A highly configurable, synthesizable direct-mapped cache memory implementation in Verilog HDL for ASIC/FPGA designs and computer architecture education.

🎯 Overview
This project provides a parameterized direct-mapped cache module implementing modern cache memory principles. Designed with both educational clarity and practical synthesis in mind, it serves as a foundation for understanding memory hierarchy concepts and can be integrated into larger SoC designs.

Key Highlights:

✅ Fully parameterized and reusable RTL design
✅ Synchronous operation with single-cycle hit latency
✅ Synthesizable for FPGA/ASIC targets
✅ Comprehensive testbench with self-checking capabilities
✅ Clean, well-commented code following industry standards
📚 Table of Contents
Features
Architecture
Getting Started
Module Interface
Configuration Examples
Simulation & Testing
Synthesis Guidelines
Performance Analysis
Design Trade-offs
Roadmap
Contributing
License
✨ Features
Core Capabilities
Direct-Mapped Cache Architecture - One cache line per index for minimal hardware complexity
Write-Through Policy - Ensures data consistency between cache and memory
Write-Allocate Strategy - Allocates cache lines on write misses
Configurable Geometry - Adjustable cache size, block size, and data width
Real-Time Hit Detection - Immediate hit/miss indication for performance monitoring
Technical Specifications
Feature	Specification
Cache Type	Direct-Mapped
Write Policy	Write-Through
Allocation Policy	Write-Allocate
Addressing	Byte-addressable
Clock Domain	Single clock, synchronous
Reset Type	Synchronous, active-high
Hit Latency	1 cycle
Miss Penalty	Configurable (simulated)
🏗 Architecture
Memory Organization
The cache employs a direct-mapped structure where each memory address maps to exactly one cache line based on its index field.

Memory Address Structure (32-bit default):
┌─────────────────────┬──────────────┬─────────────┐
│    Tag (20 bits)    │ Index (8 bits)│ Offset (4 bits)│
└─────────────────────┴──────────────┴─────────────┘
        │                    │              │
        │                    │              └─→ Byte within block
        │                    └────────────────→ Cache line selector
        └─────────────────────────────────────→ Address tag comparison
Cache Line Structure
Each cache line contains:

Valid Bit (1 bit) - Indicates if the line contains valid data
Tag Field (configurable) - Stores the address tag for comparison
Data Block (configurable) - Stores the cached data block
Default Configuration
Parameter	Default Value	Derived Values
CACHE_SIZE	4096 bytes	Total capacity
BLOCK_SIZE	16 bytes	Cache line size
ADDR_WIDTH	32 bits	Address bus width
DATA_WIDTH	32 bits	Data word width
Computed		
NUM_BLOCKS	256 lines	CACHE_SIZE / BLOCK_SIZE
INDEX_BITS	8 bits	log₂(NUM_BLOCKS)
OFFSET_BITS	4 bits	log₂(BLOCK_SIZE)
TAG_BITS	20 bits	ADDR_WIDTH - INDEX_BITS - OFFSET_BITS
Operation Flowchart
┌─────────────┐
│ Memory Req  │
└──────┬──────┘
       │
       ▼
┌─────────────────┐
│ Extract Tag,    │
│ Index, Offset   │
└────────┬────────┘
         │
         ▼
    ┌────────┐      Valid=0
    │ Valid? ├──────────────────┐
    └───┬────┘                  │
        │ Valid=1               │
        ▼                       ▼
    ┌────────┐              ┌──────┐
    │ Tag    │ Tag≠         │ MISS │
    │ Match? ├──────────────►└──────┘
    └───┬────┘              
        │ Tag=
        ▼
    ┌──────┐
    │  HIT │
    └──────┘
🚀 Getting Started
Prerequisites
Required Tools:

Verilog HDL simulator (choose one):
Icarus Verilog (Open-source)
ModelSim (Industry standard)
Xilinx Vivado (FPGA synthesis)
Cadence Xcelium (Enterprise)
Knowledge Requirements:

Verilog HDL fundamentals
Basic computer architecture concepts
Understanding of cache memory principles
Installation
Clone the repository:
bash
git clone https://github.com/yourusername/cache-memory-verilog.git
cd cache-memory-verilog
Verify file structure:
cache-memory-verilog/
├── src/
│   └── cache_memory.v          # Main cache module
├── tb/
│   └── tb_cache_memory.v       # Testbench
├── sim/
│   └── run_sim.sh              # Simulation script
├── docs/
│   └── architecture.md         # Detailed architecture docs
└── README.md
Quick Start Simulation
Option 1: Icarus Verilog (Recommended for beginners)
bash
# Compile
iverilog -g2012 -o cache_sim src/cache_memory.v tb/tb_cache_memory.v

# Run simulation
vvp cache_sim

# View waveforms (optional)
gtkwave dump.vcd
Option 2: Cadence Xcelium
bash
xrun src/cache_memory.v tb/tb_cache_memory.v \
     -access +rwc \
     -gui \
     +define+DUMP_VCD
Option 3: ModelSim
bash
# Compile
vlog -work work src/cache_memory.v tb/tb_cache_memory.v

# Simulate
vsim -c work.tb_cache_memory -do "run -all; quit"

# Interactive GUI mode
vsim work.tb_cache_memory
Option 4: Xilinx Vivado
tcl
# Launch Vivado and run in TCL console
read_verilog src/cache_memory.v
read_verilog -sv tb/tb_cache_memory.v
launch_simulation
run all
🔧 Module Interface
Port Definitions
verilog
module cache_memory #(
    parameter CACHE_SIZE = 4096,
    parameter BLOCK_SIZE = 16,
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
) (
    input  wire                  clk,         // System clock
    input  wire                  rst,         // Synchronous reset (active-high)
    input  wire [ADDR_WIDTH-1:0] addr,        // Memory address
    input  wire [DATA_WIDTH-1:0] write_data,  // Data to write
    input  wire                  mem_read,    // Read enable
    input  wire                  mem_write,   // Write enable
    output reg  [DATA_WIDTH-1:0] read_data,   // Data read from cache
    output reg                   hit          // Cache hit indicator
);
Signal Descriptions
Signal	Type	Width	Description
clk	Input	1	Positive edge-triggered system clock
rst	Input	1	Synchronous reset, clears all valid bits
addr	Input	ADDR_WIDTH	Byte-aligned memory address
write_data	Input	DATA_WIDTH	Data word to write to cache
mem_read	Input	1	Asserted high for read operation
mem_write	Input	1	Asserted high for write operation
read_data	Output	DATA_WIDTH	Data output (valid on cache hit)
hit	Output	1	Asserted when tag matches and line is valid
Timing Diagram
        ┌───┐   ┌───┐   ┌───┐   ┌───┐   ┌───┐
clk     ┘   └───┘   └───┘   └───┘   └───┘   └───
        
addr    ────<A0>────<A1>────<A2>────<A3>────────
        
mem_read ────┌───────┐   ┌───────┐─────────────
             └───────┘   └───────┘
        
hit     ─────────────┌───┐───────────────────────
                     └───┘
        
read_data ───<XX>────<D0>────<XX>────────────────
🎨 Configuration Examples
Example 1: Small L1 Cache (Embedded System)
verilog
cache_memory #(
    .CACHE_SIZE(1024),      // 1 KB cache
    .BLOCK_SIZE(16),        // 16-byte lines
    .ADDR_WIDTH(32),        // 32-bit addressing
    .DATA_WIDTH(32)         // 32-bit data words
) l1_cache (
    .clk(sys_clk),
    .rst(sys_rst),
    .addr(cpu_addr),
    .write_data(cpu_wdata),
    .mem_read(cpu_read),
    .mem_write(cpu_write),
    .read_data(cpu_rdata),
    .hit(cache_hit)
);
Example 2: Larger L2 Cache
verilog
cache_memory #(
    .CACHE_SIZE(65536),     // 64 KB cache
    .BLOCK_SIZE(64),        // 64-byte lines (common for L2)
    .ADDR_WIDTH(32),
    .DATA_WIDTH(64)         // 64-bit data path
) l2_cache (
    .clk(sys_clk),
    .rst(sys_rst),
    .addr(l1_miss_addr),
    .write_data(l1_writeback_data),
    .mem_read(l1_miss_read),
    .mem_write(l1_writeback),
    .read_data(l2_data),
    .hit(l2_hit)
);
Example 3: Custom Configuration for DSP Application
verilog
cache_memory #(
    .CACHE_SIZE(2048),      // 2 KB instruction cache
    .BLOCK_SIZE(32),        // 32-byte lines
    .ADDR_WIDTH(24),        // 16 MB address space
    .DATA_WIDTH(32)
) inst_cache (
    .clk(dsp_clk),
    .rst(dsp_rst),
    .addr(pc_addr),
    .write_data(32'h0),     // Read-only for instructions
    .mem_read(fetch_enable),
    .mem_write(1'b0),
    .read_data(instruction),
    .hit(i_cache_hit)
);
🧪 Simulation & Testing
Testbench Overview
The included testbench (tb_cache_memory.v) validates core functionality through systematic test scenarios:

Reset Test - Verifies initialization of valid bits and registers
Write Miss Test - Confirms cache line allocation on first write
Write Hit Test - Validates tag matching and data update
Read Hit Test - Tests data retrieval from cache
Read Miss Test - Verifies miss detection and handling
Conflict Test - Checks eviction behavior for same-index addresses
Expected Console Output
================================================
         CACHE MEMORY TESTBENCH v1.0            
================================================
[INFO] Configuration:
       - Cache Size: 4096 bytes
       - Block Size: 16 bytes
       - Num Blocks: 256
       - Addr Width: 32 bits
       - Data Width: 32 bits
================================================

[T=50ns]  RESET SEQUENCE COMPLETE
[T=80ns]  WRITE @0x00000040 = 0xAABBCCDD | Hit=0 ✓
[T=110ns] READ  @0x00000040 = 0xAABBCCDD | Hit=1 ✓
[T=140ns] READ  @0x00001040 = 0xDEADBEEF | Hit=0 ✓
[T=170ns] WRITE @0x00001040 = 0x12345678 | Hit=0 ✓
[T=200ns] READ  @0x00001040 = 0x12345678 | Hit=1 ✓
[T=230ns] READ  @0x00000040 = 0xDEADBEEF | Hit=0 ✓ (Evicted)

================================================
         TEST RESULT: ALL TESTS PASSED          
================================================
Running Specific Tests
bash
# Run with VCD dump
iverilog -DDUMP_VCD -o cache_sim src/cache_memory.v tb/tb_cache_memory.v
vvp cache_sim

# Run with specific configuration
iverilog -DCACHE_SIZE=8192 -DBLOCK_SIZE=32 -o cache_sim *.v
vvp cache_sim
Code Coverage (Optional)
bash
# Using Verilator for coverage
verilator --coverage --exe --build tb/tb_cache_memory.cpp src/cache_memory.v
./obj_dir/Vcache_memory
verilator_coverage --annotate logs/coverage logs/coverage.dat
⚙️ Synthesis Guidelines
FPGA Synthesis (Xilinx)
tcl
# Vivado synthesis script
read_verilog src/cache_memory.v
synth_design -top cache_memory -part xc7a35tcpg236-1

# Generate reports
report_utilization -file reports/utilization.rpt
report_timing_summary -file reports/timing.rpt
Expected Resource Usage (Artix-7):

Resource	4KB Cache	8KB Cache
LUTs	~1,200	~2,400
FFs	~850	~1,700
BRAM	2 tiles	4 tiles
Fmax	~200 MHz	~180 MHz
ASIC Synthesis (Design Compiler)
tcl
# Synopsys DC synthesis script
read_verilog src/cache_memory.v
current_design cache_memory

# Set constraints
create_clock -period 5.0 [get_ports clk]
set_input_delay 1.0 -clock clk [all_inputs]
set_output_delay 1.0 -clock clk [all_outputs]

# Compile
compile_ultra

# Reports
report_area
report_timing
Synthesis Optimization Tips
Memory Inference: Ensure synthesis tool infers block RAM/SRAM for data storage
Timing Closure: Register all outputs for better timing
Resource Sharing: Tag comparators can be optimized by synthesis tools
Power Optimization: Consider clock gating for unused cache lines
📊 Performance Analysis
Cache Behavior Metrics
Hit Rate Calculation
Hit Rate = (Cache Hits) / (Total Accesses) × 100%

Example:
- Total Accesses: 1000
- Cache Hits: 850
- Hit Rate: 85%
Average Memory Access Time (AMAT)
AMAT = Hit Time + (Miss Rate × Miss Penalty)

Example:
- Hit Time: 1 cycle
- Miss Rate: 15%
- Miss Penalty: 100 cycles
- AMAT = 1 + (0.15 × 100) = 16 cycles
Access Pattern Impact
Pattern Type	Hit Rate	Notes
Sequential	High (70-90%)	Good spatial locality
Strided (small)	Medium (50-70%)	Depends on stride vs block size
Random	Low (10-30%)	Poor locality
Repeated	Very High (90%+)	Good temporal locality
Conflict Miss Example
Address Mapping (Direct-Mapped):
- 0x00000040 → Index 0x04 (Tag 0x00000)
- 0x00001040 → Index 0x04 (Tag 0x00001)
- 0x00002040 → Index 0x04 (Tag 0x00002)

Accessing these addresses in sequence causes conflict misses!
⚖️ Design Trade-offs
Direct-Mapped vs Set-Associative
Aspect	Direct-Mapped	2-Way Set-Associative	4-Way Set-Associative
Hardware Cost	Lowest	Medium	Higher
Hit Time	Fastest (1 cycle)	+1 cycle (MUX delay)	+1-2 cycles
Miss Rate	Higher	Lower	Lowest
Complexity	Simplest	Moderate	Complex
Power	Lowest	Medium	Higher
Write Policies Comparison
Write-Through (Current Implementation)

✅ Simple consistency with main memory
✅ No writeback logic needed
❌ High memory traffic on writes
❌ Lower write performance
Write-Back (Alternative)

✅ Lower memory bandwidth usage
✅ Better write performance
❌ Requires dirty bit logic
❌ Complex eviction handling
Block Size Impact
Small Blocks (8-16 bytes):
+ Lower miss penalty
+ Less wasted bandwidth
- Higher tag overhead
- Less spatial locality exploitation

Large Blocks (64-128 bytes):
+ Better spatial locality
+ Lower tag overhead
- Higher miss penalty
- More wasted bandwidth on small accesses
🗺 Roadmap
Version 2.0 (Planned)
 Set-Associative Cache - 2-way and 4-way configurations
 LRU Replacement Policy - Proper replacement for associative caches
 Write-Back Policy - Add dirty bit and writeback logic
 Burst Transfer Support - Multi-word access in single transaction
 AXI4 Interface - Industry-standard bus protocol
 Performance Counters - Hardware hit/miss tracking
Version 3.0 (Future)
 Multi-Level Cache - Integrated L1/L2 hierarchy
 Cache Coherency - MESI protocol for multi-core
 Prefetching Logic - Stride-based prefetch unit
 ECC Support - Error detection and correction
 Power Management - Dynamic voltage/frequency scaling
 Formal Verification - SystemVerilog assertions
Community Requests
Have a feature request? Open an issue with the enhancement label!

🤝 Contributing
We welcome contributions! Here's how you can help:

Reporting Issues
Search existing issues to avoid duplicates
Provide detailed description with reproduction steps
Include simulation logs and waveforms if applicable
Submitting Pull Requests
bash
# Fork the repository and clone
git clone https://github.com/your-fork/cache-memory-verilog.git
cd cache-memory-verilog

# Create feature branch
git checkout -b feature/your-feature-name

# Make changes and test
# ... edit files ...
make test

# Commit with clear message
git commit -m "Add feature: description of changes"

# Push and create PR
git push origin feature/your-feature-name
Coding Standards
Follow Verilog IEEE 1800-2012 standard
Use consistent indentation (2 spaces)
Comment complex logic blocks
Include testbench for new features
Ensure synthesis clean (no latches, no warnings)
Review Process
Automated tests must pass
Code review by maintainers
Documentation updates required
Squash commits before merge
📄 License
This project is licensed under the MIT License - see the LICENSE file for details.

MIT License

Copyright (c) 2025 Rishikiran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

[Full license text...]
🙏 Acknowledgments
Textbook References: Computer Organization and Design by Patterson & Hennessy
Industry Standards: ARM AMBA AXI Protocol Specification
Community: Thanks to the open-source hardware community for tools and support
Inspiration: Based on classic cache architectures from MIPS R4000 and ARM Cortex series
📞 Contact & Support
Author: Rishikiran
Email: rishikiranpamu723@gmail.com
GitHub: 123ec0054-cpu
Documentation: Full docs
Issues: Report bugs

📖 Additional Resources
Learning Materials
Computer Architecture Course - MIT OpenCourseWare
Cache Memory Tutorial
Verilog HDL Guide
Related Projects
OpenRISC OR1200 Processor
RISC-V Rocket Chip
PicoRV32
Tools & Simulators
Icarus Verilog
GTKWave
Verilator
Yosys
<div align="center">
⭐ Star this repository if you find it helpful! ⭐



</div>
