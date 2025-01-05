<a name="readme-top"></a>


<!-- PROJECT SHIELDS -->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">

![Image](images/RISC-V-Processor-pipelined-datapth-with-33-instructions.drawio.png)


<h3 align="center">Pipelined RISC-V Processor</h3>

  <p align="center">
<!--     <a href="https://github.com/youssef-mansor/Pipelined-RISC-V-Processor">View Demo</a> -->
<!--     · -->
    <a href="https://github.com/youssef-mansor/Pipelined-RISC-V-Processor/issues">Report Bug</a>
    ·
    <a href="https://github.com/youssef-mansor/Pipelined-RISC-V-Processor/issues">Request Feature</a>
  </p>
</div>


<!-- ABOUT THE PROJECT -->


## About The Project

The projects  builds up a datapath and constructs a simple version of a processor sufficient to implement most of  RISC-V32I instructions. The project is inspired by the textbook [Computer Organization and Design: The Hardware/software Interface" by David A Patterson and John L. Hennessy](https://www.google.com.eg/books/edition/Computer_Organization_and_Design_MIPS_Ed/o3bhDwAAQBAJ?hl=en&gbpv=0).

> **Note**
> This project is still under development and is not yet ready for deployment, but feel free to fork it and use it for your own purposes!

## Technical Details

### Supported subset of the core RISC-V32I instruction set:
<img src="https://drive.google.com/uc?export=download&id=19kXwGxpmvvPZfc_-EWPYvXpKguXLI3-f" alt="Subset of RISCV32I Instructions" width="500">

#### Notes:
- For our subset, the 7-bit opcode field is enough to decode the instruction except for RFormat instructions (ADD, SUB, AND, OR) which all have the same value (0110011) for
that field. For R-Format instructions, the actual operation is determined by the funct3 and
funct7 fields (only bit 30 from the funct7 field is needed).
- The least significant 2 bits of the 7-bit opcode field are always 11 and as such can be
excluded from any decoding circuit inputs.
- If the datapath is implemented without any ports, we will not be able to verify that it is actually executing instructions, so we need to add a monitoring mechanism allowing us to peek inside the processor and check some of the values on the wire. In order to do so we will define the following 2 output ports:
  1. A 16-bit output port connected to the 16 LEDs on the Nexys A7 board. Based on a 2-bit led selection input port (e.g., ledSel) connected to 2 of the Nexys A7 input switches, these LEDs will be used to view either:
      - The instruction currently being executed. These are 32 bits, so we will split them in half. When ledSel = 00, output Instruction[15:0] and when ledSel = 01, output Instruction [31:16].
      - A concatenation of all the control signals (including ALUOp, ALU selection lines, the zero flag, and the output of the branching AND gate). These are exactly 14 bits (the most significant 2 bits of the 16-bit output will always be zero). This concatenation of signals should be outputted when ledSel = 10;
     
  2. A 13-bit output port connected to one of the two 4-digits seven segment displays (SSD) on the Nexys A7 board. Based on a 4-bit SSD selection input port (e.g., ssdSel) connected to 4 of the Nexys A7 input switches, the SSD will be used to view either:
      - The PC output (when ssdSel = 0000)
      - The PC+4 adder output (when ssdSel = 0001)
      - The branch target adder output (when ssdSel = 0010)
      - The PC input (when ssdSel = 0011)
      - The data read from the register file based on RS1 (when ssdSel = 0100)
      - The data read from the register file based on RS2 (when ssdSel = 0101)
      - The data provided as an input to the register file (when ssdSel = 0110)
      - The immediate generator output (when ssdSel = 0111)
      - The shift left 1 output (when ssdSel = 1000)
      - The output of the ALU 2nd source multiplexer (when ssdSel = 1001)
      - The output of the ALU (when ssdSel = 1010)
      - The memory output (when ssdSel = 1011)
    
Note that all the above values are 32-bit values; however, we will only be displaying the least significant 13-bits of each which is enough for this project’s purposes.


### Components of the Design

#### Non-memory Components
- N_bit_register
  - ```
    module N_bit_register #(parameter N = 32)(
    input load,
    input clk,
    input rst,
    input [N-1:0] D,
    output [N-1:0] Q
    );
    ```
  - **Function**: To be used in Program Counter and Register File

- ImmGen
  - ```
    module ImmGen (output reg [31:0] gen_out,
               input [31:0] inst);
    ```
  - **Function**: Generate the appropriate immediate value for instruction based on the opcode
- N_bit_ALU
  - ```
    module N_bit_ALU #(parameter N = 32)( //refactor later to delete some wires
    input [N-1:0] A,
    input [N-1:0] B,
    input [3:0] sel,
    output reg [N-1:0] ALU_output,
    output ZeroFlag,
    output NegativeFlag,
    output OverflowFlag,
    output CarryFlag
    );
    ```
  - **Function**: Executes arithmatic operations, although selection lines allow up to 16 different arithmetic and logic oepration.
- Register_file
  - ```
    module Register_file(
    input clk,
    input rst,
    input [4:0] read_reg_1_indx,
    input [4:0] read_reg_2_indx,
    input [4:0] write_reg_indx,
    input [31:0] write_data,
    input reg_write, // if 1, write_data into write_reg_indx
    output [31:0] read_reg_1_data,
    output [31:0] read_reg_2_data
    );
    ```
- control_unit
  - ```
    module control_unit(
    input [4:0] Inst_6_2, //represet instruction[6:2]
    output reg [1:0] Branch,
    output reg MemRead,
    output reg MemtoReg,
    output reg [3:0] ALUOp,
    output reg MemWrite,
    output reg ALUsrc,
    output reg RegWrite
    );
    ```
  - **Function**: Create a module representing the control unit. The control unit is a combinational circuit with the following truth table (enough to support to the 7 instructions we are interested in)
  - [//]: <> (<img src="https://drive.google.com/uc?export=download&id=15UziXtJ3vu2N4pHhT0080myo7jOPkpp7" alt="Subset of RISCV32I Instructions" width="500">)
  - truth table to be added soon.
- ALU_control_unit
  - ```
    module ALU_control_unit(
    input [1:0] ALUOp,
    input [2:0] funct3, //instruction[14:12]
    input bit_30,
    output reg [3:0] ALU_selection
    );
    ```
  - ALU control unit using the following truth table
  - [//]: <> (<img src="https://drive.google.com/uc?export=download&id=1cKVgo20PKcfNSKJNBYAQpYn_xvmPHXMC" alt="Subset of RISCV32I Instructions" width="500">)
  - truth table to be added soon.
- n_bit_2_x_1_MUX
  - ```
    module n_bit_2_x_1_MUX #(N = 4)(//TODO potential error
    input [N-1:0] a,
    input [N-1:0] b,
    input s,
    output [N-1:0] o
    );
    ```
- branch_control
  - ```
    module branch_control(
    input [1:0] branchOp, 
    input [2:0] funct3, 
    input zf, 
    input cf, 
    input sf, 
    input vf, 
    output reg [1:0] PCSrc);
    ```
  - **Function**: Determines the source of PC whether (PC + 4, branch address target, ALU_output). Uses PCSrc as selection line for a 4x1MUX.

#### Memory Components
- InstMem
  - ```
    module InstMem (input [5:0] addr, output [31:0] data_out); 
    ```
  - **Function**: a word addressable instruction memory with a maximum capacity of 64 words (256 bytes) with 6 address bits; however, since the PC contains the byte address of the instruction to be executed, we must divide it by 4 (discard the least significant 2 bits) before connecting it to the read address input of the instruction memory to convert it to a word address.
- DataMem
  - ```
    module DataMem(
    input clk, 
    input MemRead, 
    input MemWrite,
    input [5:0] addr, 
    input [31:0] data_in, 
    output reg [31:0] data_out);
    ```
  - **Function**:  For similar reasons to the instruction memory, we will implement a word addressable data memory with a maximum capacity of 64 words (256 bytes) with 6 address bits. Similarly, since the ALU computes the byte address of the data item to be loaded or stored, we must divide it by 4 (discard the least significant 2 bits) before connecting it to the address input of the data memory to convert it to a word address
- BCD
  - ```
    module BCD ( 
    input [12:0] num, 
    output reg [3:0] Thousands,
    output reg [3:0] Hundreds, 
    output reg [3:0] Tens, 
    output reg [3:0] Ones 
    ); 
    ```
  - **Function**: Binary to Decimal conversion.
- Four_Digit_Seven_Segment_Driver_Optimized
  - ```
    module Four_Digit_Seven_Segment_Driver_Optimized(
    input clk,
    input [12:0] num, 
    output reg [3:0] Anode, 
    output reg [6:0] LED_out
    );
    ```
  - **Function**: Driver for the seven segment display in the Nexys A7-100T
- RISCV_SSD_Top
  - ```
    module RISCV_SSD_Top(
    input clk_push_button,
    input rst,
    input ssd_clk,
    input [1:0] ledSel,
    input [3:0] ssdSel,
    output [3:0] Anode,
    output [15:0] LEDs,
    output [6:0] LED_out
    );
    ```
  - **Function**: top-level module instantiating both the RISC-V module and the SSD driver module to connect them together.

## Building the Project
1. Download `Single Cycle Implementation of RISC-V.srcs` Folder.
2. Open Vivado.
3. Create New Project.
4. Add  `Single Cycle Implementation of RISC-V.srcs` Folder to the project through `Add directory` button.

## Tech Stack

The project utilizes the following technologies:

[![Verilog](https://img.shields.io/badge/Language-Verilog-blue.svg)](http://www.verilog.com/)
[![Vivado 2018.2](https://img.shields.io/badge/Tool-Vivado%202018.2-green.svg)](https://www.xilinx.com/products/design-tools/vivado.html)
[![Nexys A7-100T](https://img.shields.io/badge/Board-Nexys%20A7--100T-blue.svg)](https://reference.digilentinc.com/reference/programmable-logic/nexys-a7/start)






<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/youssef-mansor/Pipelined-RISC-V-Processor.svg?style=for-the-badge
[contributors-url]: https://github.com/youssef-mansor/Pipelined-RISC-V-Processor/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/youssef-mansor/Pipelined-RISC-V-Processor.svg?style=for-the-badge
[forks-url]: https://github.com/youssef-mansor/Pipelined-RISC-V-Processor/network/members
[stars-shield]: https://img.shields.io/github/stars/youssef-mansor/Pipelined-RISC-V-Processor.svg?style=for-the-badge
[stars-url]: https://github.com/youssef-mansor/Pipelined-RISC-V-Processor/stargazers
[issues-shield]: https://img.shields.io/github/issues/youssef-mansor/Pipelined-RISC-V-Processor.svg?style=for-the-badge
[issues-url]: https://github.com/youssef-mansor/Pipelined-RISC-V-Processor/issues
[license-shield]: https://img.shields.io/github/license/youssef-mansor/Pipelined-RISC-V-Processor.svg?style=for-the-badge
[license-url]: https://github.com/youssef-mansor/Pipelined-RISC-V-Processor/blob/main/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/youssef-m-86a690174/
[product-screenshot]: images/screenshot.png

