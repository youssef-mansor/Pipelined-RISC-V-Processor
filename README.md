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

  <a href="https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation">
    <img src="https://drive.google.com/uc?export=download&id=1wGgrfm3xScgIi3ep1-Dd5cYMml5hcO3K" alt="RISC-V Datapath" width="500">
  </a>

<h3 align="center">RISC-V Datapath single cycle implementation</h3>

  <p align="center">
<!--     <a href="https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation">View Demo</a> -->
<!--     · -->
    <a href="https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation/issues">Report Bug</a>
    ·
    <a href="https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation/issues">Request Feature</a>
  </p>
</div>


<!-- ABOUT THE PROJECT -->


## About The Project

The projects  builds up a datapath and constructs a simple version of a processor sufficient to implement a subset of  RISC-V32I instructions. The project is inspired by the textbook [Computer Organization and Design: The Hardware/software Interface" by David A Patterson and John L. Hennessy](https://www.google.com.eg/books/edition/Computer_Organization_and_Design_MIPS_Ed/o3bhDwAAQBAJ?hl=en&gbpv=0).

> **Note**
> This project is still under development and is not yet ready for deployment, but feel free to fork it and use it for your own purposes!

## Technical Details

### Supported subset of the core RISC-V32I instruction set:
<img src="https://drive.google.com/uc?export=download&id=19ed6VQ4XCn4rO2sM6Pnzr-0p87bh-qN1" alt="Subset of RISCV32I Instructions" width="500">

- The memory-reference instructions:
  - Load doubleword (ld)
  - Store doubleword (sd)
- The arithmetic-logical instructions:
  - Add
  - Subtract
  - Bitwise AND (and)
  - Bitwise OR (or)
- The conditional branch instruction:
  - Branch if equal (beq)
#### Notes:
- For our subset, the 7-bit opcode field is enough to decode the instruction except for RFormat instructions (ADD, SUB, AND, OR) which all have the same value (0110011) for
that field. For R-Format instructions, the actual operation is determined by the funct3 and
funct7 fields (only bit 30 from the funct7 field is needed).
- The least significant 2 bits of the 7-bit opcode field are always 11 and as such can be
excluded from any decoding circuit inputs.

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
  - **Function**: Generate the appropriate immediate value for (BEQ, SW, LW) based on the opcode
- N_bit_ALU
  - ```
    module N_bit_ALU #(parameter N = 32)(
      input [N-1:0] A,
      input [N-1:0] B,
      input [3:0] sel,
      output reg [N-1:0] ALU_output,
      output ZeroFlag
      );
    ```
  - **Function**: Executes arithmatic operations, although selection lines allow up to 16 different arithmetic and logic oeprations I only implemented 4 operations (addition, subtraction, ANDing, and ORing) while the remaining 12 operations will produce zero output regardless the input. The selections line for adding, subtraction, ANDing, and ORing is 0010, 0110, 0000, and 0001 respectively.
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
    output reg Branch,
    output reg MemRead,
    output reg MemtoReg,
    output reg [1:0] ALUOp,
    output reg MemWrite,
    output reg ALUsrc,
    output reg RegWrite
    );
    ```
  - **Function**: Create a module representing the control unit. The control unit is a combinational circuit with the following truth table (enough to support to the 7 instructions we are interested in)
  - <img src="https://drive.google.com/uc?export=download&id=15UziXtJ3vu2N4pHhT0080myo7jOPkpp7" alt="Subset of RISCV32I Instructions" width="500">
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
  - <img src="https://drive.google.com/uc?export=download&id=1cKVgo20PKcfNSKJNBYAQpYn_xvmPHXMC" alt="Subset of RISCV32I Instructions" width="500">
- n_bit_2_x_1_MUX
  - ```
    module n_bit_2_x_1_MUX #(N = 4)(//TODO potential error
    input [N-1:0] a,
    input [N-1:0] b,
    input s,
    output [N-1:0] o
    );
    ```

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

## Building the Project


## Tech Stack

The project utilizes the following technologies:

[![Verilog](https://img.shields.io/badge/Language-Verilog-blue.svg)](http://www.verilog.com/)
[![Vivado 2018.2](https://img.shields.io/badge/Tool-Vivado%202018.2-green.svg)](https://www.xilinx.com/products/design-tools/vivado.html)
[![Nexys A7-100T](https://img.shields.io/badge/Board-Nexys%20A7--100T-blue.svg)](https://reference.digilentinc.com/reference/programmable-logic/nexys-a7/start)






<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/youssef-mansor/RISC-V-Datapath-single-cycle-implementation.svg?style=for-the-badge
[contributors-url]: https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/youssef-mansor/RISC-V-Datapath-single-cycle-implementation.svg?style=for-the-badge
[forks-url]: https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation/network/members
[stars-shield]: https://img.shields.io/github/stars/youssef-mansor/RISC-V-Datapath-single-cycle-implementation.svg?style=for-the-badge
[stars-url]: https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation/stargazers
[issues-shield]: https://img.shields.io/github/issues/youssef-mansor/RISC-V-Datapath-single-cycle-implementation.svg?style=for-the-badge
[issues-url]: https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation/issues
[license-shield]: https://img.shields.io/github/license/youssef-mansor/RISC-V-Datapath-single-cycle-implementation.svg?style=for-the-badge
[license-url]: https://github.com/youssef-mansor/RISC-V-Datapath-single-cycle-implementation/blob/main/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/youssef-m-86a690174/
[product-screenshot]: images/screenshot.png

