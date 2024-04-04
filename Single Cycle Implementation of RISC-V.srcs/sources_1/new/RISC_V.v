`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/09/2024 04:31:40 PM
// Design Name: 
// Module Name: RISC_V
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module RISC_V(
    input  clk, //connected to a push button
    input rst,  //connected to a push button (initialize pc and RF to zeros)
    input [1:0] ledSel,// Select infomration displayed on leds (instructions, control signals)
    input [3:0] ssdSel,// Select info displayed on SSD (PC, rs1, rs2, imm, etc..) //connected to pin E3
    output reg [15:0] LEDs,
    output reg [12:0] ssd
    );
    
    //Declaring all wires
    
    //instruction and PC
    wire [31:0] instruction;  
    wire [31:0] PC, PC_plus_4, branch_target, PC_input;
    wire zero_and_branch;

    //Control Unit  
    wire [1:0] ALUOp;
    wire Branch, MemRead, MemtoReg, MemWrite, AlUSrc, RegWrite;
    
    //ALU
    wire [3:0] ALU_sel;
    wire zeroFlag;
    wire [31:0] ALU_2nd_src_MUX_out;
    wire [31:0] ALU_output;
    
    //Register File
    wire [31:0] read_data_1;
    wire [31:0] read_data_2;
    wire [31:0] write_data;
    
    //Immediate Generator & shift lef
    wire [31:0] ImmGen_output;
    wire [31:0] shift_left_1_out;
    
    //Data Memory
    wire [31:0] read_data_mem;
    
    //Instantiating Modules
    //Instantiate Program counter
    N_bit_register program_counter(.load(1'b1),
                                   .clk(clk),
                                   .rst(rst),
                                   .D(PC_input),
                                   .Q(PC));
    //instruction memory
    InstMem IM(.addr(PC[7:2]),  //instead of PC[5:0]/4
               .data_out(instruction));
    //register file
    Register_file RF(.clk(clk),
                     .rst(rst),
                     .read_reg_1_indx(instruction[19:15]),
                     .read_reg_2_indx(instruction[24:20]),
                     .write_reg_indx(instruction[11:7]),
                     .write_data(write_data),
                     .reg_write(RegWrite),
                     .read_reg_1_data(read_data_1),
                     .read_reg_2_data(read_data_2));
    //control unit
    control_unit CU ( .Inst_6_2(instruction[6:2]),
                      .Branch(Branch),
                      .MemRead(MemRead),
                      .MemtoReg(MemtoReg),
                      .ALUOp(ALUOp),
                      .MemWrite(MemWrite),
                      .ALUsrc(ALUSrc),
                      .RegWrite(RegWrite));
    //Immediate generator
    ImmGen immediate_generator (.gen_out(ImmGen_output), 
                                .inst(instruction));
   //ALU Control
     ALU_control_unit alu_control (.ALUOp(ALUOp),
                                   .funct3(instruction[14:12]),
                                   .bit_30(instruction[30]),
                                   .ALU_selection(ALU_sel));
   //ALU
     N_bit_ALU #(32) alu_inst (.A(read_data_1),
                               .B(ALU_2nd_src_MUX_out),
                               .sel(ALU_sel),
                               .ALU_output(ALU_output),
                               .ZeroFlag(zeroFlag));
   //MUX for ALU 2nd source
   n_bit_2_x_1_MUX  ALU_2nd_src( .a(ImmGen_output), //when ALUSrc
                                .b(read_data_2),
                                .s(ALUSrc),
                                .o(ALU_2nd_src_MUX_out));
   //Data Memory
   DataMem data_memory (
       .clk(clk),
       .MemRead(MemRead),
       .MemWrite(MemWrite),
       .addr(ALU_output[7:2]), //instead of ALU_output[5:0]/4
       .data_in(read_data_2),
       .data_out(read_data_mem)
   );
   //MUX for Data Memory output
   n_bit_2_x_1_MUX RF_write_data(.a(read_data_mem),
                                 .b(ALU_output),
                                 .s(MemtoReg),
                                 .o(write_data));
   //shift left
   n_bit_shift_left SL( .D(ImmGen_output),
                        .Q(shift_left_1_out));
   //Adder for immediate
   assign branch_target = shift_left_1_out + PC;
   //Adder for PC
   assign PC_plus_4 = PC + 4; 
   //MUX for PC input 
   n_bit_2_x_1_MUX mux_pc(.a(branch_target),
                          .b(PC_plus_4),
                          .s(zeroFlag & Branch),//TODO potential error
                          .o(PC_input));
          
   //RISC-V input output
   always @(*) begin
       if(ledSel == 2'b00)
           LEDs = instruction [15:0];
       else if (ledSel == 2'b01)
           LEDs = instruction [31:16];
       else if (ledSel == 2'b10)
           LEDs = {8'b00000000, ALUOp, ALU_sel, zeroFlag, zeroFlag & Branch};
       else
           LEDs = 0;
       end    
   
   always @(*) begin
       if(ssdSel == 4'b0000)
           ssd = PC;
       else if(ssdSel == 4'b0001)
           ssd = PC + 1;
       else if(ssdSel == 4'b0010)
           ssd = branch_target;
       else if(ssdSel == 4'b0011)
           ssd = PC_input;
       else if(ssdSel == 4'b0100)
           ssd = read_data_1;
       else if(ssdSel == 4'b0101)
           ssd = read_data_2;
       else if(ssdSel == 4'b0110)
           ssd = write_data;
       else if(ssdSel == 4'b0111)
           ssd = ImmGen_output;
       else if(ssdSel == 4'b1000)
           ssd = shift_left_1_out;
       else if(ssdSel == 4'b1001)
           ssd = ALU_2nd_src_MUX_out;
       else if(ssdSel == 4'b1010)
           ssd = ALU_output;
       else if(ssdSel == 4'b1011)
           ssd = read_data_mem;
   end

                                     
endmodule
