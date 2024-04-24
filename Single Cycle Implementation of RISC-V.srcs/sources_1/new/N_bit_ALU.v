`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/27/2024 07:14:16 AM
// Design Name: 
// Module Name: N_bit_ALU
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

//module MUX(
//    input a,//input 0
//    input b,//input 1
//    input s,//selection
//    output o //output
//    );
    
//    assign o = (s)? a: b;
//endmodule

//module RippleCarryAdder #(parameter N = 4)( // Change N to your desired bit width
//    input [N-1:0] A,
//    input [N-1:0] B,
//    input cin,
//    output [N-1:0] Sum,
//    output cout
//);

//module n_bit_2_x_1_MUX #(N = 4)(//TODO potential error
//    input [N-1:0] a,
//    input [N-1:0] b,
//    input s,
//    output [N-1:0] o
//    );
    
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
    
    wire [31:0] add, sub, not_B;
    assign not_B = (~B);
    assign {CarryFlag, add} = sel[0]? (A + not_B + 1'b1): (A + B); //when sel[2] = 1 then operation is subtraction

    assign ZeroFlag = (ALU_output == 0);
    assign NegativeFlag = add[31]; //determines whether the ALU ouptu is negative
    assign OverflowFlag = (A[31] ^ (not_B[31]) ^ add[31] ^ CarryFlag);
            

    wire [4:0] shift_amount = B[N-1:0] > N-1 ? N-1 : B[4:0];
              
   // MUX 16 * 1
   always @(*) begin
       case(sel)
           4'b0000: ALU_output = add;
           4'b0001: ALU_output = add;
           4'b0101: ALU_output = A & B;
           4'b0100: ALU_output = A | B;
           4'b0111: ALU_output = A ^ B;
           
           4'b1000: ALU_output = A << shift_amount;//holds SLL or SLLI output
           4'b1001: ALU_output = A >> shift_amount;//holds SRL or SRLI output
           4'b1010: ALU_output = A >>> shift_amount; //TODO not working properly, //holds SRA or SRAI output
           
           4'b1101: ALU_output = {31'b0,(NegativeFlag != OverflowFlag)}; //SLT: to be understood
           4'b1111: ALU_output = {31'b0,(~CarryFlag)};
           
           4'b1011: ALU_output = B;
           default: ALU_output = 0;//
       endcase
   end
     
endmodule
