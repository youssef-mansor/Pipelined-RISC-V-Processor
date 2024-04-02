`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/20/2024 09:18:48 AM
// Design Name: 
// Module Name: N_bit_register
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
//module DFlipFlop (input clk, 
//                  input rst, 
//                  input D, 
//                  output reg Q);

//module MUX(
//    input a,//input 0
//    input b,//input 1
//    input s,//selection
//    output o //output
//    );
module N_bit_register #(parameter N = 32)(
    input load, //if 1 then Q = D on triggering clock edge
    input clk,
    input rst,
    input [N-1:0] D,
    output [N-1:0] Q
    );
    


    genvar i;
    generate
        for(i = 0; i < N; i = i + 1) begin
            wire a;
            //assign a = (load)?D[i] :Q[i];
            MUX mx(.a(D[i]), .b(Q[i]), .s(load), .o(a));
            DFlipFlop DFF(.clk(clk), .rst(rst), .D(a), .Q(Q[i]));
        end
    endgenerate
    
endmodule
