`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/13/2024 08:18:17 PM
// Design Name: 
// Module Name: nbit_4to1_mux
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

//module n_bit_2_x_1_MUX #(N = 32)(//TODO potential error
//    input [N-1:0] a,
//    input [N-1:0] b,
//    input s,
//    output [N-1:0] o
//    );
    
module nbit_4to1_mux #(N = 32)(
    input [N-1:0] in0, //chosen sel = 0
    input [N-1:0] in1, //chosen sel = 1
    input [N-1:0] in2, //chosen sel = 2
    input [N-1:0] in3, //chosen sel = 3
    input [1:0] sel,          // 2-bit select input
    output [N-1:0] out        // n-bit wide output
);

    wire [N-1:0] mux0_out;
    wire [N-1:0] mux1_out;

    // Instantiate the first level of 2-to-1 muxes
    n_bit_2_x_1_MUX #(.N(N)) mux0(
        .a(in3),
        .b(in2),
        .s(sel[0]),
        .o(mux0_out)
    );

    n_bit_2_x_1_MUX #(.N(N)) mux1(
        .a(in1),
        .b(in0),
        .s(sel[0]),
        .o(mux1_out)
    );

    // Instantiate the second level to select between the outputs of the first level
    n_bit_2_x_1_MUX #(.N(N)) mux2(
        .a(mux0_out),
        .b(mux1_out),
        .s(sel[1]),
        .o(out)
    );
endmodule

