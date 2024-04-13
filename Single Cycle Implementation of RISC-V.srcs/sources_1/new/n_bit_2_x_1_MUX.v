`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/20/2024 10:08:56 AM
// Design Name: 
// Module Name: n_bit_2_x_1_MUX
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

module n_bit_2_x_1_MUX #(N = 32)(
    input [N-1:0] a,
    input [N-1:0] b,
    input s,
    output [N-1:0] o
    );
    
    genvar i;
    generate
        for(i = 0; i < N; i = i + 1) begin
            MUX mx(.a(a[i]), .b(b[i]), .s(s), .o(o[i]));
        end
    endgenerate
    
endmodule
