`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/13/2024 07:53:16 PM
// Design Name: 
// Module Name: Forwarding_unit
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


module Forwarding_unit(
    input [4:0] ID_EX_RegisterRs1,
    input [4:0] ID_EX_RegisterRs2,
    input [4:0] EX_MEM_RegisterRd,
    input [4:0] MEM_WB_RegisterRd,
    
    input EX_MEM_RegWrite,
    input MEM_WB_RegWrite,
    
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB
    );
    
    always @(EX_MEM_RegWrite or EX_MEM_RegisterRd or ID_EX_RegisterRs1 or ID_EX_RegisterRs2 or
             MEM_WB_RegWrite or MEM_WB_RegisterRd) begin
        // Initialize ForwardA and ForwardB to zero
        ForwardA <= 2'b00;
        ForwardB <= 2'b00;
    
        // Forwarding logic for ForwardA
        if (EX_MEM_RegWrite && (EX_MEM_RegisterRd != 0) && (EX_MEM_RegisterRd == ID_EX_RegisterRs1)) begin
            ForwardA = 2'b10;
        end else if (MEM_WB_RegWrite && (MEM_WB_RegisterRd != 0) && 
                     !(EX_MEM_RegWrite && (EX_MEM_RegisterRd != 0) && (EX_MEM_RegisterRd == ID_EX_RegisterRs1)) && 
                     (MEM_WB_RegisterRd == ID_EX_RegisterRs1)) begin
            ForwardA <= 2'b01;
        end
    
        // Forwarding logic for ForwardB
        if (EX_MEM_RegWrite && (EX_MEM_RegisterRd != 0) && (EX_MEM_RegisterRd == ID_EX_RegisterRs2)) begin
            ForwardB <= 2'b10;
        end else if (MEM_WB_RegWrite && (MEM_WB_RegisterRd != 0) && 
                     !(EX_MEM_RegWrite && (EX_MEM_RegisterRd != 0) && (EX_MEM_RegisterRd == ID_EX_RegisterRs2)) && 
                     (MEM_WB_RegisterRd == ID_EX_RegisterRs2)) begin
            ForwardB <= 2'b01;
        end
    end

endmodule
