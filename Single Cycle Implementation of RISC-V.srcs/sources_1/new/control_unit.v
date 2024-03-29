`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/27/2024 09:19:04 AM
// Design Name: 
// Module Name: control_unit
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
    
   always @(*) begin
        case(Inst_6_2)
            5'b01100: begin //R
               Branch = 2'b00;
               MemRead <= 0;
               MemtoReg <= 0;
               ALUOp = 4'b0000;
               MemWrite <= 0;
               ALUsrc <= 0;
               RegWrite <= 1;
//               if(inst[25]==0)
//                   AluOp = 4'b0000;
//               else
//                   AluOp = 4'b1001;
                      end
            5'b00000:begin //I loading instructions
                  Branch <= 2'b00;
                  MemRead <= 1;
                  MemtoReg <= 1;
                  ALUOp <= 4'b0011;
                  MemWrite <= 0;
                  ALUsrc <= 1;
                 RegWrite <= 1;
                     end//
            5'b00100: begin //I arithmatic and logical instructions with immediates
                Branch <= 2'b00;
                MemRead <= 0;
                MemtoReg <= 0;
                ALUOp <= 4'b0001;
                MemWrite <= 0;
                ALUsrc <= 1;
                RegWrite <= 1;  
                end
            5'b01000: begin //S
                  Branch <= 2'b00;
                  MemRead <= 0;
                  MemtoReg <= 1'bx;//don't care
                  ALUOp <= 4'b0100;
                  MemWrite <= 1;
                  ALUsrc <= 1;
                 RegWrite <= 0;
                     end//
            5'b11000: begin //B
                  Branch <= 2'b01;
                  MemRead <= 0;
                  MemtoReg <= 1'bx;//don't care
                  ALUOp <= 4'b0010;
                  MemWrite <= 0;
                  ALUsrc <= 0;
                  RegWrite <= 0;
                        end//
           
//            5'b00000: begin
//                          Branch <= 0;
//                          MemRead <= 0;
//                          MemtoReg <= 0;//don't care
//                          ALUOp <= 2'b10;
//                          MemWrite <= 0;
//                          ALUsrc <= 1;
//                            RegWrite <= 1;
//                        end//
            default: ;//don't do anything
        endcase
    end
    
endmodule
