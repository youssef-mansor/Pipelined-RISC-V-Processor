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
`include "defines.v" 

//module control_unit(
//    input [31:0] inst, //represet instruction[6:2]
//    output reg [1:0] Branch,
//    output reg MemRead,
//    output reg MemtoReg,
//    output reg [3:0] ALUOp,
//    output reg MemWrite,
//    output reg ALUsrc,
//    output reg RegWrite,
//    output reg [1:0] RegData
//    );
//    wire [4:0] Inst_6_2;
//    assign Inst_6_2 = inst[6:2];
//   always @(*) begin
//        case(Inst_6_2)
//            5'b01100: begin //R
//               Branch = 2'b00;
//               MemRead <= 0;
//               MemtoReg <= 0;
//               ALUOp = 4'b0000;
//               MemWrite <= 0;
//               ALUsrc <= 0;
//               RegWrite <= 1;
//               if(inst[25]==0)
//                   ALUOp = 4'b0000;
//               else
//                   ALUOp = 4'b1001;
//               end
//            5'b00000:begin //I loading instructions
//                  Branch <= 2'b00;
//                  MemRead <= 1;
//                  MemtoReg <= 1;
//                  ALUOp <= 4'b0011;
//                  MemWrite <= 0;
//                  ALUsrc <= 1;
//                 RegWrite <= 1;
//                     end//
//            5'b00100: begin //I arithmatic and logical instructions with immediates
//                Branch <= 2'b00;
//                MemRead <= 0;
//                MemtoReg <= 0;
//                ALUOp <= 4'b0001;
//                MemWrite <= 0;
//                ALUsrc <= 1;
//                RegWrite <= 1;  
//                end
//            5'b01000: begin //S
//                  Branch <= 2'b00;
//                  MemRead <= 0;
//                  MemtoReg <= 1'bx;//don't care
//                  ALUOp <= 4'b0100;
//                  MemWrite <= 1;
//                  ALUsrc <= 1;
//                 RegWrite <= 0;
//                     end//
//            5'b11000: begin //B
//                  Branch <= 2'b01;
//                  MemRead <= 0;
//                  MemtoReg <= 1'bx;//don't care
//                  ALUOp <= 4'b0010;
//                  MemWrite <= 0;
//                  ALUsrc <= 0;
//                  RegWrite <= 0;
//                        end//
           
////            5'b00000: begin
////                          Branch <= 0;
////                          MemRead <= 0;
////                          MemtoReg <= 0;//don't care
////                          ALUOp <= 2'b10;
////                          MemWrite <= 0;
////                          ALUsrc <= 1;
////                            RegWrite <= 1;
////                        end//
//            default: ;//don't do anything
//        endcase
//    end
    
//endmodule


module control_unit(input [31:0] inst,output reg [3:0] ALUOp,output reg MemRead,
output reg MemWrite,output reg RegWrite, output reg ALUSrc,
output reg MemtoReg, output reg [1:0] RegData, output reg [1:0] Branch);

always @(*) begin 

  case (inst[`IR_opcode]) 
    `OPCODE_Branch:
          begin 
            ALUOp = 4'b0010;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b0;
            ALUSrc = 1'b0;
            MemtoReg = 1'b0; // 
            RegData = 2'b01;
            Branch = 2'b01;
          end
    `OPCODE_Load:
          begin
            ALUOp = 4'b0011;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            ALUSrc = 1'b1;
            MemtoReg = 1'b1; // 
            RegData = 2'b10;
            Branch = 2'b00;
          end
    `OPCODE_Store:
          begin
            ALUOp = 4'b0100;
            MemRead = 1'b0;
            MemWrite = 1'b1;
            RegWrite = 1'b0;
            ALUSrc = 1'b1;
            MemtoReg = 1'b0; // 
            RegData = 2'b01;
            Branch = 2'b00;
          end
    `OPCODE_JALR:
          begin
            ALUOp = 4'b0101;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            ALUSrc = 1'b1;
            MemtoReg = 1'b0; // 
            RegData = 2'b00;
            Branch = 2'b11;
          end
    `OPCODE_JAL:
          begin
            ALUOp = 4'b0110;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            ALUSrc = 1'b1;
            MemtoReg = 1'b0; // 
            RegData = 2'b00;
            Branch = 2'b10;
          end
    `OPCODE_Arith_I:
          begin
            ALUOp = 4'b0001;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            ALUSrc = 1'b1;
            MemtoReg = 1'b0; // 
            RegData = 2'b10;
            Branch = 2'b00;
          end       
    `OPCODE_Arith_R:
          begin
            //ALUOp = 4'b0000;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            ALUSrc = 1'b0;
            MemtoReg = 1'b0; // 
            RegData = 2'b10;
            Branch = 2'b00;
            if(inst[25]==0)
                ALUOp = 4'b0000;
            else
                ALUOp = 4'b1001; 
          end
    `OPCODE_AUIPC:
          begin
            ALUOp = 4'b0111;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            ALUSrc = 1'b0;
            MemtoReg = 1'b0; // 
            RegData = 2'b01;
            Branch = 2'b00;
          end
    `OPCODE_LUI:
          begin
            ALUOp = 4'b1000;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            ALUSrc = 1'b1;
            MemtoReg = 1'b0; // donotCare
            RegData = 2'b10;
            Branch = 2'b00;
            end
  endcase

end 

endmodule
