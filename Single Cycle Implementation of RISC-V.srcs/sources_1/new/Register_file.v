`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/27/2024 08:48:53 AM
// Design Name: 
// Module Name: Register_file
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

//module N_bit_register #(parameter N = 32)(
//    input load,
//    input clk,
//    input rst,
//    input [N-1:0] D,
//    output [N-1:0] Q
//    );
    
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

    // Instantiate 32 N-bit registers and put them inside an array of 32 elements
    reg [31:0] registers[31:0];
    
    // Initialize registers to 0 on reset
    integer i;
    
//    always @(posedge clk) begin
//        registers[0] = 0;
//    end
    
  
    
    // Handle write operation on the negative edge of the clock
    always @(negedge clk) begin
     if(rst == 1'b1) begin
               for (i = 0; i < 32; i = i + 1) begin
                   registers[i] <= 0;
               end
           end
       else begin
         if(reg_write && write_reg_indx != 0)
                registers[write_reg_indx] <= write_data; //can't change x0 (TODO)
        end
      end
    
    // Assign read_reg_1_data and read_reg_2_data to the register of index read_reg_1_indx and read_reg_2_indx
    // These are combinational logic, directly mapping the outputs to the register array
    assign read_reg_1_data = (read_reg_1_indx == 0) ? 0 : registers[read_reg_1_indx];
    assign read_reg_2_data = (read_reg_2_indx == 0) ? 0 : registers[read_reg_2_indx];

endmodule

