`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/02/2024 07:27:20 AM
// Design Name: 
// Module Name: RISC_V_piplined_tb
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


module RISC_V_piplined_tb;

// Inputs
reg clk;
reg rst;
//reg [1:0] ledSel;
//reg [3:0] ssdSel;

//// Outputs
//wire [15:0] LEDs;
//wire [12:0] ssd;

// Instantiate the Unit Under Test (UUT)
RISC_V_piplined uut (
    .clk(clk),
    .rst(rst)
//    .ledSel(ledSel),
//    .ssdSel(ssdSel),
//    .LEDs(LEDs),
//    .ssd(ssd)
);
always #50 clk = !clk;
initial begin
    clk = 0;
    rst = 1;
    #100
    rst = 0;
end
//initial begin
//    // Initialize Inputs
//    clk = 0;
//    rst = 1;
//    ledSel = 0;
//    ssdSel = 0;

//    // Wait 100 ns for global reset to finish
//    #100;
//    rst = 1;
    
//    // Stimulate the ledSel input
//    ledSel = 2'b00; #100;
//    ledSel = 2'b01; #100;
//    ledSel = 2'b10; #100;
//    ledSel = 2'b11; #100;

//    // Reset ledSel and stimulate the ssdSel input
//    ledSel = 2'b00;
//    ssdSel = 4'b0000; #100;
//    ssdSel = 4'b0001; #100;
//    ssdSel = 4'b0010; #100;
//    ssdSel = 4'b0011; #100;
//    ssdSel = 4'b0100; #100;
//    ssdSel = 4'b0101; #100;
//    ssdSel = 4'b0110; #100;
//    ssdSel = 4'b0111; #100;
//    ssdSel = 4'b1000; #100;
//    ssdSel = 4'b1001; #100;
//    ssdSel = 4'b1010; #100;
//    ssdSel = 4'b1011; #100;
//end

endmodule
