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
reg [1:0] ledSel;
reg [3:0] ssdSel;

// Outputs
wire [15:0] LEDs;
wire [12:0] ssd;

// Instantiate the Unit Under Test (UUT)
RISC_V_piplined uut (
    .clk(clk),
    .rst(rst),
    .ledSel(ledSel),
    .ssdSel(ssdSel),
    .LEDs(LEDs),
    .ssd(ssd)
);

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
    
//    // Reset the system
//    rst = 0; #20; rst = 1; #20;
    
//    // Stimulate the ledSel input
//    ledSel = 2'b00; #20;
//    ledSel = 2'b01; #20;
//    ledSel = 2'b10; #20;
//    ledSel = 2'b11; #20;

//    // Reset ledSel and stimulate the ssdSel input
//    ledSel = 2'b00;
//    ssdSel = 4'b0000; #20;
//    ssdSel = 4'b0001; #20;
//    ssdSel = 4'b0010; #20;
//    ssdSel = 4'b0011; #20;
//    ssdSel = 4'b0100; #20;
//    ssdSel = 4'b0101; #20;
//    ssdSel = 4'b0110; #20;
//    ssdSel = 4'b0111; #20;
//    ssdSel = 4'b1000; #20;
//    ssdSel = 4'b1001; #20;
//    ssdSel = 4'b1010; #20;
//    ssdSel = 4'b1011; #20;
//end

always #50 clk = !clk; // Toggle clock every 10ns

endmodule
