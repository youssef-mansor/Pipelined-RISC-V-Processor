`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/13/2024 08:24:39 PM
// Design Name: 
// Module Name: nbit_4to1_mux_tb
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


module nbit_4to1_mux_tb;

    parameter n = 32;  // Width of the data inputs and output
    reg [n-1:0] in0, in1, in2, in3;
    reg [1:0] sel;
    wire [n-1:0] out;

    // Instantiate the 4-to-1 Mux
    nbit_4to1_mux #(.n(n)) my_mux (
        .in0(in0),
        .in1(in1),
        .in2(in2),
        .in3(in3),
        .sel(sel),
        .out(out)
    );

    // Initial block for test stimulus
    initial begin
        // Initialize inputs
        in0 = 8'hAA;  // 10101010
        in1 = 8'h55;  // 01010101
        in2 = 8'hFF;  // 11111111
        in3 = 8'h00;  // 00000000

        // Test each selection
        sel = 2'b00; #10;  // Select input 0
        sel = 2'b01; #10;  // Select input 1
        sel = 2'b10; #10;  // Select input 2
        sel = 2'b11; #10;  // Select input 3

        // Finish the simulation
        $finish;
    end

    // Monitor changes
    initial begin
        $monitor("At time %t, sel = %b, out = %h", $time, sel, out);
    end

endmodule
