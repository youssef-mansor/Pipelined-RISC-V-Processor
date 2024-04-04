`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/23/2024 09:17:36 PM
// Design Name: 
// Module Name: RISC_V_tb
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


module RISC_V_tb;

// Inputs
reg clk;
reg rst;
reg [1:0] ledSel;
reg [3:0] ssdSel;

// Outputs
wire [15:0] LEDs;
wire [12:0] ssd;

// Clock generation
always #50 clk = ~clk; // 50MHz Clock

// Instantiate the Unit Under Test (UUT)
RISC_V_piplined uut (
    .clk(clk), 
    .rst(rst), 
    .ledSel(ledSel), 
    .ssdSel(ssdSel), 
    .LEDs(LEDs), 
    .ssd(ssd)
);
//initial begin
//        clk = 1;
//        rst = 1;
//        #50
//        rst = 0;
//end
integer i;
integer j;
initial begin
    for (i = 0; i < 64; i = i + 1) begin
        // Initialize Inputs
        clk = 1;
        rst = 1;
        ledSel = 0;
        ssdSel = 0;
        #100;        
        rst = 0;
        // Set ledSel and ssdSel according to the current iteration
        case (i)
            0: begin ledSel = 2'b00; ssdSel = 4'b0000; end
            1: begin ledSel = 2'b00; ssdSel = 4'b0001; end
            2: begin ledSel = 2'b00; ssdSel = 4'b0010; end
            3: begin ledSel = 2'b00; ssdSel = 4'b0011; end
            4: begin ledSel = 2'b00; ssdSel = 4'b0100; end
            5: begin ledSel = 2'b00; ssdSel = 4'b0101; end
            6: begin ledSel = 2'b00; ssdSel = 4'b0110; end
            7: begin ledSel = 2'b00; ssdSel = 4'b0111; end
            8: begin ledSel = 2'b00; ssdSel = 4'b1000; end
            9: begin ledSel = 2'b00; ssdSel = 4'b1001; end
            10: begin ledSel = 2'b00; ssdSel = 4'b1010; end
            11: begin ledSel = 2'b00; ssdSel = 4'b1011; end
            12: begin ledSel = 2'b00; ssdSel = 4'b1100; end
            13: begin ledSel = 2'b00; ssdSel = 4'b1101; end
            14: begin ledSel = 2'b00; ssdSel = 4'b1110; end
            15: begin ledSel = 2'b00; ssdSel = 4'b1111; end
            16: begin ledSel = 2'b01; ssdSel = 4'b0000; end
            17: begin ledSel = 2'b01; ssdSel = 4'b0001; end
            18: begin ledSel = 2'b01; ssdSel = 4'b0010; end
            19: begin ledSel = 2'b01; ssdSel = 4'b0011; end
            20: begin ledSel = 2'b01; ssdSel = 4'b0100; end
            21: begin ledSel = 2'b01; ssdSel = 4'b0101; end
            22: begin ledSel = 2'b01; ssdSel = 4'b0110; end
            23: begin ledSel = 2'b01; ssdSel = 4'b0111; end
            24: begin ledSel = 2'b01; ssdSel = 4'b1000; end
            25: begin ledSel = 2'b01; ssdSel = 4'b1001; end
            26: begin ledSel = 2'b01; ssdSel = 4'b1010; end
            27: begin ledSel = 2'b01; ssdSel = 4'b1011; end
            28: begin ledSel = 2'b01; ssdSel = 4'b1100; end
            29: begin ledSel = 2'b01; ssdSel = 4'b1101; end
            30: begin ledSel = 2'b01; ssdSel = 4'b1110; end
            31: begin ledSel = 2'b01; ssdSel = 4'b1111; end
            32: begin ledSel = 2'b10; ssdSel = 4'b0000; end
            33: begin ledSel = 2'b10; ssdSel = 4'b0001; end
            34: begin ledSel = 2'b10; ssdSel = 4'b0010; end
            35: begin ledSel = 2'b10; ssdSel = 4'b0011; end
            36: begin ledSel = 2'b10; ssdSel = 4'b0100; end
            37: begin ledSel = 2'b10; ssdSel = 4'b0101; end
            38: begin ledSel = 2'b10; ssdSel = 4'b0110; end
            39: begin ledSel = 2'b10; ssdSel = 4'b0111; end
            40: begin ledSel = 2'b10; ssdSel = 4'b1000; end
            41: begin ledSel = 2'b10; ssdSel = 4'b1001; end
            42: begin ledSel = 2'b10; ssdSel = 4'b1010; end
            43: begin ledSel = 2'b10; ssdSel = 4'b1011; end
            44: begin ledSel = 2'b10; ssdSel = 4'b1100; end
            45: begin ledSel = 2'b10; ssdSel = 4'b1101; end
            46: begin ledSel = 2'b10; ssdSel = 4'b1110; end
            47: begin ledSel = 2'b10; ssdSel = 4'b1111; end
            48: begin ledSel = 2'b11; ssdSel = 4'b0000; end
            49: begin ledSel = 2'b11; ssdSel = 4'b0001; end
            50: begin ledSel = 2'b11; ssdSel = 4'b0010; end
            51: begin ledSel = 2'b11; ssdSel = 4'b0011; end
            52: begin ledSel = 2'b11; ssdSel = 4'b0100; end
            53: begin ledSel = 2'b11; ssdSel = 4'b0101; end
            54: begin ledSel = 2'b11; ssdSel = 4'b0110; end
            55: begin ledSel = 2'b11; ssdSel = 4'b0111; end
            56: begin ledSel = 2'b11; ssdSel = 4'b1000; end
            57: begin ledSel = 2'b11; ssdSel = 4'b1001; end
            58: begin ledSel = 2'b11; ssdSel = 4'b1010; end
            59: begin ledSel = 2'b11; ssdSel = 4'b1011; end
            60: begin ledSel = 2'b11; ssdSel = 4'b1100; end
            61: begin ledSel = 2'b11; ssdSel = 4'b1101; end
            62: begin ledSel = 2'b11; ssdSel = 4'b1110; end
            63: begin ledSel = 2'b11; ssdSel = 4'b1111; end
            default: begin ledSel = 2'b00; ssdSel = 4'b0000; end // Default case
        endcase
        //execute all 12 instructions in instruction memory for the current combination of ledSel and ssdSel
        $display("combination number %d: ledSel: %b, ssdSel: %b", i, ledSel, ssdSel);
        for (j = 0; j < 12; j = j + 1) begin
            $display("Cycle: %d, LEDs: %h, ssd: %h", j, LEDs, ssd);
            #100; // Wait for 100ns before moving to the next instruction
        end
    end
end
endmodule

