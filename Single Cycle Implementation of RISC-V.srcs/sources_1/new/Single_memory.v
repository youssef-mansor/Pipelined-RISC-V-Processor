`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/23/2024 09:06:09 PM
// Design Name: 
// Module Name: Single_memory
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


module Single_memory(input clk, input MemRead,input [2:0]func3, input MemWrite,input [7:0] addr,input [31:0] data_in, output reg [31:0] data_out);

     reg [7:0] mem [0:510]; //255 + 255 = 510
     localparam [8:0] pointer = 256; // 100000000    //  points to the first row reserved for data
     wire [8:0] data_address;
     assign data_address = addr + pointer;    // 8 bits + 9 bits = 9 bits
     
     // for reading
     always @(*)begin 
         if(clk == 1)  // there is no need to test whtether MemRead == 1 as we will not write instructions and we are always reading instructions to the memory
             data_out = {mem[addr+3],mem[addr+2],mem[addr+1],mem[addr]};  // read instriction
         else begin
            if(MemRead == 1) 
                    begin 
                      case(func3) 
                          3'b000: data_out <= {{24{mem[data_address][7]}},mem[data_address]};//lb
                          3'b001: data_out <= {{16{mem[data_address+1][7]}},mem[data_address+1],mem[data_address]};//lh       
                          3'b010: data_out <= {mem[data_address+3],mem[data_address+2],mem[data_address+1],mem[data_address]};//lw         
                          3'b100: data_out <= {24'd0,mem[data_address]};//lbu
                          3'b101: data_out <= {16'd0,mem[data_address+1],mem[data_address]};//lhu
                          default: data_out <= 0;
                      endcase
                    end
                  else data_out <= 0 ;
         end
     end
     
     // for data writing
     always @(posedge clk)begin 
         if(MemWrite==1)
             case(func3) 
                   3'b000: mem[data_address] = data_in[7:0]; //sb
                   3'b001: 
                       begin 
                       {mem[data_address+1],mem[data_address]} = {data_in[15:8],data_in[7:0]};
                       end
                   3'b010:
                        begin 
                         {mem[data_address+3],mem[data_address+2],mem[data_address+1],mem[data_address]} = {data_in[31:24],data_in[23:15],data_in[15:8],data_in[7:0]};
                        end
             endcase
     end
     
     initial begin
     //instructions block
// Initialize instructions in memory for a RISC-V processor
     // {mem[high], mem[high-1], mem[low+1], mem[low]} = 32-bit Instruction
//    {mem[3],mem[2],mem[1],mem[0]} = 32'b0000000_00000_00000_000_00000_0110011; // add x0, x0, x0          0  
//     {mem[7], mem[6], mem[5], mem[4]} = 32'b00000000000000011111000010110111; // lui x1, 31
//     {mem[11], mem[10], mem[9], mem[8]} = 32'b00000000000000010000000100010111; // auipc x2, 16
//     {mem[15], mem[14], mem[13], mem[12]} = 32'b00000001000000000000000111101111; // jal x3, 16
//     {mem[19], mem[18], mem[17], mem[16]} = 32'b00000000011000101000001010110011; // add x5, x5, x6
//     {mem[23], mem[22], mem[21], mem[20]} = 32'b01000000010000101000001010110011; // sub x5, x5, x4
//     {mem[27], mem[26], mem[25], mem[24]} = 32'b00000000000000011000001101100111; // jalr x6, x3, 0
//     {mem[31], mem[30], mem[29], mem[28]} = 32'b00000010001000001000001000110011; // mul x4, x1, x2
//     {mem[35], mem[34], mem[33], mem[32]} = 32'b00000010000100100100001010110011; // div x5, x4, x1
//     {mem[39], mem[38], mem[37], mem[36]} = 32'b00000010000100100110001100110011; // rem x6, x4, x1
//     {mem[43], mem[42], mem[41], mem[40]} = 32'b00000010000100100110001100110011; // jalr x7, x3, 0


//     //Instructions block
         {mem[3],mem[2],mem[1],mem[0]}    =32'b0000000_00000_00000_000_00000_0110011 ; //add x0, x0, x0          0                   
         {mem[7],mem[6],mem[5],mem[4]}    =32'b000000000000_00000_010_00001_0000011 ; //lw x1, 0(x0)             17                  
         {mem[11],mem[10],mem[9],mem[8]}  =32'b000000000100_00000_010_00010_0000011 ; //lw x2, 4(x0)             9 
         {mem[15],mem[14],mem[13],mem[12]}=32'b000000001000_00000_010_00011_0000011 ; //lw x3, 8(x0)             25
         {mem[19],mem[18],mem[17],mem[16]}=32'b0000000_00010_00001_110_00100_0110011 ; //or x4, x1, x2           25
         {mem[23],mem[22],mem[21],mem[20]}=32'b00000000001100100000010001100011; //beq x4, x3, 8
         {mem[27],mem[26],mem[25],mem[24]}=32'b0000000_00010_00001_000_00011_0110011 ; //add x3, x1, x2          *
         {mem[31],mem[30],mem[29],mem[28]}=32'b0000000_00010_00011_000_00101_0110011 ; //add x5, x3, x2    34 <--*
         {mem[35],mem[34],mem[33],mem[32]}=32'b0000000_00101_00000_010_01100_0100011; //sw x5, 12(x0) 0          *
         {mem[39],mem[38],mem[37],mem[36]}=32'b000000001100_00000_010_00110_0000011 ; //lw x6, 12(x0)  34        *
         {mem[43],mem[42],mem[41],mem[40]}=32'b0000000_00001_00110_111_00111_0110011 ; //and x7, x6, x1 0    
         {mem[47],mem[46],mem[45],mem[44]}=32'b0100000_00010_00001_000_01000_0110011 ; //sub x8, x1, x2  8       8
         {mem[51],mem[50],mem[49],mem[48]}=32'b0000000_00010_00001_000_00000_0110011 ; //add x0, x1, x2  26      26
         {mem[55],mem[54],mem[53],mem[52]}=32'b0000000_00001_00000_000_01001_0110011 ; //add x9, x0, x1 17       17

//    //Data block
        {mem[259],mem[258],mem[257],mem[256]}=32'd17; 
        {mem[263],mem[262],mem[261],mem[260]}=32'd9; 
        {mem[267],mem[266],mem[265],mem[264]}=32'd25;
    end
    
    
    
endmodule
