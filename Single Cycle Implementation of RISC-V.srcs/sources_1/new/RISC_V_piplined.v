`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/01/2024 09:02:50 PM
// Design Name: 
// Module Name: RISC_V_piplined
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


module RISC_V_piplined(
    input  clk, //connected to a push button
    input rst  //connected to a push button (initialize pc and RF to zeros)
//    input [1:0] ledSel,// Select infomration displayed on leds (instructions, control signals)
//    input [3:0] ssdSel,// Select info displayed on SSD (PC, rs1, rs2, imm, etc..) //connected to pin E3
//    output reg [15:0] LEDs,
//    output reg [12:0] ssd
    );
//Declaring all wires
    
    //instruction memory and PC
    wire [31:0] instruction;  
    wire [31:0] PC, PC_plus_4, branch_target, PC_input;
    //wire zero_and_branch;
    
    //IF/ID register
    wire [31:0] IF_ID_PC, IF_ID_Inst; //checked
    

    //Control Unit  
    wire [1:0] ALUOp; //checked
    wire Branch, MemRead, MemtoReg, MemWrite, RegWrite, ALUSrc;
    
    //ID/EX register
    wire [7:0] ID_EX_Ctrl;
    wire [31:0] ID_EX_PC, ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [3:0] ID_EX_Func;
    wire [4:0] ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;
    
    
    //ALU
    wire [3:0] ALU_sel;
    wire zeroFlag;
    wire [31:0] ALU_2nd_src_MUX_out;
    wire [31:0] ALU_output;
    wire [1:0] ForwardA;
    wire [1:0] ForwardB;
    wire [31:0] ALU_first_input;
    wire [31:0] ALU_register_second_input;
    
    //Register File
    wire [31:0] read_data_1;
    wire [31:0] read_data_2;
    wire [31:0] write_data;
    
    //Immediate Generator & shift lef
    wire [31:0] ImmGen_output;
    wire [31:0] shift_left_1_out;
    
    //EX/MEM
    wire [4:0] EX_MEM_Ctrl;
    wire [31:0] EX_MEM_BranchAddOut, EX_MEM_ALU_out, EX_MEM_RegR2;
    wire [4:0] EX_MEM_Rd;
    wire EX_MEM_Zero;
    
    //Data Memory
    wire [31:0] read_data_mem;
    
    
    //MEM/WB register
    wire [1:0] MEM_WB_Ctrl;
    wire [31:0] MEM_WB_Mem_out, MEM_WB_ALU_out;
    wire [4:0] MEM_WB_Rd;
    
    //Instantiating Modules
    //Instantiate Program counter
    N_bit_register #(32) program_counter(.load(1'b1),
                                   .clk(clk),
                                   .rst(rst),
                                   .D(PC_input),
                                   .Q(PC));
   //Not that the adder outlined in the datapth doesn't exist, instead pc MUX chooses between 
   //PC_plus_4 which is a wire assign PC_plus_4 = PC + 4;
   //same for IF/ID register it should take same wire.
   //Adder for PC
   assign PC_plus_4 = PC + 4; 
    //instruction memory
    InstMem IM(.addr(PC[7:2]),// InstMem IM(.addr(PC[5:0]/4), TODO
               .data_out(instruction));
               
     N_bit_register #(64) IF_ID(
                                .clk(clk),
                                .rst(rst),
                                .load(1'b1),//it should be always one
                                .D({PC, instruction}),
                                .Q({IF_ID_PC, IF_ID_Inst}));
    //register file
    Register_file RF(.clk(clk),
                     .rst(rst),
                     .read_reg_1_indx(IF_ID_Inst[19:15]), //rs1 address
                     .read_reg_2_indx(IF_ID_Inst[24:20]), //rs2 address
                     .write_reg_indx(MEM_WB_Rd), //changed from instruction[11:7]
                     .write_data(write_data),
                     .reg_write(MEM_WB_Ctrl[0]), //MEM_WB_Ctrl[0] = RegWrite
                     .read_reg_1_data(read_data_1),
                     .read_reg_2_data(read_data_2));
    //control unit
    control_unit CU ( .Inst_6_2(IF_ID_Inst[6:2]), //TODO all signals below
                      .Branch(Branch),
                      .MemRead(MemRead),
                      .MemtoReg(MemtoReg),
                      .ALUOp(ALUOp),
                      .MemWrite(MemWrite),
                      .ALUsrc(ALUSrc),
                      .RegWrite(RegWrite));
    //Immediate generator
    ImmGen immediate_generator (.gen_out(ImmGen_output), 
                                .inst(IF_ID_Inst));
   
   N_bit_register #(155) ID_EX ( //8 + 32 * 4 + 1 + 3 + 5  + 10
                               .clk(clk),
                               .rst(rst),
                               .load(1'b1), //TODO make sure it's written each triggering edge
                               .D({
                               { //re-ordered to match datapath figure WB, M, EX
                                   ALUSrc,     //EX 7
                                   ALUOp,      //EX 6
                                               //EX 5
                                   MemWrite,   //M  4
                                   MemRead,    //M  3
                                   Branch,     //M  2
                                   MemtoReg,   //WB 1
                                   RegWrite    //WB 0
                                   },
                                    IF_ID_PC, //32
                                    read_data_1, //32
                                    read_data_2, //32
                                    ImmGen_output, //32
                                    
                                    {IF_ID_Inst[30],
                                    IF_ID_Inst[14:12]}, //func3 //4
                                    
                                    IF_ID_Inst[19:15],       //Rs1 //5
                                    IF_ID_Inst[24:20],       //Rs2 //5
                                    IF_ID_Inst[11:7]} //rd         //5
                                    ),
                                  
                               .Q({ID_EX_Ctrl,      //8
                                   ID_EX_PC,        //32
                                   ID_EX_RegR1,     //32
                                   ID_EX_RegR2,     //32
                                   ID_EX_Imm,       //32
                                   ID_EX_Func,      //4
                                   ID_EX_Rs1,       //5
                                   ID_EX_Rs2,       //5
                                   ID_EX_Rd}        //5
                                   )
                               );

   //ALU Control
     ALU_control_unit alu_control (.ALUOp(ID_EX_Ctrl[6:5]),
                                   .funct3(ID_EX_Func[2:0]),//instruction[14:12]
                                   .bit_30(ID_EX_Func[3]), //instruction[30]
                                   .ALU_selection(ALU_sel));
                                   
   //MUX for ALU 2nd source
   n_bit_2_x_1_MUX  ALU_2nd_src( .a(ID_EX_Imm), //when ALUSrc //ImmGen_output
                                .b(ALU_register_second_input), //read_data_2
                                .s(ID_EX_Ctrl[7]),
                                .o(ALU_2nd_src_MUX_out));
   //Forwarding Unit
   Forwarding_unit forwarding_unit(
                                .ID_EX_RegisterRs1(ID_EX_Rs1),
                                .ID_EX_RegisterRs2(ID_EX_Rs2),
                                .EX_MEM_RegisterRd(EX_MEM_Rd),
                                .MEM_WB_RegisterRd(MEM_WB_Rd),
                                .EX_MEM_RegWrite(EX_MEM_Ctrl[0]),
                                .MEM_WB_RegWrite(MEM_WB_Ctrl[0]),
                                .ForwardA(ForwardA),
                                .ForwardB(ForwardB)
   );
                         
   //MUX for ForwardA
   nbit_4to1_mux #(32) MUX_ForwardA(
                                .in0(ID_EX_RegR1), //chosen sel = 00 normal case no forwarding
                                .in1(write_data), //chosen sel = 01
                                .in2(EX_MEM_ALU_out), //chosen sel = 10
                                .in3(), //chosen sel = 11
                                .sel(ForwardA),
                                .out(ALU_first_input)
   );
   
   
   

   //MUX for ForwardB
   nbit_4to1_mux #(32) MUX_ForwardB(
                                .in0(ID_EX_RegR2), //chosen sel = 00 normal case no forwarding
                                .in1(write_data), //chosen sel = 01
                                .in2(EX_MEM_ALU_out), //chosen sel = 10
                                .in3(), //chosen sel = 11
                                .sel(ForwardB),
                                .out(ALU_register_second_input)
   );
   
    //ALU
     N_bit_ALU #(32) alu_inst (.A(ALU_first_input),
                               .B(ALU_2nd_src_MUX_out),
                               .sel(ALU_sel),
                               .ALU_output(ALU_output),
                               .ZeroFlag(zeroFlag));

                                
    //shift left                                    changes their place they were right before pc mux SL and immediate adder output
    n_bit_shift_left SL( .D(ID_EX_Imm),
                         .Q(shift_left_1_out));
    //Adder for immediate
    assign branch_target = shift_left_1_out + ID_EX_PC;
                                
   //EX/MEM
   N_bit_register #(107) EX_MEM (.clk(clk),
                  .rst(rst),
                  .load(1'b1),
                  .D({
                      ID_EX_Ctrl[4:0], //M, WB
                       branch_target,
                       zeroFlag,
                       ALU_output,
                       ID_EX_RegR2,
                       ID_EX_Rd}),
                  .Q({EX_MEM_Ctrl, EX_MEM_BranchAddOut, EX_MEM_Zero,
                       EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_Rd}));
   
   //Data Memory
   DataMem data_memory (
       .clk(clk),
       .MemRead(EX_MEM_Ctrl[3]),
       .MemWrite(EX_MEM_Ctrl[4]),
       .addr(EX_MEM_ALU_out[7:2]),
       .data_in(EX_MEM_RegR2),
       .data_out(read_data_mem)
   );
   
   //MEM/WB
   N_bit_register #(71) MEM_WB(
                                .clk(clk),
                                .rst(rst),
                                .load(1'b1),
                                .D({
                                    EX_MEM_Ctrl[1:0],
                                    read_data_mem,
                                    EX_MEM_ALU_out,
                                    EX_MEM_Rd
                                    }),
                                .Q({MEM_WB_Ctrl,MEM_WB_Mem_out, MEM_WB_ALU_out,
                                    MEM_WB_Rd})
   );
   

   //MUX for Data Memory output
   n_bit_2_x_1_MUX RF_write_data(.a(MEM_WB_Mem_out),
                                 .b(MEM_WB_ALU_out),
                                 .s(MEM_WB_Ctrl[1]),
                                 .o(write_data));

   //MUX for PC input 
   n_bit_2_x_1_MUX mux_pc(.a(EX_MEM_BranchAddOut),
                          .b(PC_plus_4),
                          .s(EX_MEM_Zero & EX_MEM_Ctrl[2]),//TODO potential error //EX_MEM_Ctrl[2] is Branch
                          .o(PC_input));
          
//   //RISC-V input output
//   always @(*) begin
//       if(ledSel == 2'b00)
//           LEDs = instruction [15:0];
//       else if (ledSel == 2'b01)
//           LEDs = instruction [31:16];
//       else if (ledSel == 2'b10)
//           LEDs = {8'b00000000, ALUOp, ALU_sel, zeroFlag, zeroFlag & Branch};
//       else
//           LEDs = 0;
//       end    
   
//   always @(*) begin
//       if(ssdSel == 4'b0000)
//           ssd = PC;
//       else if(ssdSel == 4'b0001)
//           ssd = PC + 1;
//       else if(ssdSel == 4'b0010)
//           ssd = branch_target;
//       else if(ssdSel == 4'b0011)
//           ssd = PC_input;
//       else if(ssdSel == 4'b0100)
//           ssd = read_data_1;
//       else if(ssdSel == 4'b0101)
//           ssd = read_data_2;
//       else if(ssdSel == 4'b0110)
//           ssd = write_data;
//       else if(ssdSel == 4'b0111)
//           ssd = ImmGen_output;
//       else if(ssdSel == 4'b1000)
//           ssd = shift_left_1_out;
//       else if(ssdSel == 4'b1001)
//           ssd = ALU_2nd_src_MUX_out;
//       else if(ssdSel == 4'b1010)
//           ssd = ALU_output;
//       else if(ssdSel == 4'b1011)
//           ssd = read_data_mem;
//   end

                                     
endmodule
