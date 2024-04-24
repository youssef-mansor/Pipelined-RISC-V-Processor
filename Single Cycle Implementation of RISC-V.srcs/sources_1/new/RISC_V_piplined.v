module RISC_V_piplined(
    input  clk, //connected to a push button
    input rst  //connected to a push button (initialize pc and RF to zeros)
);
    
    //Declaring all wires
    
    //instruction and PC
    wire [31:0] instruction;  
    wire [31:0] PC, PC_plus_4, branch_target, PC_input;
    wire zero_and_branch;
    wire [1:0] PCSrc; //out of branch_control into mux_pc
    
    //IF/ID register
    wire [31:0] IF_ID_PC, IF_ID_Inst;
    
    //Control Unit  
    wire [3:0] ALUOp;
    wire [1:0] branchOp, RegData;
    wire MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
    
    //ID/EX register
    wire [10:0] ID_EX_Ctrl;
    wire [31:0] ID_EX_PC, ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [3:0] ID_EX_Func;
    wire [4:0] ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;
    
    //ALU
    wire [3:0] ALU_sel;
    wire zeroFlag;
    wire [31:0] ALU_2nd_src_MUX_out;
    wire [31:0] ALU_output;
    wire [31:0] NegativeFlag;
    wire [31:0] OverflowFlag;
    wire [31:0] CarryFlag;
    wire [1:0] ForwardA;
    wire [1:0] ForwardB;
    wire [31:0] ALU_first_input;
    wire [31:0] ALU_register_second_input;
        
        
    //Register File
    wire [31:0] read_data_1;
    wire [31:0] read_data_2;
    wire [31:0] write_data;
    
    //hazard unit
    wire stall;
    wire [10:0] ctrl_signals;

    
    //Immediate Generator & shift lef
    wire [31:0] ImmGen_output;
    wire [31:0] shift_left_1_out;
    
 
    //EX/MEM
    wire [5:0] EX_MEM_Ctrl;
    wire [31:0] EX_MEM_BranchAddOut, EX_MEM_ALU_out, EX_MEM_RegR2, EX_MEM_CarryFlag, EX_MEM_NegativeFlag, EX_MEM_OverflowFlag;
    wire [4:0] EX_MEM_Rd;
    wire [2:0] EX_MEM_funct3;
    wire EX_MEM_Zero;

    
    //MEM/WB register
    wire [1:0] MEM_WB_Ctrl;
    wire [31:0] MEM_WB_Mem_out, MEM_WB_ALU_out;
    wire [4:0] MEM_WB_Rd;
           
    //Extra Wires yet to be placed proberly
    wire branch_true;
    wire [31:0] NOP;
    wire [31:0] instructin_or_NOP;
    wire [5:0] EX_MEM_ctrl_signals_or_flush;
    wire [31:0] memory_out;
    wire [7:0] single_mem_addr;
    wire [31:0] MEM_WB_PC_PLUS_4;
    wire [31:0] MEM_WB_PC_PLUS_IMM;
    wire [31:0] IF_ID_PC_PLUS_4;
    wire [31:0] rd_input;
    wire [31:0] ID_EX_PC_PLUS_4;       
    assign branch_true = (PCSrc != 2'b00)? 1:0;
    assign NOP = 32'b0000000_00000_00000_000_00000_0110011; 
          
    //Instantiating Modules
    //Instantiate Program counter
    N_bit_register program_counter(.load(stall),
                                   .clk(clk),
                                   .rst(rst),
                                   .D(PC_input),
                                   .Q(PC));
                                   
    n_bit_2_x_1_MUX #(8) single_mem_mux(
        .a(PC[7:0]),
        .b(EX_MEM_ALU_out[7:0]),
        .s(clk),
        .o(single_mem_addr)
    );                               
                                   
    Single_memory sm(
        .clk(clk),
        .MemRead(EX_MEM_Ctrl[4]),
        .func3(EX_MEM_funct3),
        .MemWrite(EX_MEM_Ctrl[5]),
        .addr(single_mem_addr),
        .data_in(EX_MEM_RegR2),
        .data_out(memory_out)
    );
    
    N_bit_register #(96) IF_ID(
                              .clk(~clk),
                              .rst(rst),
                              .load(stall),//it should be always one
                              .D({PC, PC_plus_4, memory_out}),
                              .Q({IF_ID_PC, IF_ID_PC_PLUS_4,IF_ID_Inst}));
    // Hazard detection unit
    HazardDetectionUnit hazard_detection_unit(
        .IF_ID_Rs1(IF_ID_Inst[19:15]),
        .IF_ID_Rs2(IF_ID_Inst[24:20]),
        .ID_EX_Rd(ID_EX_Rd),
        .ID_EX_MemRead(ID_EX_Ctrl[4]),
        .stall(stall)
    );
    
    n_bit_2_x_1_MUX #(11) ctrl_mux(
        .b({ALUSrc, ALUOp, MemWrite,MemRead,branchOp, MemtoReg, RegWrite}),
        .a(11'b0),
        .s(~stall | branch_true),  //when we want to stall stall = 0, so its negative is 1 and in this mux a is picked when selection is 1
        .o(ctrl_signals)
    );
    
    MUX_4x1 mux_rd( .a(MEM_WB_PC_PLUS_4), //00
                    .b(MEM_WB_PC_PLUS_IMM), //01
                    .c(write_data), //10
                    .d(), //11
                    .sel(RegData), //RegData
                    .out(rd_input));
                                         
    //register file
    Register_file RF(.clk(clk),
                     .rst(rst),
                     .read_reg_1_indx(IF_ID_Inst[19:15]),
                     .read_reg_2_indx(IF_ID_Inst[24:20]),
                     .write_reg_indx(MEM_WB_Rd),
                     .write_data(rd_input),
                     .reg_write(MEM_WB_Ctrl[0]),
                     .read_reg_1_data(read_data_1),
                     .read_reg_2_data(read_data_2));
    //control unit
    control_unit CU ( .inst(IF_ID_Inst),
                      .Branch(branchOp),
                      .MemRead(MemRead),
                      .MemtoReg(MemtoReg),
                      .ALUOp(ALUOp),
                      .MemWrite(MemWrite),
                      .ALUSrc(ALUSrc),
                      .RegWrite(RegWrite),
                      .RegData(RegData));
    //Immediate generator
    ImmGen immediate_generator (.Imm(ImmGen_output), 
                                .IR(IF_ID_Inst));
    N_bit_register #(190) ID_EX ( //11 + 32 * 4 + 1 + 3 + 5  + 10
                                 .clk(clk),
                                 .rst(rst),
                                 .load(1'b1), //TODO make sure it's written each triggering edge
                                 .D({
                                 { //re-ordered to match datapath figure WB, M, EX
//                                     ALUSrc,     //EX 10
//                                     ALUOp,      //EX 9
//                                                 //EX 8
//                                                 //EX 7
//                                                 //EX 6
//                                     MemWrite,   //M  5
//                                     MemRead,    //M  4
//                                     branchOp,   //M  3
//                                                 //M  2
//                                     MemtoReg,   //WB 1
//                                     RegWrite    //WB 0
                                      ctrl_signals
                                     },
                                      IF_ID_PC, //32
                                      IF_ID_PC_PLUS_4, //32
                                      read_data_1, //32
                                      read_data_2, //32
                                      ImmGen_output, //32
                                      
                                      {IF_ID_Inst[30],
                                      IF_ID_Inst[14:12]}, //func3 //4
                                      
                                      IF_ID_Inst[19:15],       //Rs1 //5
                                      IF_ID_Inst[24:20],       //Rs2 //5
                                      IF_ID_Inst[11:7]} //rd         //5
                                      ),
                                    
                                 .Q({ID_EX_Ctrl,      //11
                                     ID_EX_PC,        //32
                                     ID_EX_PC_PLUS_4,
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
     ALU_control_unit alu_control (.ALUOp(ID_EX_Ctrl[9:6]),
                                   .funct3(ID_EX_Func[2:0]),
                                   .bit_30(ID_EX_Func[3]),
                                   .ALU_selection(ALU_sel));
  
   //ALU
     N_bit_ALU #(32) alu_inst (.A(ALU_first_input),
                               .B(ALU_2nd_src_MUX_out),
                               .sel(ALU_sel),
                               .ALU_output(ALU_output),
                               .ZeroFlag(zeroFlag),
                               .NegativeFlag(NegativeFlag),
                               .OverflowFlag(OverflowFlag),
                               .CarryFlag(CarryFlag));
   //MUX for ALU 2nd source
   n_bit_2_x_1_MUX  ALU_2nd_src( .a(ID_EX_Imm), //when ALUSrc
                                .b(ALU_register_second_input),
                                .s(ID_EX_Ctrl[10]),
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
    
     //Adder for immediate
     assign branch_target = ID_EX_Imm + ID_EX_PC;
     
     n_bit_2_x_1_MUX #(6) EX_MEM_ctrl_mux(
        .a(6'b0),
        .b(ID_EX_Ctrl[5:0]),
        .s(branch_true),
        .o(EX_MEM_ctrl_signals_or_flush)
     );
    //EX/MEM
    wire [31:0] EX_MEM_PC_PLUS_4;
    N_bit_register #(239) EX_MEM (.clk(~clk),
                   .rst(rst),
                   .load(1'b1),
                   .D({//5 + 32 + 32 + 1 + 32 + 32 + 32 + 3 + 32 + 5
                       EX_MEM_ctrl_signals_or_flush, //M, WB
                        ID_EX_PC_PLUS_4,
                        branch_target,
                        CarryFlag, //new
                        zeroFlag,
                        ALU_output,
                        NegativeFlag, //new
                        OverflowFlag, //new
                        ID_EX_Func[2:0],
                        ALU_register_second_input,
                        ID_EX_Rd}),
                        
                   .Q({ EX_MEM_Ctrl, 
                        EX_MEM_PC_PLUS_4,
                        EX_MEM_BranchAddOut, 
                        EX_MEM_CarryFlag,
                        EX_MEM_Zero,
                        EX_MEM_ALU_out,
                        EX_MEM_NegativeFlag,
                        EX_MEM_OverflowFlag, 
                        EX_MEM_funct3,
                        EX_MEM_RegR2, 
                        EX_MEM_Rd}));
                        

   //MEM/WB

   N_bit_register #(135) MEM_WB(
        .clk(clk),
        .rst(rst),
        .load(1'b1),
        .D({
            EX_MEM_Ctrl[1:0],
            EX_MEM_PC_PLUS_4,
            EX_MEM_BranchAddOut,
            memory_out,
            EX_MEM_ALU_out,
            EX_MEM_Rd
            }),
        .Q({MEM_WB_Ctrl,MEM_WB_PC_PLUS_IMM,MEM_WB_PC_PLUS_4, MEM_WB_Mem_out, MEM_WB_ALU_out,
            MEM_WB_Rd})
   );   
   
   //MUX for Data Memory output
   n_bit_2_x_1_MUX RF_write_data(.a(MEM_WB_Mem_out),
                                 .b(MEM_WB_ALU_out),
                                 .s(MEM_WB_Ctrl[1]),
                                 .o(write_data));
    
   //Adder for PC
   assign PC_plus_4 = PC + 4; 
   
   //MUX for PC input 
//   n_bit_2_x_1_MUX mux_pc(.a(branch_target),
//                          .b(PC_plus_4),
//                          .s(zeroFlag & Branch),//TODO potential error
//                          .o(PC_input));


    branch_control bc(.branchOp(EX_MEM_Ctrl[3:2]),
                    .funct3(EX_MEM_funct3),
                    .zf(EX_MEM_Zero),
                    .cf(EX_MEM_CarryFlag),
                    .sf(EX_MEM_NegativeFlag),
                    .vf(EX_MEM_OverflowFlag),
                    .PCSrc(PCSrc));
                    
    MUX_4x1 mux_pc(.a(PC_plus_4), //sel = 2'b00
                    .b(EX_MEM_BranchAddOut), //sel = 2'b01
                    .c(EX_MEM_ALU_out), //sel 2'b10 for jalr but TODO
                    .d(32'bx), //sel 2'b11
                    .sel(PCSrc),
                    .out(PC_input));//
                                     
endmodule
