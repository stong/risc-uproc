// TODO: default_nettype none
// `default_nettype none

module project3_frame(
  input        CLOCK_50,
  input        RESET_N,
  input  [3:0] KEY,
  input  [9:0] SW,
  output [6:0] HEX0,
  output [6:0] HEX1,
  output [6:0] HEX2,
  output [6:0] HEX3,
  output [6:0] HEX4,
  output [6:0] HEX5,
  output [9:0] LEDR
);

  parameter DBITS    = 32;
  parameter INSTSIZE = 32'd4;
  parameter INSTBITS = 32;
  parameter REGNOBITS = 4;
  parameter REGWORDS = (1 << REGNOBITS);
  parameter IMMBITS  = 16;
  parameter STARTPC  = 32'h100;
  parameter ADDRHEX  = 32'hFFFFF000;
  parameter ADDRLEDR = 32'hFFFFF020;
  parameter ADDRKEY  = 32'hFFFFF080;
  parameter ADDRSW   = 32'hFFFFF090;

  parameter CACHE_INDEX_BITS = 9;
  parameter CACHE_SIZE = 2 << CACHE_INDEX_BITS;
  parameter CACHE_INDEX_LO = 2;
  parameter CACHE_INDEX_HI = CACHE_INDEX_LO+CACHE_INDEX_BITS-1;
  
  // **[PRJ3-Part2] TODO:Change this to "fmedian2.mif" before submitting
  // [NOTICE] please note that both imem and dmem use the SAME "IDMEMINITFILE".
  parameter IDMEMINITFILE = "tests/fmedian2.mif";

 
  
  parameter IMEMADDRBITS = 16;
  parameter IMEMWORDBITS = 2;
  parameter IMEMWORDS    = (1 << (IMEMADDRBITS - IMEMWORDBITS));
  parameter DMEMADDRBITS = 16;
  parameter DMEMWORDBITS = 2;
  parameter DMEMWORDS    = (1 << (DMEMADDRBITS - DMEMWORDBITS));
   
  parameter OP1BITS  = 6;
  parameter OP1_ALUR = 6'b000000;
  parameter OP1_BEQ  = 6'b001000;
  parameter OP1_BLT  = 6'b001001;
  parameter OP1_BLE  = 6'b001010;
  parameter OP1_BNE  = 6'b001011;
  parameter OP1_JAL  = 6'b001100;
  parameter OP1_LW   = 6'b010010;
  parameter OP1_SW   = 6'b011010;
  parameter OP1_ADDI = 6'b100000;
  parameter OP1_ANDI = 6'b100100;
  parameter OP1_ORI  = 6'b100101;
  parameter OP1_XORI = 6'b100110;
  
  // Add parameters for secondary opcode values 
  /* OP2 */
  parameter OP2BITS  = 8;
  parameter OP2_EQ   = 8'b00001000;
  parameter OP2_LT   = 8'b00001001;
  parameter OP2_LE   = 8'b00001010;
  parameter OP2_NE   = 8'b00001011;
  parameter OP2_ADD  = 8'b00100000;
  parameter OP2_AND  = 8'b00100100;
  parameter OP2_OR   = 8'b00100101;
  parameter OP2_XOR  = 8'b00100110;
  parameter OP2_SUB  = 8'b00101000;
  parameter OP2_NAND = 8'b00101100;
  parameter OP2_NOR  = 8'b00101101;
  parameter OP2_NXOR = 8'b00101110;
  parameter OP2_RSHF = 8'b00110000;
  parameter OP2_LSHF = 8'b00110001;
  
  parameter HEXBITS  = 24;
  parameter LEDRBITS = 10;
  parameter KEYBITS = 4;
 


  //*** PLL ***//
  // The reset signal comes from the reset button on the DE0-CV board
  // RESET_N is active-low, so we flip its value ("reset" is active-high)
  // The PLL is wired to produce clk and locked signals for our logic
  wire clk;
  wire locked;
  wire reset;

  Pll myPll(
    .refclk (CLOCK_50),
    .rst        (!RESET_N),
    .outclk_0   (clk),
    .locked     (locked)
  );

  assign reset = !locked;





  //*** -------------------- FETCH STAGE -------------------- ***//
  //*** -------------------- FETCH STAGE -------------------- ***//
  //*** -------------------- FETCH STAGE -------------------- ***//
  //*** -------------------- FETCH STAGE -------------------- ***//
  //*** -------------------- FETCH STAGE -------------------- ***//
  // The PC register and update logic
  
  // branch prediction signals
  reg mispred_EX;
  reg [DBITS-1:0] pctarget_EX; // target PC for branches/jumps

  // LATCHES
  reg [DBITS-1:0] CUR_IP; // THIS HOLDS THE CURRENT INSTRUCTION POINTER
  reg [DBITS-1:0] PC_FE; // THIS HOLDS THE NON-INCREMENTED PC FOR LATER STAGES
  reg [INSTBITS-1:0] inst_FE;
  reg [OP1BITS-1:0] op1_FE;          // op1_FE_w
  reg signed [DBITS-1:0] immval_FE;  // imm_FE_w
  reg [2:0] ctrlsig_FE; // {is_br_FE_w, is_jmp_FE_w, predict_branch_FE_w}
  reg [DBITS-1:0] predict_target_FE; // nextpc_FE_w

  wire [DBITS-1:0] curpc_FE_w;
  wire [DBITS-1:0] nextpc_FE_w;
  wire [DBITS-1:0] inst_FE_w;
  wire stall_pipe; // stall line from ID stage
  
  // PARTIAL DECODE
  wire [OP1BITS-1:0] op1_FE_w;   // OP
  wire [IMMBITS-1:0] imm_FE_w;   // immediate value
  wire [DBITS-1:0] sxt_imm_FE_w; // sext imm
  wire [2:0] ctrlsig_FE_w;
  wire is_br_FE_w;  // 1 if this instruction is a conditional jump
  wire is_jmp_FE_w; // 1 if this instruction is a jump

  // BRANCH AND TARGET PREDICTION
  reg branch_cache_FE [CACHE_SIZE-1:0];
  reg [DBITS-1:0] target_cache_FE [CACHE_SIZE-1:0];
  
  wire [CACHE_INDEX_BITS-1:0] cache_index_FE_w; // pc[CACHE_INDEX_HI:CACHE_INDEX_LO]
  wire predict_branch_FE_w;
  wire [DBITS-1:0] predict_target_FE_w;
  

  // I-MEM
  (* ram_init_file = IDMEMINITFILE, ramstyle = "no_rw_check, M10K" *)
  reg [DBITS-1:0] imem [IMEMWORDS-1:0];

  // **TODO:
  // This statement is used to initialize the I-MEM
  // during simulation using Model-Sim
  // please remove this part before submitting
  //initial begin
  // $readmemh("tests/fmedian2.hex", imem);
  // $readmemh("tests/fmedian2.hex", dmem);
  //end
  
  assign curpc_FE_w = CUR_IP;
  assign inst_FE_w = imem[curpc_FE_w[IMEMADDRBITS-1:IMEMWORDBITS]];
  

  // PARTIAL DECODE
  assign op1_FE_w = inst_FE_w[31:26];
  assign imm_FE_w = inst_FE_w[23:8];

  assign is_br_FE_w = (op1_FE_w == OP1_BEQ) || (op1_FE_w == OP1_BLT) || (op1_FE_w == OP1_BLE) || (op1_FE_w == OP1_BNE);
  assign is_jmp_FE_w = op1_FE_w == OP1_JAL;
  
  SXT mysxt (.IN(imm_FE_w), .OUT(sxt_imm_FE_w));
  
  // if jalr always branches; if Bxx check cache
  assign cache_index_FE_w = curpc_FE_w[CACHE_INDEX_HI:CACHE_INDEX_LO];
  assign predict_branch_FE_w = is_jmp_FE_w ? 1 :
                               is_br_FE_w ? branch_cache_FE[cache_index_FE_w] :
                               0;
  assign predict_target_FE_w = is_jmp_FE_w ? target_cache_FE[cache_index_FE_w] :
                               is_br_FE_w ? (curpc_FE_w + 4 + 4 * sxt_imm_FE_w) :
                               CUR_IP + INSTSIZE;
  
  assign ctrlsig_FE_w = {is_br_FE_w, is_jmp_FE_w, predict_branch_FE_w};
  
  // calculate next pc
  assign nextpc_FE_w = reset ? STARTPC :
                       mispred_EX ? pctarget_EX :
                       stall_pipe ? CUR_IP :
                       predict_branch_FE_w ? predict_target_FE_w :
                       CUR_IP + INSTSIZE;

  // FE_latch
  always @ (posedge clk) begin
    CUR_IP            <= nextpc_FE_w;

    if(reset) begin
      PC_FE             <= STARTPC;
      inst_FE           <= {INSTBITS{1'b0}};
      op1_FE            <= {OP1BITS{1'b0}};
      immval_FE         <= {IMMBITS{1'b0}};
      ctrlsig_FE        <= 2'h0;
      predict_target_FE <= {DBITS{1'b0}};
    end else if (mispred_EX) begin // flush on mispredict
      PC_FE             <= {DBITS{1'b0}};
      inst_FE           <= {INSTBITS{1'b0}};
      op1_FE            <= {OP1BITS{1'b0}};
      immval_FE         <= {IMMBITS{1'b0}};
      ctrlsig_FE        <= 2'h0;
      predict_target_FE <= {DBITS{1'b0}};
    end else if (!stall_pipe) begin
      PC_FE             <= curpc_FE_w;
      inst_FE           <= inst_FE_w;
      op1_FE            <= op1_FE_w;
      immval_FE         <= sxt_imm_FE_w;
      ctrlsig_FE        <= ctrlsig_FE_w;
      predict_target_FE <= nextpc_FE_w;
    end
  end


  //*** -------------------- DECODE STAGE -------------------- ***//
  //*** -------------------- DECODE STAGE -------------------- ***//
  //*** -------------------- DECODE STAGE -------------------- ***//
  //*** -------------------- DECODE STAGE -------------------- ***//
  //*** -------------------- DECODE STAGE -------------------- ***//
  wire [OP1BITS-1:0] op1_ID_w;  // ALUop
  wire [OP2BITS-1:0] op2_ID_w;  // ALUop
  wire [REGNOBITS-1:0] rd_ID_w; // register number of Rd
  wire [REGNOBITS-1:0] rs_ID_w; // register number of Rs
  wire [REGNOBITS-1:0] rt_ID_w; // register number of Rt

  // Two read ports, always using rs and rt for register numbers
  wire [DBITS-1:0] regval1_ID_w;
  wire [DBITS-1:0] regval2_ID_w;
  wire [DBITS-1:0] immval_ID_w; // sext imm
  
  // Control signals
  wire [5:0] ctrlsig_ID_w;
  wire is_br_ID_w;          // 1 if this instruction is a conditional jump
  wire is_jmp_ID_w;         // 1 if this instruction is a jump
  wire predict_branch_ID_w; // 1 if branch predictor predicts 'branch taken'
  wire rd_mem_ID_w;         // 1 if this instruction reads memory
  wire wr_mem_ID_w;         // 1 if this instruction writes to memory
  wire wr_reg_ID_w;         // 1 if this instruction writes to a register

  wire destination_rd_ID_w; // if this instruction writes to a register, 1 if this instruction writes to Rd, 0 if it writes to Rt instead
  
  wire hazard_rs_ID_w; // 1 if there is currently a hazard on Rs with later stages (0 if Rs is R0)
  wire hazard_rt_ID_w; // 1 if there is currently a hazard on Rt with later stages (0 if Rt is R0)
  
  wire [REGNOBITS-1:0] wregno_ID_w; // the register number being written to, 0 if none

  // Register file
  reg [DBITS-1:0] regs [REGWORDS-1:0];

  // ID Latches
  reg [DBITS-1:0] PC_ID;
  reg signed [DBITS-1:0] regval1_ID;    // regval1_ID_w
  reg signed [DBITS-1:0] regval2_ID;    // regval2_ID_w
  reg signed [DBITS-1:0] immval_ID;     // immval_ID_w
  reg [OP1BITS-1:0] op1_ID;             // op1_ID_w
  reg [OP2BITS-1:0] op2_ID;             // op2_ID_w
  reg [5:0] ctrlsig_ID;                 // ctrlsig_ID_w
  reg [REGNOBITS-1:0] wregno_ID;        // wregno_ID_w
  reg [INSTBITS-1:0] inst_ID;           // inst_ID_w
  reg [INSTBITS-1:0] predict_target_ID; // predict_target_FE
  
  // Declared here for stall check
  wire is_br_EX_w;
  wire is_jmp_EX_w;
  wire predict_branch_EX_w;
  wire wr_reg_EX_w;  // whether MEM is writing to a register (for stall check)
  wire wr_reg_MEM_w; // whether EX is writing to a register (for stall check)
  wire [REGNOBITS-1:0] wregno_EX_w;  // the register EX *CURRENTLY* wants to write to
  wire [REGNOBITS-1:0] wregno_MEM_w; // the register MEM *CURRENTLY* wants to write to

  // Wires to latches
  assign op1_ID_w = op1_FE;
  assign op2_ID_w = inst_FE[25:18];
  assign rd_ID_w = inst_FE[11:8];
  assign rs_ID_w = inst_FE[7:4];
  assign rt_ID_w = inst_FE[3:0];

  // Read register values
  assign regval1_ID_w = regs[rs_ID_w];
  assign regval2_ID_w = regs[rt_ID_w];
  assign immval_ID_w = immval_FE;

  // Control signals  
  assign is_br_ID_w = ctrlsig_FE[2];
  assign is_jmp_ID_w = ctrlsig_FE[1];
  assign predict_branch_ID_w = ctrlsig_FE[0];
  assign rd_mem_ID_w = op1_ID_w == OP1_LW;
  assign wr_mem_ID_w = op1_ID_w == OP1_SW;
  assign wr_reg_ID_w = !(is_br_ID_w || wr_mem_ID_w); // changed from !(is_br_ID_w || is_jmp_ID_w || wr_mem_ID_w); because JAL writes to Rt
  assign ctrlsig_ID_w = {is_br_ID_w, is_jmp_ID_w, predict_branch_ID_w, rd_mem_ID_w, wr_mem_ID_w, wr_reg_ID_w};

  // Calculate destination regno
  assign destination_rd_ID_w = (op1_ID_w == 0);
  assign wregno_ID_w = wr_reg_ID_w ? (destination_rd_ID_w ? rd_ID_w : rt_ID_w) : 4'b0;


  // Specify stall condition
  // stall on all jumps/branches, and data hazard against EX or MEM
  assign hazard_rs_ID_w = (rs_ID_w != 4'b0) && ((wr_reg_EX_w && (wregno_EX_w == rs_ID_w)) || (wr_reg_MEM_w && (wregno_MEM_w == rs_ID_w)));
  assign hazard_rt_ID_w = (rt_ID_w != 4'b0) && ((wr_reg_EX_w && (wregno_EX_w == rt_ID_w)) || (wr_reg_MEM_w && (wregno_MEM_w == rt_ID_w)));

  assign stall_pipe = hazard_rs_ID_w || hazard_rt_ID_w;


  // ID_latch
  always @ (posedge clk) begin
    if(reset) begin
      PC_ID             <= {DBITS{1'b0}};
      inst_ID           <= {INSTBITS{1'b0}};
      op1_ID            <= {OP1BITS{1'b0}};
      op2_ID            <= {OP2BITS{1'b0}};
      immval_ID         <= {IMMBITS{1'b0}};
      regval1_ID        <= {DBITS{1'b0}};
      regval2_ID        <= {DBITS{1'b0}};
      regval2_ID        <= {DBITS{1'b0}};
      wregno_ID         <= {REGNOBITS{1'b0}};
      predict_target_ID <= {DBITS{1'b0}};
      ctrlsig_ID        <= 5'h0;
    end else if(stall_pipe) begin // emit a NOP for the next stage if stalled
      PC_ID             <= {DBITS{1'b0}};
      inst_ID           <= {INSTBITS{1'b0}};
      op1_ID            <= {OP1BITS{1'b0}};
      op2_ID            <= {OP2BITS{1'b0}};
      immval_ID         <= {IMMBITS{1'b0}};
      regval1_ID        <= {DBITS{1'b0}};
      regval2_ID        <= {DBITS{1'b0}};
      wregno_ID         <= {REGNOBITS{1'b0}};
      predict_target_ID <= {DBITS{1'b0}};
      ctrlsig_ID        <= 5'h0; 
    end else if(mispred_EX) begin // flush if mispredict.
      PC_ID             <= {DBITS{1'b0}};
      inst_ID           <= {INSTBITS{1'b0}};
      op1_ID            <= {OP1BITS{1'b0}};
      op2_ID            <= {OP2BITS{1'b0}};
      immval_ID         <= {IMMBITS{1'b0}};
      regval1_ID        <= {DBITS{1'b0}};
      regval2_ID        <= {DBITS{1'b0}};
      wregno_ID         <= {REGNOBITS{1'b0}};
      predict_target_ID <= {DBITS{1'b0}};
      ctrlsig_ID        <= 5'h0; 
    end else begin
      PC_ID             <= PC_FE;
      op1_ID            <= op1_ID_w;
      op2_ID            <= op2_ID_w;
      immval_ID         <= immval_ID_w;
      regval1_ID        <= regval1_ID_w;
      regval2_ID        <= regval2_ID_w;
      wregno_ID         <= wregno_ID_w;
      ctrlsig_ID        <= ctrlsig_ID_w;
      predict_target_ID <= predict_target_FE;
      inst_ID           <= inst_FE;
    end
  end






  //*** -------------------- AGEN/EXEC STAGE -------------------- ***//
  //*** -------------------- AGEN/EXEC STAGE -------------------- ***//
  //*** -------------------- AGEN/EXEC STAGE -------------------- ***//
  //*** -------------------- AGEN/EXEC STAGE -------------------- ***//
  //*** -------------------- AGEN/EXEC STAGE -------------------- ***//
  // Control signals
  wire rd_mem_EX_w;
  wire wr_mem_EX_w;
  // wr_reg_EX_w is already declared in ID stage for stall detection purposes
  wire [2:0] ctrlsig_EX_w;
 
  wire mispred_EX_w;
  wire [DBITS-1:0] pctarget_EX_w;
  wire should_branch_EX_w;
  wire [CACHE_INDEX_BITS-1:0] cache_index_EX_w;

  reg [REGNOBITS-1:0] wregno_EX;
  reg [INSTBITS-1:0] inst_EX; /* This is for debugging */
  reg [DBITS-1:0] PC_EX; /* This is for debugging. */
  reg [2:0] ctrlsig_EX;

  // Note that aluout_EX_r is declared as reg, but it is output signal from combi logic
  reg signed [DBITS-1:0] aluout_EX_r;
  reg br_cond_EX_r;  
  reg [DBITS-1:0] aluout_EX;
  reg [DBITS-1:0] regval2_EX;
  
  always @* begin
    case (op1_ID)
      OP1_BEQ : br_cond_EX_r = (regval1_ID == regval2_ID);
      OP1_BLT : br_cond_EX_r = (regval1_ID < regval2_ID);
      OP1_BLE : br_cond_EX_r = (regval1_ID <= regval2_ID);
      OP1_BNE : br_cond_EX_r = (regval1_ID != regval2_ID);
      default : br_cond_EX_r = 1'b0;
    endcase
  end

  always @* begin
    if(op1_ID == OP1_ALUR)
      case (op2_ID)
        OP2_EQ   : aluout_EX_r = {31'b0, regval1_ID == regval2_ID}; // TODO clean this up
        OP2_LT   : aluout_EX_r = {31'b0, regval1_ID <  regval2_ID};
        OP2_LE   : aluout_EX_r = {31'b0, regval1_ID <= regval2_ID};
        OP2_NE   : aluout_EX_r = {31'b0, regval1_ID != regval2_ID};
        
        OP2_ADD  : aluout_EX_r = {31'b0, regval1_ID + regval2_ID};
        OP2_AND  : aluout_EX_r = {31'b0, regval1_ID & regval2_ID};
        OP2_OR   : aluout_EX_r = {31'b0, regval1_ID | regval2_ID};
        OP2_XOR  : aluout_EX_r = {31'b0, regval1_ID ^ regval2_ID};
        OP2_SUB  : aluout_EX_r = {31'b0, regval1_ID - regval2_ID};
        OP2_NAND : aluout_EX_r = {31'b0, ~(regval1_ID & regval2_ID)};
        OP2_NOR  : aluout_EX_r = {31'b0, ~(regval1_ID | regval2_ID)};
        OP2_NXOR : aluout_EX_r = {31'b0, ~(regval1_ID ^ regval2_ID)};
        OP2_RSHF : aluout_EX_r = {31'b0, regval1_ID >>> regval2_ID};
        OP2_LSHF : aluout_EX_r = {31'b0, regval1_ID <<< regval2_ID};
        
        default  : aluout_EX_r = {DBITS{1'b0}};
      endcase
    else if(op1_ID == OP1_LW || op1_ID == OP1_SW || op1_ID == OP1_ADDI)
      aluout_EX_r = regval1_ID + immval_ID;
    else if(op1_ID == OP1_ANDI)
      aluout_EX_r = regval1_ID & immval_ID;
    else if(op1_ID == OP1_ORI)
      aluout_EX_r = regval1_ID | immval_ID;
    else if(op1_ID == OP1_XORI)
      aluout_EX_r = regval1_ID ^ immval_ID;
    else if(op1_ID == OP1_JAL)
      aluout_EX_r = PC_ID + 4;
    else
      aluout_EX_r = {DBITS{1'b0}};
  end
  
  assign wregno_EX_w = wregno_ID;
  
  assign is_br_EX_w = ctrlsig_ID[5];
  assign is_jmp_EX_w = ctrlsig_ID[4];
  assign predict_branch_EX_w = ctrlsig_ID[3];
  assign rd_mem_EX_w = ctrlsig_ID[2];
  assign wr_mem_EX_w = ctrlsig_ID[1];
  assign wr_reg_EX_w = ctrlsig_ID[0];
  assign ctrlsig_EX_w = {rd_mem_EX_w, wr_mem_EX_w, wr_reg_EX_w};

  // update branch-related signals here
  assign pctarget_EX_w = is_jmp_EX_w ? (regval1_ID + 4*immval_ID) :
                         (is_br_EX_w && br_cond_EX_r) ? (PC_ID + 4 + 4*immval_ID) :
                         PC_ID + INSTSIZE;
  assign should_branch_EX_w = is_jmp_EX_w || (is_br_EX_w && br_cond_EX_r);
  assign mispred_EX_w = (is_jmp_EX_w || is_br_EX_w) && (pctarget_EX_w != predict_target_ID);
  
  assign cache_index_EX_w = PC_ID[CACHE_INDEX_HI:CACHE_INDEX_LO];

  // EX_latch
  always @ (posedge clk) begin
    if(reset) begin
      PC_EX       <= {DBITS{1'b0}};
      inst_EX     <= {INSTBITS{1'b0}};
      aluout_EX   <= {DBITS{1'b0}};
      wregno_EX   <= {REGNOBITS{1'b0}};
      ctrlsig_EX  <= 3'h0;
      mispred_EX  <= 1'b0;
      pctarget_EX <= {DBITS{1'b0}};
      regval2_EX  <= {DBITS{1'b0}};
    end else if (mispred_EX) begin // flush on previous cycle mispredict
      PC_EX       <= {DBITS{1'b0}};
      inst_EX     <= {INSTBITS{1'b0}};
      aluout_EX   <= {DBITS{1'b0}};
      wregno_EX   <= {REGNOBITS{1'b0}};
      ctrlsig_EX  <= 3'h0;
      mispred_EX  <= 1'b0;
      pctarget_EX <= {DBITS{1'b0}};
      regval2_EX  <= {DBITS{1'b0}};
    end else begin
      PC_EX       <= PC_ID;
      inst_EX     <= inst_ID;
      aluout_EX   <= aluout_EX_r;
      wregno_EX   <= wregno_EX_w;
      ctrlsig_EX  <= ctrlsig_EX_w;
      mispred_EX  <= mispred_EX_w;
      pctarget_EX <= pctarget_EX_w;
      regval2_EX  <= regval2_ID;

      // update branch predictors
      branch_cache_FE[cache_index_EX_w] <= should_branch_EX_w;
      if (is_jmp_EX_w) begin
        target_cache_FE[cache_index_EX_w] <= pctarget_EX_w;
      end
    end
  end
  





  //*** -------------------- MEM STAGE -------------------- ***//
  //*** -------------------- MEM STAGE -------------------- ***//
  //*** -------------------- MEM STAGE -------------------- ***//
  //*** -------------------- MEM STAGE -------------------- ***//
  //*** -------------------- MEM STAGE -------------------- ***//

  wire rd_mem_MEM_w;
  wire wr_mem_MEM_w;
  
  wire [DBITS-1:0] memaddr_MEM_w;
  wire [DBITS-1:0] rd_val_MEM_w;

  reg [REGNOBITS-1:0] wregno_MEM;
  reg [INSTBITS-1:0] inst_MEM; /* This is for debugging */
  reg [DBITS-1:0] PC_MEM; /* This is for debugging */
  reg [DBITS-1:0] regval_MEM;  
  reg ctrlsig_MEM;
  // D-MEM
  (* ram_init_file = IDMEMINITFILE *)
  reg [DBITS-1:0] dmem[DMEMWORDS-1:0];

  assign memaddr_MEM_w = aluout_EX;
  assign wregno_MEM_w = wregno_EX;
  
  assign rd_mem_MEM_w = ctrlsig_EX[2];
  assign wr_mem_MEM_w = ctrlsig_EX[1];
  assign wr_reg_MEM_w = ctrlsig_EX[0];
  // Read from D-MEM
  assign rd_val_MEM_w = (memaddr_MEM_w == ADDRKEY) ? {{(DBITS-KEYBITS){1'b0}}, ~KEY} :
                                    dmem[memaddr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]];

  // Write to D-MEM
  always @ (posedge clk) begin
    if(wr_mem_MEM_w)
      dmem[memaddr_MEM_w[DMEMADDRBITS-1:DMEMWORDBITS]] <= regval2_EX;
  end

  always @ (posedge clk) begin
    if(reset) begin
      PC_MEM      <= {DBITS{1'b0}};
      inst_MEM    <= {INSTBITS{1'b0}};
      regval_MEM  <= {DBITS{1'b0}};
      wregno_MEM  <= {REGNOBITS{1'b0}};
      ctrlsig_MEM <= 1'b0;
    end else begin
      PC_MEM      <= PC_EX;
      inst_MEM    <= inst_EX;
      regval_MEM  <= rd_mem_MEM_w ? rd_val_MEM_w : aluout_EX;
      wregno_MEM  <= wregno_MEM_w;
      ctrlsig_MEM <= ctrlsig_EX[0];
    end
  end






  //*** -------------------- WRITE-BACK STAGE -------------------- ***//
  //*** -------------------- WRITE-BACK STAGE -------------------- ***//
  //*** -------------------- WRITE-BACK STAGE -------------------- ***//
  //*** -------------------- WRITE-BACK STAGE -------------------- ***//
  //*** -------------------- WRITE-BACK STAGE -------------------- ***//

  wire wr_reg_WB_w; 
  // regs is already declared in the ID stage

  assign wr_reg_WB_w = ctrlsig_MEM;
  
  always @ (negedge clk) begin
    if(reset) begin
      regs[0] <= {DBITS{1'b0}};
      regs[1] <= {DBITS{1'b0}};
      regs[2] <= {DBITS{1'b0}};
      regs[3] <= {DBITS{1'b0}};
      regs[4] <= {DBITS{1'b0}};
      regs[5] <= {DBITS{1'b0}};
      regs[6] <= {DBITS{1'b0}};
      regs[7] <= {DBITS{1'b0}};
      regs[8] <= {DBITS{1'b0}};
      regs[9] <= {DBITS{1'b0}};
      regs[10] <= {DBITS{1'b0}};
      regs[11] <= {DBITS{1'b0}};
      regs[12] <= {DBITS{1'b0}};
      regs[13] <= {DBITS{1'b0}};
      regs[14] <= {DBITS{1'b0}};
      regs[15] <= {DBITS{1'b0}};
    end else if(wr_reg_WB_w) begin
      regs[wregno_MEM] <= regval_MEM;
    end
  end
  
  
  /*** I/O ***/
  // Create and connect HEX register
  reg [23:0] HEX_out;
  
  SevenSeg ss5(.OUT(HEX5), .IN(HEX_out[23:20]), .OFF(1'b0));
  SevenSeg ss4(.OUT(HEX4), .IN(HEX_out[19:16]), .OFF(1'b0));
  SevenSeg ss3(.OUT(HEX3), .IN(HEX_out[15:12]), .OFF(1'b0));
  SevenSeg ss2(.OUT(HEX2), .IN(HEX_out[11:8]), .OFF(1'b0));
  SevenSeg ss1(.OUT(HEX1), .IN(HEX_out[7:4]), .OFF(1'b0));
  SevenSeg ss0(.OUT(HEX0), .IN(HEX_out[3:0]), .OFF(1'b0));
  
  always @ (posedge clk) begin
    if(reset)
      HEX_out <= 24'hFEDEAD;
    else if(wr_mem_MEM_w && (memaddr_MEM_w == ADDRHEX))
      HEX_out <= regval2_EX[HEXBITS-1:0];
  end

  reg [9:0] LEDR_out;
 

  // **TODO: Write the code for LEDR here
  always @ (posedge clk) begin
    if(reset)
      LEDR_out <= 10'd0;
    else if(wr_mem_MEM_w && (memaddr_MEM_w == ADDRLEDR))
      LEDR_out <= regval2_EX[LEDRBITS-1:0];
  end
  assign LEDR = LEDR_out;
  
endmodule // project3_frame



module SXT(IN, OUT);
  parameter IBITS = 16;
  parameter OBITS = 32;

  input  [IBITS-1:0] IN;
  output [OBITS-1:0] OUT;

  assign OUT = {{(OBITS-IBITS){IN[IBITS-1]}}, IN};
endmodule
