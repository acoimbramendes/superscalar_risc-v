/////////////////////////////////// Processador IO2I //////////////////////////////////////
//grupo: Anderson Coimbra, Arthur Fortini, Frederico Chaves.
//
//RISC-V – utilizamos o I4 anterior como base.
//Disparo fora de ordem.
//Unidades funcionais do execute fora de ordem:
//  MULT = 4 ciclos
//  DIV = 6 ciclos
//  LOAD = 2 ciclos
//  STORE = 1 ciclo
//  ALU = 1 ciclo
//Escrita no registrador e na memória em ordem.
//Controle de Hazards
//  Estrutural WB e Unidades do execute (MULT, DIV, LOAD, STORE)
//  Controle de Hazard de controle
//  Controle de Hazard de dados:  
//    Encaminhamento
//    Disparo fora de ordem
//    Stall
/////////////////////////////////// Processador IO2I //////////////////////////////////////
module testbench();
  logic clock;
  logic reset;
  reg [31:0] i4_out;
  
  io2i io2i (.clk(clock), .rst(reset), .wb_writedata(i4_out));
  
  initial begin
    clock = 0;
    #1;
    forever begin 
      clock = ~clock; #1;
    end
  end
  integer j;
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
   	reset = 0;
    $monitor("%t wb_writedata = %h \n", $time, io2i.wb_writedata);
    repeat (20) @(posedge clock);
    reset = 1;
    for(j=0;j<16;j++) begin
      $display("PRF-R%d = %h",j, io2i.issue.PRF.memory[j]);
    end
    for(j=0;j<16;j++) begin
      $display("ARF-R%d = %h",j, io2i.ARF.memory[j]);
    end
    $display("MEM[10] = %h", io2i.execute.memory.ram[10]);
    $display("MEM[1] = %h", io2i.execute.memory.ram[1]);
    @(posedge clock);

    repeat (20) @(posedge clock);
    force io2i.issue_queue.read_enable = 1'b1;
    force io2i.issue_queue.write_enable = 1'b0;
    repeat (20) @(posedge clock);
    @(posedge clock)

    for(j=0;j<16;j++) begin
      $display("PRF-R%d = %h",j, io2i.issue.PRF.memory[j]);
    end
    for(j=0;j<16;j++) begin
      $display("ARF-R%d = %h",j, io2i.ARF.memory[j]);
    end
    $display("MEM[10] = %h", io2i.execute.memory.ram[10]);
    $display("MEM[1] = %h", io2i.execute.memory.ram[1]);
    
    for(j=0;j<16;j++) begin
      if(io2i.ARF.memory[j] != io2i.issue.PRF.memory[j]) begin
        $display("ERROR: PRF != ARF Reg= %d", j);
      	error = error + 1;
      end
      
    end
    if (error == 0 )
        $display("TESTE PASSOU: PRF == ARF ");
    $finish();
    

  end 
endmodule

module io2i (input clk, rst, output [31:0] wb_writedata, 
    
            output reg [31:0] commit_writedata,        
            //FLAGS de erro
            output reg error_rob_buffer_full, 
            output reg error_fsb_buffer_full,      
            output reg error_fsb_buffer_empty
            );

  wire [31:0] sigext, data1, data2, aluout, readdata, inst, data1_prop, data2_prop, sigext_prop;
  wire zero, memread, memwrite, memtoreg, branch, alusrc, alusrc_prop, memread_prop, memwrite_prop, memtoreg_prop, branch_prop;
  wire [9:0] funct, funct_prop;
  wire [2:0] aluop, aluop_prop;
  wire [4:0] rs1_address, rs2_address, d_address, d_address_prop, d_address_prop_two, wb_regwrite_addr;
  wire [3:0] op_type, op_type_prop;

  //Adicionado
  wire [31:0] d_inst, d_pc;
  
  wire clk_fetch, clk_decode, clk_reg_bank, clk_issue, clk_exec,clk_exec_free ;
  
  wire wb_regwrite, regwrite_decode_out, regwrite_decode_out_prop;
  wire alu_memtoreg_out, alu_regwrite_out,  alu_memread_out,  alu_memwrite_out;
  wire [31:0] alu_data2;
  wire mem_regwrite_out, mem_memtoreg_out;
  wire [31:0] mem_aluout;
  
  // ACM ADICIONADO - MOVER pro topo
  wire [7:0] new_intr_to_rob;
  wire [3:0] wb_finish_addr;
  wire store_instr;
  wire [31:0] store_addr;
  wire [31:0] store_data;
  wire fsb_we;
  wire [31:0] data_to_mem,data_to_mem_fsb;
  wire [31:0] mem_addr,addr_to_mem_fsb ;
  
  wire commiting;
  wire copy_to_mem_en;
  wire [9:0] rob_out;
  reg [3:0] reorder_id_next,reorder_id_to_exe,reorder_id_to_wb,reorder_id_to_iq, wb_finish_reorder_id;
  
  wire exec_foward_A, exec_foward_B;
  wire [1:0] exec_foward_stage_a,exec_foward_stage_b;
  wire [2:0] exec_foward_stage_cycle_a,exec_foward_stage_cycle_b;
  
  wire [4:0] scoreboard_line;
  wire [8:0] scoreboard_data;
  wire scoreboard_write, scoreboard_read;
  
  reg write_iq_enable, iq_read_en;
  reg [57:0] iq_data;
  reg [61:0] issued_instruction;
  reg stall_rob_wr;
  wire branch_from_decode;
  reg branch_stall;

  //STALL FORCE LOGIC
  assign stall_rob_wr = (execute.alu_stall_req | execute.stall_wb_request);
  assign clk_fetch    = (branch_stall | execute.alu_stall_req | execute.stall_wb_request) ? 1'b1 : clk;
  assign clk_decode   = (execute.alu_stall_req | execute.stall_wb_request) ? 1'b1 : clk;
  assign clk_issue    = (execute.alu_stall_req | execute.stall_wb_request) ? 1'b1 : clk;
  assign clk_exec = execute.stall_wb_request ?  1'b1 : clk;
  assign clk_exec_free = clk;
  assign clk_reg_bank = clk;

  //branch stall logic
  always @(posedge branch or posedge branch_from_decode or rst) begin
    if (!rst) begin
      	branch_stall <= 0;
    end else if(branch) begin
      	branch_stall <= 0;
    end else begin
    	branch_stall <= branch_from_decode;
    end
      
  end

  // FETCH STAGE - Adicionado sinais de controle
  fetch fetch (
			.zero(zero), .rst(rst), 
    		.clk(clk_fetch), .branch(branch), 
    		.sigext(sigext), .d_inst(d_inst), .d_pc(d_pc)
  			);

  // DECODE STAGE - Gerar sinais de controle
  decode decode (
    			.inst(d_inst), 
    			.clk(clk_decode),
    			.rst(rst),
    			.branch_stall(branch_stall || branch),
                .branch (branch_from_decode)
    
    	       //ACM ROB
    			,.data_to_rob(new_intr_to_rob)
    			,.reorder_id_next(reorder_id_next)
    			,.reorder_id(reorder_id_to_iq)
    			//ACM IQ
    		   ,.write_iq_enable(write_iq_enable)
			   ,.iq_data(iq_data)   
    	
	);

  // ISSUE STAGE 
  issue issue (
        .clk(clk_issue), .rst(rst), 
    		    
    	//Sinais de wb para escrever no reg
        .writedata(wb_writedata), 
    	.wb_regwrite_addr(wb_regwrite_addr),
  		.regwrite_in(wb_regwrite), 
    	.clk_register_bank(clk_reg_bank),
    
    	//Sinais SB
        .sb_register(scoreboard_line),
        .sb_data(scoreboard_data),
        .sb_write(scoreboard_write),
    
    	//Sinais Foward
        .foward_A(exec_foward_A), .foward_B(exec_foward_B), 
        .foward_stage_a(exec_foward_stage_a), .foward_stage_b(exec_foward_stage_b), 
        .foward_stage_cycle_a(exec_foward_stage_cycle_a),
    	.foward_stage_cycle_b(exec_foward_stage_cycle_b),
    	
    	//sinais pro execute
        .rd_out(d_address_prop), .operation_type_out(op_type_prop),
        .e_data1(data1_prop), .e_data2(data2_prop), 
	    .e_memread(memread_prop), .e_memwrite(memwrite_prop),
        .e_memtoreg(memtoreg_prop), .e_branch(branch_prop),
    	.e_regwrite_decode_out(regwrite_decode_out_prop),
    	.e_ImmGen(sigext_prop),	
    	.e_alusrc(alusrc_prop)
    
    	//Sinais reorder
        ,.reorder_id_out(reorder_id_to_exe)
    
    	//IQ
    	,.issued_instruction(issued_instruction)
    	,.read_iq(iq_read_en)
		);

  // EXECUTE STAGE
  execute execute (
			//Sinais
			.clk(clk_exec), .rst(rst), .clk_free(clk_exec_free),
    		.in1(data1_prop), .in2(data2_prop), .ImmGen(sigext_prop), 
    		.alusrc(alusrc_prop), .aluout(aluout),
    		
    		.branch(branch_prop), .branch_out(branch),
    		.zero(zero),.sigext_branch(sigext),
    
			//Propagando sinais de controle pelo pipeline
			.memtoreg_in(memtoreg_prop),
    		.regwrite_in(regwrite_decode_out_prop),
    		.memread_in(memread_prop), 
    		.memwrite_in(memwrite_prop),
			.memtoreg_out(alu_memtoreg_out), 
    		.regwrite_out(alu_regwrite_out), 
    		.memwrite_out(alu_memwrite_out),
    
             //forward
            .foward_A(exec_foward_A), .foward_B(exec_foward_B), 
            .foward_stage_a(exec_foward_stage_a), .foward_stage_b(exec_foward_stage_b), 
            .foward_stage_cycle_a(exec_foward_stage_cycle_a),
    		.foward_stage_cycle_b(exec_foward_stage_cycle_b),
	
    		.operation_type(op_type_prop), .e_rd_in(d_address_prop),
    		.e_rd_out(d_address_prop_two),	
    		.memdata_out(readdata),
    		.reorder_id_from_issue(reorder_id_to_exe),
    		.reorder_id_to_wb(reorder_id_to_wb),
    		.data_to_mem(store_data),
    		.writeaddr_to_mem(store_addr)

			);
  
  
  // WRITEBACK STAGE
  writeback writeback (
			.rst(rst), .clk(clk),.aluout(aluout),
			.readdata(readdata),
			.memtoreg(alu_memtoreg_out),
			.regwrite_in(alu_regwrite_out),
			.regwrite(wb_regwrite),
			.write_data(wb_writedata),
            .w_rd_in(d_address_prop_two),
			//ACM ADICIONADO
			.reg_addr(wb_regwrite_addr),
            .reorder_id(reorder_id_to_wb),
			.wb_finish_reorder_id(wb_finish_reorder_id), //rob addr
			
			//Sinais para fsb - vem do exe - STORE
			.store_instr(alu_memwrite_out),
			.store_addr(store_addr),
			.store_data(store_data),
			.data_to_mem(data_to_mem),
			.mem_addr(mem_addr),
			.fsb_we(fsb_we)
			);
			

  //IQ + Scoreboard
  scoreboard scoreboard (.clk(clk_decode), .read(scoreboard_read), .sb_write(scoreboard_write), .sb_data(scoreboard_data), .sb_register(scoreboard_line));
  
  issue_queue issue_queue (
    .clk(clk_issue), .resetn(rst), 
                    .write_enable(write_iq_enable),
                    .reorder_id(reorder_id_to_iq),
                    .data_in(iq_data),
    .read_enable(iq_read_en),
                    .data_out(issued_instruction),
                    .full(),.empty()
  );//iq_read_en
  integer h;
  reg [8:0] sb_table_data_r1[7:0];
  reg [8:0] sb_table_data_r2[7:0];
  //Sinais para Debug
  reg [4:0] r1 [7:0];
  reg [4:0] r2 [7:0];
  reg [7:0]  pdeing_r1 ;
  reg [7:0]  pdeing_r2 ;
  reg [7:0]  qt_1;
  reg [7:0]  qt_2;
  //atualizando issue_queue pending
  always @ (*) begin
    if (rst) begin
      for (h=0; h<8; h++) begin
        //sb_table_data_r2[h][1] sb_table_data_r2[h][1] -> Por causa do fw
        sb_table_data_r2[h] = scoreboard.sb_table[issue_queue.q_table[h][4:0]];
        issue_queue.q_table[h][5]  = sb_table_data_r2[h][8] & !sb_table_data_r2[h][0];
        sb_table_data_r1[h] = scoreboard.sb_table[issue_queue.q_table[h][11:7]];
        issue_queue.q_table[h][12] = sb_table_data_r1[h][8]  & !sb_table_data_r1[h][0];

        //sinais para debug
        r1[h] = issue_queue.q_table[h][4:0];
        r2[h] = issue_queue.q_table[h][11:7];
        pdeing_r1[h] = sb_table_data_r1[h][8]  & !sb_table_data_r1[h][0];
        pdeing_r2[h] = sb_table_data_r2[h][8]  & !sb_table_data_r2[h][0];
        qt_1[h] = issue_queue.q_table[h][12];
        qt_2[h] = issue_queue.q_table[h][5];
      end
    end
  end
  
  
	//////////////Commit + FSB + ROB/////////////////
    reg wb_finish;
    assign wb_finish = alu_regwrite_out | alu_memwrite_out ;
  reorder_buffer rob (
    			.clk(clk),
				.rstn(rst),
    			.stall(stall_rob_wr),
				.new_instruction(new_intr_to_rob),
				.wb_finish(wb_finish),
				.wb_finish_reorder_id(reorder_id_to_wb),
				.commit_rd(commiting),
                .empty(), .full(error_rob_buffer_full), //flag de erro
    			.rob_out(rob_out),
    			.reorder_id_next(reorder_id_next)
  				);

  reg[31:0] addr_to_mem;
  finish_store_buffer fsb ( //Uma posição de memoria só, mas gera uma flag de error
				.rstn(rst),
				.clk(clk),
				.data_in(data_to_mem),
				.addr_in(mem_addr),
				.we(fsb_we),
				.commit_rd(copy_to_mem_en),
				.error_fsb_buffer_full(error_fsb_buffer_full),      //FLAGS de erro
				.error_fsb_buffer_empty(error_fsb_buffer_empty),
    .data_to_mem(data_to_mem_fsb),
    .addr_to_mem(addr_to_mem_fsb)
			  );

  //LOGICA AUXILIAR para escrita em ordem na memoria usando os dados do FSB
  assign execute.memory.w_data = data_to_mem_fsb; 
  assign execute.memory.w_en = copy_to_mem_en;
  assign execute.memory.w_addr = addr_to_mem_fsb;
  
  reg[4:0] copy_prf_to_arf_addr;
  reg copy_prf_to_arf_en;
  
  commit commit (
    		.rstn(rst),
    		.clk(clk),
			.commiting(commiting),
			.rob_data(rob_out),
            .copy_to_mem_en(copy_to_mem_en),
			.copy_prf_to_arf_en(copy_prf_to_arf_en),
			.copy_prf_to_arf_addr(copy_prf_to_arf_addr)
			);
			
  reg[31:0] PRF_data;
  
  //LOGICA AUXILIAR para escrita em ordem no ARF copiando os dados do PRF
  assign PRF_data = issue.PRF.memory[copy_prf_to_arf_addr]; 		//XMR
  assign commit_writedata = copy_prf_to_arf_en ? PRF_data : 'bZ;    // Sinal para faciltar o debug apenas

  Register_Bank_1port ARF (  .clk(clk),
							  .regwrite(copy_prf_to_arf_en),
							  .writedata(PRF_data),
							  .writereg(copy_prf_to_arf_addr),
							  .read_data(), //TODO colocar no output
							  .read_reg()
	                 		  ); 
  
  
endmodule
reg error = 0;





`define AND 10
`define OR  9
`define ADD 3
`define SUB 5
`define MULT 4 
`define DIV 8   
`define LOAD 12  
`define STORE 13  
`define BEQ 6  

//MACROS pro ROB
`define PENDING         2'b10
`define FINISH          2'b11
`define NOT_SPECULATIVE 1'b0
`define SPECULATIVE     1'b1
`define INST_STORE      1'b1
`define NOT_INST_STORE  1'b0
`define VALID           1'b1
`define NOT_VALID       1'b0

///////////FETCH//////////////////////////////////////////////////

module fetch (input zero, rst, clk, branch, input [31:0] sigext, output reg [31:0] d_inst,d_pc);
  wire [31:0] pc, pc_4, new_pc;
  wire [31:0] inst;
  assign pc_4 = 4 + pc; // pc+4  Adder
  assign new_pc = (branch & zero) ? pc_4 + sigext : pc_4; // new PC Mux

  reg [31:0] inst_mem;
  assign inst = inst_mem;

  //Módulo de PC
  PC program_counter(new_pc, clk, rst, pc);
  //Criando um modulo só para a memoria de instrução
  InstructionMem instruction_memory (
    .rst(rst),
    //.addr(pc[31:2]), //TODO CHECAR
    .addr(pc[31:2]),
    .inst_out(inst_mem)
   );

  //PIPELINE IF para DECODE - adicionando o bloco sequencial
  always @(posedge clk) begin
    if (!rst) begin
	  d_inst   <= 0;
	  d_pc     <= 0;
	end
    else begin
	  d_inst   <= inst;
	  d_pc     <= pc_4;
    end
  end
endmodule

module PC (input [31:0] pc_in, input clk, rst, output reg [31:0] pc_out);
  always @(posedge clk) begin
    pc_out <= pc_in;
    if (~rst)
      pc_out <= 0;
  end
endmodule

module InstructionMem (
  input rst,
  //input [31:0] addr,
  input  [29:0] addr,
  output [31:0] inst_out
);

  reg [31:0] inst_mem [0:127];
  //reg [31:0] inst_mem [0:31];

  assign inst_out = inst_mem[addr];

  initial begin
    // Exemplos
    //inst_mem[0] <= 32'h00000000; // nop
    //inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    //inst_mem[2] <= 32'h00210233; // add  x4, x2, x2  ok
    //inst_mem[1] <= 32'h00202223; // sw x2, 8(x0) ok
    //inst_mem[1] <= 32'h0050a423; // sw x5, 8(x1) ok
    //inst_mem[2] <= 32'h0000a003; // lw x1, x0(0) ok
    //inst_mem[1] <= 32'hfff00113; // addi x2,x0,-1 ok
    //inst_mem[2] <= 32'h00318133; // add x2, x3, x3 ok
    //inst_mem[3] <= 32'h40328133; // sub x2, x5, x3 ok
/*
#programa do risc-v
NOP
ADDi x2 x0 5
ADD  x4 x2 x2
#Programa da aula
MUL x5 x6 x7
ADDI x15 x14 1
MUL x9 x5 x8
MUL x7 x9 x10
ADDI x12 x11 1
ADDI x13 x12 1
ADDI x14 x12 2
*/
    
    //test instruções + disparo ooo + execute ooo + Hazard estrutural + encaminhamento
	inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h00210233; // add  x4, x2, x2  
    inst_mem[3] <= 32'h027302b3; // MUL x5 x6 x7
    inst_mem[4] <= 32'h00518333; // add x6 x3 x5
    inst_mem[5] <= 32'h00400113; // addi x2, x0, 4  ok
    inst_mem[6] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[7] <= 32'h00600113; // addi x2, x0, 6  ok
    inst_mem[8] <= 32'h00100393; // addi x7, x0, 1  ok
    inst_mem[9] <= 32'h0090a023; // SW x9, 0(x1)
    inst_mem[10] <= 32'h00052183;// LW x3, 0(x10)
    inst_mem[11] <= 32'h00000000;// nop
    
/*
	//test commit in-order
    inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h027302b3; //MUL x5 x6 x7
    inst_mem[3] <= 32'h00518233;  // add x4 x3 x5
    inst_mem[4] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[5] <= 32'h00052183; //LW x3, 0(x10)
    inst_mem[6] <= 32'h0090a023; //SW x9, 0(x1)
    inst_mem[7] <= 32'h00000000; // nop
  */
    
    //test hazard estrutural - DIV
  /*  inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h00518233; // add x4 x3 x5
    inst_mem[3] <= 32'h027302b3; // MUL x5 x6 x7
    inst_mem[4] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[5] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[6] <= 32'h00000000; // nop
    */
/*
    //test hazard estrutural - WB
    inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[3] <= 32'h00518233;  // add x4 x3 x5
    inst_mem[4] <= 32'h027302b3; //MUL x5 x6 x7
    inst_mem[5] <= 32'h00518233;  // add x4 x3 x5
    inst_mem[6] <= 32'h00000000; // nop
 */  

/*	//  BEQ + stall
    inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00100393; //ADDi x7 x0 1
    inst_mem[2] <= 32'h00138463; //BEQ x7,x1, B
    inst_mem[3] <= 32'h00200393; //ADDi x7 x0 2
    inst_mem[4] <= 32'h00300313; //B: ADDi x6 x0 2
*/
    
  /*  
    //test hazard de dados
    //inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    //inst_mem[2] <= 32'h00210233; // add  x4, x2, x2  ok -> hazard de dados

	//inst_mem[4] <= 32'h00170793; 
	//inst_mem[5] <= 32'h028284b3; 
	//inst_mem[6] <= 32'h02a483b3; 
	//inst_mem[7] <= 32'h00158613; 
	//inst_mem[8] <= 32'h00160693; 
	//inst_mem[9] <= 32'h00260713; 
*/	
  end
endmodule

/////////////////////////////////////////DECODE//////////////////////////////////////////////////

module decode (
				input [31:0] inst, //writedata,
				input clk, rst, //clk_register_bank,rst,
                output reg write_iq_enable,
  				output reg [57:0] iq_data,                
  				
  				//ACM2 ADICIONADO
  				output reg [7:0] data_to_rob,
  				input  [3:0] reorder_id_next,
  				output reg [3:0] reorder_id,
  				output reg branch,
  				input branch_stall
  
				);

  wire [4:0] rs1, rs2, rd;

  //reg [31:0] data1_pipe, data2_pipe, ImmGen_pipe, writedata_pipe;
  reg [31:0] ImmGen;
  reg [2:0] aluop;
  wire [6:0] opcode;
  reg [9:0] funct;
  reg alusrc, branch_dec;

  //reg alusrc_pipe, memtoreg_pipe, regwrite_pipe;
  //reg memread_pipe, memwrite_pipe, branch_pipe;
  
   //ACM2 ADICIONADO
  reg speculative;
  assign speculative = 1'b0; //No caso nao vai ter instrução especulativa, branch causa stalls, mas teria que alterar essa logica pra fazer - o ROB tem suporte pra especulativo 
  
  ///AF
  reg [3:0] aluctrl;

  assign opcode = inst[6:0];
  assign rs1    = inst[19:15];
  assign rs2    = inst[24:20];
  assign rd     = inst[11:7];
  assign funct = {inst[31:25],inst[14:12]}; 

  ControlUnit_dec control_dec (rst,opcode, inst, aluop, alusrc, ImmGen, branch_dec);
    
  alucontrol alucontrol (aluop, funct, aluctrl);
 
  always @ (posedge clk) begin
    branch <= branch_dec;
  end

  reg [8:0] aux_source1_line;
  reg [8:0] aux_source2_line;
  reg [8:0] aux_dest_line;
  reg pending_src1, pending_src2, valid_src1, valid_src2, valid_dest;

  always @(*) begin
    aux_source1_line = scoreboard.sb_table[rs1];
    aux_source2_line = scoreboard.sb_table[rs2];
    aux_dest_line = scoreboard.sb_table[rd];

    if (aux_source1_line[8] == 1) begin pending_src1 = 'b1; end else pending_src1 = 'b0;
    if (aux_source2_line[8] == 1) begin pending_src2 = 'b1; end else pending_src2 = 'b0;

    case (aluctrl)
        4'd`ADD, 4'd`SUB, 4'd`AND, 4'd`OR,4'd`MULT,4'd`DIV, 4'd`BEQ: begin
          if (alusrc) begin    //addi, subi, sub to branch, load
                valid_src1 = 'b1;
                valid_src2 = 'b0;
                valid_dest = 'b1;
            end
            else begin 
                valid_src1 = 'b1;
                valid_src2 = 'b1;
                valid_dest = 'b1;
            end
        end
        4'd`STORE: begin
            valid_src1 = 'b1;
            valid_src2 = 'b1;
            valid_dest = 'b0;
        end
        default: begin
            valid_src1 = 'b1;
            valid_src2 = 'b1;
            valid_dest = 'b1;
        end
    endcase
  end

  always @(posedge clk) begin
    if (!rst || branch_stall || branch) begin
        iq_data <= 'b0;
        write_iq_enable = 'b0;
    end
    else begin
      if (inst == 'b0 || inst === 'bx) begin
        iq_data <= 'b0;
        write_iq_enable <= 'b0;
      end else begin
        iq_data <= {alusrc, aluctrl, ImmGen, speculative, valid_dest, rd, valid_src1, 1'bZ, rs1, valid_src2, 1'bZ, rs2};
        write_iq_enable <= 'b1;
      end
    end
  end


 always @(posedge clk) begin
    reorder_id <= reorder_id_next;
 end
 always @ * begin
   if(branch_stall  || branch_dec || branch) begin
   	  data_to_rob = 'b0;
   end else begin
      case (aluctrl)
          4'd`ADD, 4'd`SUB, 4'd`AND, 4'd`OR,4'd`MULT,4'd`DIV: begin 
                      if(!speculative) begin
                          data_to_rob = {3'b001,rd};
                      end else begin
                          data_to_rob = {3'b101,rd};
                      end
                  end       
          4'd`LOAD: begin
                      //load
                      if(!speculative) begin
                          data_to_rob = {3'b001,rd};
                      end else begin
                          data_to_rob = {3'b101,rd};
                      end
              end
          4'd`STORE: begin
                      //Store
                      if(!speculative) begin
                          data_to_rob = {3'b010,rd};
                      end else begin
                          data_to_rob = {3'b110,rd};
                      end
                  end

          4'd6: begin //Branch
                  data_to_rob = {3'b011,rd};
          end 
          default : begin 
                  if(!speculative) begin
                    data_to_rob = {3'b000,rd};
                  end else begin
                    data_to_rob = {3'b100,rd};
                  end
          end
      endcase
   end
 end
endmodule

module ControlUnit_dec (
					input rst,
					input [6:0] opcode,
					input [31:0] inst,
  					output reg [2:0] aluop,
  					output reg alusrc,
  					output reg [31:0] ImmGen,
  					output reg branch
					);

  always @(*) begin
		aluop    <= 0;
		ImmGen   <= 0;
    	alusrc   <= 0;
    	branch   <= 0;
		case(opcode)
		  7'b0110011: begin // R type == 51
			//regwrite <= 1;
			aluop    <= 2;
				end
			  7'b1100011: begin // beq == 99
			branch   <= 1;
			aluop    <= 1;
			ImmGen   <= {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],1'b0};
				end
				7'b0010011: begin // addi == 19
            aluop    <= 2;
			alusrc   <= 1;
			ImmGen   <= {{20{inst[31]}},inst[31:20]};
		  end
				7'b0000011: begin // lw == 3
            aluop    <= 3;
			alusrc   <= 1;
			ImmGen   <= {{20{inst[31]}},inst[31:20]};
		  end
				7'b0100011: begin // sw == 35
            aluop    <= 4;
			alusrc   <= 1;
			ImmGen   <= {{20{inst[31]}},inst[31:25],inst[11:7]};
		  end
		endcase
  end
endmodule

module Register_Bank (input clk, regwrite, input [4:0] read_reg1, read_reg2, writereg, input [31:0] writedata, output [31:0] read_data1, read_data2);
  integer i;
  reg [31:0] memory [0:31]; // 32 registers de 32 bits cada

  // fill the memory
  initial begin
    for (i = 0; i <= 31; i++)
      memory[i] <= i;
  end

  assign read_data1 = (regwrite && read_reg1==writereg) ? writedata : memory[read_reg1];
  assign read_data2 = (regwrite && read_reg2==writereg) ? writedata : memory[read_reg2];

  always @(posedge clk) begin
    if (regwrite)
      memory[writereg] <= writedata;
  end
endmodule

module alucontrol (input [2:0] aluop, input [9:0] funct, output reg [3:0] alucontrol);
  wire [7:0] funct7;
  wire [2:0] funct3;

  assign funct3 = funct[2:0];
  assign funct7 = funct[9:3];

  always @(*) begin
    case (aluop)
      0: alucontrol <= 4'd15; 
      1: alucontrol <= 4'd6; // SUB to branch
      3: alucontrol <= 4'd`LOAD;
      4: alucontrol <= 4'd`STORE;
      default: begin
        case (funct3)
          0: begin
            if(funct7 == 0) begin 
              	alucontrol <=  4'd`ADD; /*ADD*/
            end else if (funct7 == 1) begin
                alucontrol <=  4'd`MULT; //MULT
            end else if (funct7 == 32) begin
                alucontrol <=  4'd`SUB; //SUB
            end
            else begin
              alucontrol <=  4'd`ADD; /*ADD - just in case*/
            end
          end
          2: alucontrol <= 4'd7; // SLT
          4: alucontrol <= 4'd`DIV; // DIV
          6: alucontrol <= 4'd`OR; // OR
          //39: alucontrol <= 4'd`NOR; // NOR
          7: alucontrol <= 4'd`AND; // AND
          default: alucontrol <= 4'd15; // Nop
        endcase
      end
    endcase
  end
endmodule

/////////////////////////////////////////ISSUE//////////////////////////////////////////////////

module issue (
            input clk, rst,

            input clk_register_bank,
  
  			//wb escrita no registrador (PRF)
            input [31:0] writedata,
            input regwrite_in,
            input[4:0] wb_regwrite_addr,  
  
  			//SB
            output logic [4:0] sb_register,
  			output logic [8:0] sb_data,
            output reg sb_write,
            
			//FW
            output reg foward_A, foward_B,
			output reg [1:0] foward_stage_a,
  			output reg [1:0] foward_stage_b,
  			output reg [2:0] foward_stage_cycle_a,
  			output reg [2:0] foward_stage_cycle_b,

            output reg [4:0] rd_out,
            output reg [3:0] operation_type_out,
            output reg [31:0] e_data1, e_data2, e_ImmGen,
	        output reg e_alusrc, e_memread, e_memwrite, e_memtoreg, e_branch,
	        
	        output reg e_regwrite_decode_out,
			
			//ROB
  			output reg [3:0] reorder_id_out,
  
			//IQ
  			input [61:0] issued_instruction,
  			output reg read_iq
);

integer i;
wire [4:0] rs1, rs2, rd;
reg [31:0] data1_pipe, data2_pipe, ImmGen, writedata_pipe;
wire [3:0] operation;
reg memtoreg_pipe, regwrite_pipe;
reg memread_pipe, memwrite_pipe, branch_pipe;
reg [3:0] reorder_id;
reg alusrc;

reg [56:0] aux_iq_lines;
  
  always @ (posedge clk) begin
    if (rst)
    	read_iq = 1'b1;
    else
      	read_iq = 1'b0;
  end

ControlUnit_iss control_iss (rst, operation, memtoreg_pipe, regwrite_pipe, memread_pipe, memwrite_pipe, branch_pipe);  
Register_Bank PRF (clk_register_bank, regwrite_in, rs1, rs2, wb_regwrite_addr, writedata, data1_pipe, data2_pipe);


assign operation = issued_instruction[56:53];
assign ImmGen    = issued_instruction[52:21];
assign rs1       = issued_instruction[11:7];
assign rs2       = issued_instruction[4:0];
assign rd        = issued_instruction[18:14];
assign alusrc    =  issued_instruction[57];
assign reorder_id = issued_instruction[61:58];
assign sb_register = rd;

reg foward_A_pipe, foward_B_pipe;
reg [1:0] foward_stage_pipe_a,foward_stage_pipe_b;
reg [2:0] foward_stage_cycle_pipe_a,foward_stage_cycle_pipe_b;

reg [8:0] aux_rs1_line;
reg [8:0] aux_rs2_line;


initial begin
	foward_stage_pipe_a = 'b00;
  	foward_stage_pipe_b = 'b00;
	foward_stage_cycle_pipe_a = 'b0;
  	foward_stage_cycle_pipe_b = 'b0;
    foward_A_pipe = 'b0;
    foward_B_pipe = 'b0;
end

always @(posedge clk) begin
   reorder_id_out <= reorder_id;
end

always @(operation) begin
    sb_write = 'b0;
    case (operation)
        4'd`ADD, 4'd`SUB, 4'd`AND, 4'd`OR: begin sb_data = 'b100000001; sb_write = 'b1; end       
        4'd`LOAD:begin sb_data =  'b101000010; sb_write = 1; end 
        4'd`STORE:begin sb_data = 'b101000001; sb_write = 1; end 
        4'd`MULT: begin sb_data = 'b110001000; sb_write = 1; end
        4'd`DIV: begin sb_data =  'b111100000; sb_write = 1; end
        4'd`BEQ: sb_write = 'b0;                           
        default : sb_write = 'b0;
    endcase
end  

always @(*) begin
    aux_rs1_line = scoreboard.sb_table[rs1];
    aux_rs2_line = scoreboard.sb_table[rs2];

  if (aux_rs1_line[8] == 1 && (aux_rs1_line[5:2] == 0)) begin
        case (aux_rs1_line[7:6])
            0: begin   //ADD SUB OR AND operation path
                foward_A_pipe = 'b1;
                foward_stage_pipe_a = 'b00;
            end
            1: begin  //LW, SW operation path
                foward_A_pipe = 'b1;
                foward_stage_pipe_a = 'b01;
            end
            2: begin  //MULT operation path
                foward_A_pipe = 'b1;
                foward_stage_pipe_a = 'b10;
            end
            3: begin  //DIV operation path
                foward_A_pipe = 'b1;
                foward_stage_pipe_a = 'b11;
            end
          	default: begin
              foward_A_pipe = 'b0;
              foward_stage_pipe_a = 'b00;
            end
        endcase
    	foward_stage_cycle_pipe_a = aux_rs1_line[1];
    end
	  else begin
      foward_A_pipe = 'b0;
    end

    if (aux_rs2_line[8] == 1) begin
        case (aux_rs2_line[7:6])
            0: begin   //ADD SUB OR AND operation path
                foward_B_pipe = 'b1;
                foward_stage_pipe_b = 'b00;
            end
            1: begin  //LW, SW operation path
                foward_B_pipe = 'b1;
                foward_stage_pipe_b = 'b01;
            end
            2: begin  //MULT operation path
                foward_B_pipe = 'b1;
                foward_stage_pipe_b = 'b10;
            end
            3: begin   //DIV operation path
                foward_B_pipe = 'b1;
                foward_stage_pipe_b = 'b11;
            end
          	default: begin
              foward_B_pipe = 'b0;
              foward_stage_pipe_b = 'b00;
            end
        endcase

       foward_stage_cycle_pipe_b = aux_rs2_line[0];
      
    end
  	else begin
      foward_B_pipe = 'b0;
    end
end

//PIPE ISS PARA EXE
always @(posedge clk) begin
    if (!rst) begin
		e_data1 		<= 'b0;
		e_data2 		<= 'b0;
		e_ImmGen 		<= 'b0;
		e_alusrc  	    <= 'b0;
		e_memread  	    <= 'b0;
		e_memwrite  	<= 'b0;
		e_memtoreg  	<= 'b0;
		e_branch  	    <= 'b0;
		e_regwrite_decode_out	<= 'b0;
        operation_type_out      <= 'b0;
        foward_A        <= 'b0;
        foward_B        <= 'b0;
        foward_stage_a    <= 'b0;
      	foward_stage_b    <= 'b0;
        rd_out          <= 'b0;
        foward_stage_cycle_a <= 'b0;
      	foward_stage_cycle_b <= 'b0;
    end
    else begin

		e_data1 		<= data1_pipe;
		e_data2 		<= data2_pipe;
		e_memread  	    <= memread_pipe;
		e_memwrite  	<= memwrite_pipe;
		e_memtoreg  	<= memtoreg_pipe;
		e_branch  	    <= branch_pipe;
		e_alusrc  	    <= alusrc;
		e_ImmGen 		<= ImmGen;
		e_regwrite_decode_out	<= regwrite_pipe;
        foward_A        <= foward_A_pipe;
        foward_B        <= foward_B_pipe;
        foward_stage_a  <= foward_stage_pipe_a;
      	foward_stage_b  <= foward_stage_pipe_b;
        rd_out          <= rd;
        foward_stage_cycle_a <= foward_stage_cycle_pipe_a;
      	foward_stage_cycle_b <= foward_stage_cycle_pipe_b;
        operation_type_out   <= operation;
		
    end
end
endmodule

module scoreboard (
    input clk,
    input read,
    input sb_write,
  	input logic [8:0] sb_data,
    input logic [4:0] sb_register
);
	
  	integer i;
  	logic [8:0] sb_table [0:31];
  
   // fill scoreboard
  	initial begin
    	for (i = 0; i <= 31; i++) 
      		sb_table[i] <= 'b0;
  	end

    always @(posedge clk) begin
        if (sb_write && (sb_register != 'b0000))
            sb_table[sb_register] <= sb_data;
    end

    always @(posedge clk) begin
      for (i = 1; i <= 31; i++) begin
        if(sb_table[i][8] == 1) begin
          if(sb_table[i][4:0] == 0) begin
            sb_table[i][8] <= 0;
          end else begin
            sb_table[i][4:0] <= sb_table[i][4:0] >> 1;
          end
        end
      end
    end
endmodule

module ControlUnit_iss (
					input rst,
					input [3:0] operation,
                    output reg memtoreg, regwrite, memread, memwrite, branch
);
    always @(*) begin
		memtoreg <= 0;
		regwrite <= 0;
		memread  <= 0;
		memwrite <= 0;
		branch   <= 0;
		case(operation)
		    4'd`ADD, 4'd`SUB, 4'd`AND, 4'd`OR,4'd`MULT,4'd`DIV: begin // R type
			    regwrite <= 1;
		    end
			4'd`BEQ: begin // beq 
			    branch   <= 1;
			end
			4'd`LOAD: begin // lw 
			    memtoreg <= 1;
			    regwrite <= 1;
			    memread  <= 1;
		    end
			4'd`STORE: begin // sw
			    memwrite <= 1;
		    end
		endcase
    end
endmodule

///////////////////////////////////////////////EXECUTE///////////////////////////////////////
module execute # (			
					parameter PIPELINE_DEPTH = 6,
					parameter DIV_CYCLES = PIPELINE_DEPTH,
					parameter MULT_CYCLES = 4
				)(
				input [31:0] in1, in2,
				output reg zero, //TODO CHECK
				output reg [31:0] aluout,

				//control signals
				input [31:0] ImmGen,
				input alusrc,
				input rst, clk, clk_free,
                input [3:0] operation_type,
  				output reg [31:0] sigext_branch,
  				input  branch,
  				output reg branch_out,

				//control signals no pipe
				input memtoreg_in, regwrite_in, memread_in, memwrite_in,
                input [4:0] e_rd_in, //AF
				output reg memtoreg_out, regwrite_out, memwrite_out,
                output reg [4:0] e_rd_out, //AF
				
				//ACM ADICIONADO
                output reg [31:0] memdata_out,
                input  [3:0]  reorder_id_from_issue,
                output reg [3:0]  reorder_id_to_wb,
                output reg [31:0] data_to_mem,
  				output reg [31:0] writeaddr_to_mem,
  
                //foward
  				input  [31:0] fw_wb_writedata,	
                input  foward_A, foward_B,
                input  [1:0] foward_stage_a,
                input  [1:0] foward_stage_b,
                input  [2:0] foward_stage_cycle_a,
                input  [2:0] foward_stage_cycle_b
				);

  reg  [31:0] data_A,data_B;
  wire [31:0] alu_B;
  wire [31:0] aluout_pipe; 

  reg zero_pipe;
  
  reg [31:0] writedata_pipe;

  assign alu_B = (alusrc) ? ImmGen : data_B ;
 
  //propagar sinais do branch

  always @ (posedge clk) begin
    sigext_branch <=ImmGen;
    branch_out <= branch;
  end
  
  always @* begin
    if (foward_A) begin
      case (foward_stage_cycle_a)
			1:
              data_A = fw_wb_writedata;
			0: 
         	  data_A = aluout;
        endcase
    end else begin
      		data_A = in1;
    end
    if (foward_B) begin
      case (foward_stage_cycle_b)
			1:
              data_B = aluout;
			0: 
              data_B = fw_wb_writedata;
        endcase
    end else begin
      		data_B = in2;
    end
  
  end
  
  //ACM Adicionado
  wire memwrite_en;
  
  wire [31:0] data_from_mem;
  wire [31:0] memdata;
  wire [31:0] readaddr_to_mem; 
  
  reg alu_stall_req;
  reg start_mult, start_div, start_basic, start_load, start_store;
  reg busy_basic, busy_mult, busy_div, busy_load, busy_store;
  reg done_basic, done_mult, done_div, done_load, done_store;
  reg [31:0] out_basic, out_mult, out_div;
  reg zero_basic;
  
  reg[3:0] reorder_id_to_wb_div, reorder_id_to_wb_mult, reorder_id_to_wb_basic, reorder_id_to_wb_load, reorder_id_to_wb_store;
  reg[4:0] e_rd_out_div, e_rd_out_mult, e_rd_out_basic, e_rd_out_load;
  reg regwrite_out_div, regwrite_out_mult, regwrite_out_basic, regwrite_out_load;
  reg memwrite_out_store;
  reg[31:0] memdata_out_load, data_to_mem_store, writeaddr_to_mem_store;
  reg memtoreg_out_load;
  reg stall_wb_request;
  
  always @ (posedge clk) begin
  		zero <= zero_basic;        //BASIC
  end
  //ALU PIPELINED
  alu_div #(.DATA_WIDTH(32), .NUMBER_CYCLES(DIV_CYCLES))
					alu_div (.a(data_A),.b(alu_B),
							 .busy(busy_div),
						     .out(out_div),.done(done_div),
						     .clk(clk),.start(start_div),.rst(rst),
							 
							 .reorder_id(reorder_id_from_issue),
							 .reorder_id_to_wb(reorder_id_to_wb_div),
							 
							 .rd_in(e_rd_in),
							 .rd_out(e_rd_out_div),
							 .regwrite_in(regwrite_in),
							 .regwrite_out(regwrite_out_div)
							 
							 );
		
  alu_mult #(.DATA_WIDTH(32), .NUMBER_CYCLES(MULT_CYCLES))
					alu_mult (.a(data_A),.b(alu_B),
							  .busy(busy_mult),
							  .out(out_mult),.done(done_mult),
							  .clk(clk),.start(start_mult),.rst(rst),
							  
							  .reorder_id(reorder_id_from_issue),
							  .reorder_id_to_wb(reorder_id_to_wb_mult),
							 
							  .rd_in(e_rd_in),
							  .rd_out(e_rd_out_mult),
							  .regwrite_in(regwrite_in),
							  .regwrite_out(regwrite_out_mult)
							  );
		    				
  alu_basic  #(.DATA_WIDTH(32))
					alu_basic(.alucontrol(operation_type),
							  .a(data_A),.b(alu_B),
							  .out(out_basic),
							  .start(start_basic),
                              .done(done_basic),
							  .zero(zero_basic),
                              .clk(clk),
							  .reorder_id(reorder_id_from_issue),
							  .reorder_id_to_wb(reorder_id_to_wb_basic),
							 
							  .rd_in(e_rd_in),
							  .rd_out(e_rd_out_basic),
							  .regwrite_in(regwrite_in),
							  .regwrite_out(regwrite_out_basic)
							  );
				
  store_unit store_unit(.clk(clk),
						.rst(rst),
                        .memwrite(memwrite_in), //store control signal
                        .data_in(data_B),
                        .A(data_A), .B(alu_B),
                        .memwrite_en(memwrite_out_store),
                        .data_to_mem(data_to_mem_store),
                        .writeaddr(writeaddr_to_mem_store),
						.start(start_store),
                        .done(done_store),
						.reorder_id(reorder_id_from_issue),
						.reorder_id_to_wb(reorder_id_to_wb_store)
							 

						);
						
  
  load_unit  load_unit (.clk(clk),
						.rst(rst),
                        .memread(memread_in), //load control signal
                        .A(data_A), .B(alu_B),
						.data_from_mem(data_from_mem),
						.data_out(memdata_out_load),
						.readaddr(readaddr_to_mem),
						.start(start_load),
                        .done(done_load),
                        .busy(busy_load),
						.reorder_id(reorder_id_from_issue),
						.reorder_id_to_wb(reorder_id_to_wb_load),
						
						.rd_in(e_rd_in),
						.rd_out(e_rd_out_load),
					    .regwrite_in(regwrite_in),
                        .regwrite_out(regwrite_out_load),
                        .memtoreg_in(memtoreg_in),
						.memtoreg_out(memtoreg_out_load)					
						
						);
						
  memory #(.DEPTH(128)) memory (
							.clk(clk),
							.w_en(),
                            .w_data(),//data_to_mem - FSB que vai escrever
                            .w_addr(), //writeaddr_to_mem - FSB que vai escrever
							.r_addr(readaddr_to_mem),
							.r_data(data_from_mem)
  						);
  

  //INPUT START LOGIC
  always @(*) begin
    case (operation_type)
      `MULT: begin
		  start_div  <= 1'b0;
		  start_basic <= 1'b0;
		  start_load <= 1'b0;
		  start_store <= 1'b0;
          if (!alu_stall_req)
            start_mult <= 1'b1;
           else
            start_mult <= 1'b0;
		end
      `DIV:  begin
		  start_mult  <= 1'b0;
		  start_basic <= 1'b0;
		  start_load <= 1'b0;
		  start_store <= 1'b0;
          if (!alu_stall_req)
			start_div  <= 1'b1;
          else
            start_div  <= 1'b0;
		end
 	  `AND,`OR,`ADD,`SUB: begin
			start_mult <= 1'b0;
			start_div  <= 1'b0;
			start_basic <= 1'b1;
			start_load <= 1'b0;
			start_store <= 1'b0;
		end
      `LOAD: begin
			start_mult <= 1'b0;
			start_div  <= 1'b0;
			start_basic <= 1'b0;
			start_load <= 1'b1;
			start_store <= 1'b0;
		end
      `STORE: begin
			start_mult <= 1'b0;
			start_div  <= 1'b0;
			start_basic <= 1'b0;
			start_load <= 1'b0;
			start_store <= 1'b1;
		end
	   default: begin
			start_mult <= 1'b0;
			start_div  <= 1'b0;
			start_basic <= 1'b0;
			start_load <= 1'b0;
			start_store <= 1'b0;
		end
    endcase
  end
  
  //Hazards Estruturais 
  //HAZARD Estrutural para unidades logicas (usar MULT or DIV antes de ter terminado)
  reg done_mult_dly, done_div_dly,done_load_dly;
  always @ (posedge clk) begin
    done_mult_dly  <= done_mult;
    done_div_dly   <= done_div;
	done_load_dly <= done_load;
  end
  
  always @(*) begin
	case (operation_type)
		`MULT: begin
          if (done_mult_dly)
            alu_stall_req <= 1'b0;
          else if(busy_mult)
			alu_stall_req <= 1'b1; //hw de mult sendo utilizado, gera stall de hw
		end
		`DIV: begin
          if (done_div_dly)
            alu_stall_req <= 1'b0;
          else if(busy_div)
			alu_stall_req <= 1'b1; //hw de mult sendo utilizado, gera stall de hw
		end
		`LOAD: begin
          if (done_load_dly)
            alu_stall_req <= 1'b0;
          else if(busy_load)
			alu_stall_req <= 1'b1; //hw de mult sendo utilizado, gera stall de hw
		end
	    default: alu_stall_req <= 1'b0; //default 0, Nada acontece;
	endcase
  end
  //hw estrutural WB
  reg [4:0] done_concat, done, done_and;
  assign done_concat = {done_div,done_mult,done_load,done_store,done_basic};
  assign done = done_concat & done_and;
  assign stall_wb_request = (done[0] + done[1] + done[2] + done[3] + done[4]) > 1;
  
  always @ (posedge clk_free) begin
	if (stall_wb_request)
		if(done[4]) begin
           done_and <= 5'b01111;
         end else if(done[3]) begin
           done_and <= 5'b00111;
         end else if(done[2]) begin
           done_and <= 5'b00011;
         end else if(done[1]) begin
           done_and <= 5'b00001;
         end else if(done[0]) begin
           done_and <= 5'b00000;
           end else begin
           done_and  <= 5'b11111;
         end
    else
      	 done_and  <= 5'b11111;
  end
  
  //mux para o output
  always @(posedge clk_free) begin
    // case (done) -> case 1xxxxx nao funcionou - usando if entao
        if(done[4]) begin // 5'b10000: begin
				reorder_id_to_wb <= reorder_id_to_wb_div; //ALL
				
				aluout       <= out_div;          //MULT + DIV + BASIC
				e_rd_out     <= e_rd_out_div;     //LOAD + MULT + DIV + BASIC 
				regwrite_out <= regwrite_out_div; //LOAD + MULT + DIV + BASIC 
				memdata_out  <= 'b0;              //LOAD
				memtoreg_out <= 'b0;      		  //LOAD
				memtoreg_out <= 'b0;      //STORE
				memwrite_out <= 'b0;      //STORE
                data_to_mem  <= 'b0;      //STORE
				writeaddr_to_mem  <= 'b0; //STORE
        end else if (done[3]) begin //5'b01000: begin
				reorder_id_to_wb <= reorder_id_to_wb_mult; //ALL
				
				aluout       <= out_mult;         //MULT + DIV + BASIC
				e_rd_out     <= e_rd_out_mult;    //LOAD + MULT + DIV + BASIC 
				regwrite_out <= regwrite_out_mult;//LOAD + MULT + DIV + BASIC 
				memdata_out  <= 'b0;              //LOAD
				memtoreg_out <= 'b0;      		  //LOAD
				memwrite_out <= 'b0;      //STORE
                data_to_mem  <= 'b0;      //STORE
				writeaddr_to_mem  <= 'b0; //STORE
        end else if (done[2]) begin //5'b00100: begin
				reorder_id_to_wb <= reorder_id_to_wb_load; //ALL
				
				aluout       <= 'b0;              //MULT + DIV + BASIC
				e_rd_out     <= e_rd_out_load;    //LOAD + MULT + DIV + BASIC 
				regwrite_out <= regwrite_out_load;//LOAD + MULT + DIV + BASIC 
				memdata_out  <= memdata_out_load; //LOAD
				memtoreg_out <= memtoreg_out_load;//LOAD
				memwrite_out <= 'b0;      //STORE
                data_to_mem  <= 'b0;      //STORE
				writeaddr_to_mem  <= 'b0; //STORE
        end else if (done[1]) begin //5'b00010: begin
				reorder_id_to_wb <= reorder_id_to_wb_store; //ALL
				
				aluout       <= 'b0;              //MULT + DIV + BASIC
				e_rd_out     <= 'b0;              //LOAD + MULT + DIV + BASIC 
				regwrite_out <= 'b0;              //LOAD + MULT + DIV + BASIC 
				memdata_out  <= 'b0;              //LOAD
				memtoreg_out <= 'b0;      		  //LOAD
				memwrite_out <= memwrite_out_store;      //STORE
                data_to_mem  <= data_to_mem_store;       //STORE
				writeaddr_to_mem  <= writeaddr_to_mem_store;   //STORE
        end else if (done[0]) begin //5'b00001: begin
				reorder_id_to_wb <= reorder_id_to_wb_basic; //ALL
				
				aluout       <= out_basic;         //MULT + DIV + BASIC
				e_rd_out     <= e_rd_out_basic;    //LOAD + MULT + DIV + BASIC 
				regwrite_out <= regwrite_out_basic;//LOAD + MULT + DIV + BASIC 
				memdata_out  <= 'b0;               //LOAD
				memtoreg_out <= 'b0;      		   //LOAD
				memwrite_out <= 'b0;      //STORE
                data_to_mem  <= 'b0;      //STORE
				writeaddr_to_mem  <= 'b0; //STORE
		end else begin // default: begin
				reorder_id_to_wb <= 'b0; //ALL
				
				aluout       <= 'b0;              //MULT + DIV + BASIC
				e_rd_out     <= 'b0;              //LOAD + MULT + DIV + BASIC 
				regwrite_out <= 'b0;              //LOAD + MULT + DIV + BASIC 
				memdata_out  <= 'b0;              //LOAD
				memtoreg_out <= 'b0;      		  //LOAD
				memwrite_out <= 'b0;      //STORE
                data_to_mem  <= 'b0;       //STORE
				writeaddr_to_mem  <= 'b0;   //STORE
		end
     // endcase
  end


endmodule



//ACM ADICIONADO
//1 ciclo: 1 para calculo do endereço, no proximo o sinal já é despachado
module store_unit (
				input clk,
  				input rst,
				input memwrite,
				input [31:0] data_in,
				input [31:0] A,B,
				output reg memwrite_en,
				output reg [31:0] data_to_mem,
				output reg [31:0] writeaddr,
				input start,
  				output done,
				input[3:0] reorder_id,
				output reg [3:0] reorder_id_to_wb
               );

	assign done = memwrite_en;
	always @(posedge clk) begin
      writeaddr <= A + B;
      if(!rst) begin
			memwrite_en <=1'b0;
      		data_to_mem <='b0;
			reorder_id_to_wb <= 'b0;
        end else if (start) begin
			reorder_id_to_wb <= reorder_id;
        	memwrite_en <=memwrite;
      		data_to_mem <= data_in;
        end
		else begin
			reorder_id_to_wb  <='b0;
        	memwrite_en <='b0;
      		data_to_mem <='b0;
		end
	end
endmodule

//2 ciclos: 1 para calculo do endereço e outro para receber o data
module load_unit (
				input clk,
  				input rst,
				input memread,
				input [31:0] A,B,
				input [31:0] data_from_mem,
				output reg [31:0] data_out,
				output reg [31:0] readaddr,
				input start,
  				output done,
				input[3:0] reorder_id,
				output reg [3:0] reorder_id_to_wb,
				input [4:0] rd_in,
				output reg [4:0] rd_out,
				input regwrite_in,
				output reg regwrite_out,
				input  memtoreg_in,
				output reg memtoreg_out,
				output reg busy
                );
	reg start_dly;

	assign busy = start_dly;
    assign done = start_dly;
    assign data_out = data_from_mem;	
  
	always @(posedge clk) begin
	
		if (start) begin
			start_dly <= start;
			readaddr <= A + B;
		end else begin
			start_dly <= 1'b0;
			readaddr <= 'b0;
		end
		
		

	end
  
	//propagar dados
	integer i;
	always @(posedge clk) begin
			reorder_id_to_wb <= reorder_id;
			rd_out <= rd_in;
			regwrite_out <= regwrite_in;
			memtoreg_out <= memtoreg_in;
	end



endmodule

/////////////////////////////////////////////WB//////////////////////////////////////
module writeback (
				  input rst, clk,
				  input [31:0] aluout, readdata,
				  input memtoreg,
				  input  regwrite_in,
                  input [4:0] w_rd_in,
				  output reg [31:0] write_data,
				  output reg regwrite,
				  
				  //ACM ADICIONADO
  				  output reg [4:0] reg_addr,
				  //Sinais para fsb - vem do exe - STORE
  				  input store_instr,
				  input [31:0] store_addr,
				  input [31:0] store_data,
		
				  output fsb_we,
				  output reg [31:0] data_to_mem,
                  output reg [31:0] mem_addr,			
                  input [3:0] reorder_id,
                  output [3:0] wb_finish_reorder_id
  
				  );

  assign wb_finish_reorder_id = reorder_id;
  
  assign regwrite = !rst ? 'b0 : regwrite_in;

  always @(*) begin
	write_data <= (memtoreg) ? readdata : aluout;
  end

  //ACM ADICIONADO
  assign reg_addr = w_rd_in;
  assign fsb_we = store_instr;
  assign data_to_mem = store_data;
  assign mem_addr = store_addr;

endmodule


/////////////////////////////////////ALU///////////////////////////////////

module alu_basic #(
					parameter  DATA_WIDTH = 32
				)
				(
				a,b,out,alucontrol,zero,clk, start, done,
				reorder_id,reorder_id_to_wb,rd_in,rd_out,
				regwrite_in,regwrite_out
				);

input clk,start;
output done;
output zero;
input[3:0] reorder_id;
output reg [3:0] reorder_id_to_wb;	 
input[4:0] rd_in;
output reg[4:0] rd_out;
input regwrite_in;
output reg regwrite_out;

input [3:0] alucontrol;
input [DATA_WIDTH-1:0] a,b;
output reg [DATA_WIDTH-1:0] out;
reg[31:0]out_pipe;
assign zero = (out == 0); // Zero recebe um valor lógico caso aluout seja igual a zero.

always @ * begin
	case(alucontrol)
        0: out <= a & b; // AND
        1: out <= a | b; // OR
        2: out <= a + b; // ADD
        6: out <= a - b; // SUB
		default:
        out <= a + b; // ADD
	endcase
end


reg[3:0] reorder_id_to_wb_pipe;
reg[4:0] rd_in_pipe;
reg      regwrite_in_pipe;

//propagar dados
integer i;
  always @(*) begin
	if(start) begin
		reorder_id_to_wb_pipe <= reorder_id;
		rd_in_pipe <= rd_in;
		regwrite_in_pipe <= regwrite_in;
	end	else begin
      	reorder_id_to_wb_pipe <= 'b0;
		rd_in_pipe <= 'b0;
		regwrite_in_pipe <= 'b0;
    end

end

assign regwrite_out = regwrite_in_pipe;
assign rd_out = rd_in_pipe;
assign reorder_id_to_wb = reorder_id_to_wb_pipe;
assign done = start;
  
endmodule

module alu_div #(
					parameter  DATA_WIDTH = 32,
					parameter  NUMBER_CYCLES = 6
				)
				(
				a,b,busy,cycles_to_finish,out,done,start,clk,rst,
				reorder_id,reorder_id_to_wb,rd_in,rd_out,
				regwrite_in,regwrite_out
				);

input[3:0] reorder_id;
output reg [3:0] reorder_id_to_wb;	 
input[4:0]  rd_in;
output reg[4:0] rd_out;
input regwrite_in;
output reg regwrite_out;

input [DATA_WIDTH-1:0] a,b;
  reg [DATA_WIDTH-1:0] result, a_in,b_in;
input  start,clk,rst;
reg start_div;
output reg [DATA_WIDTH-1:0] out;
output reg [DATA_WIDTH-1:0] cycles_to_finish;
output reg busy,done;
assign result = a_in/b_in;
assign busy = start_div;

//divisao multiciclo fake
assign done = cycles_to_finish == 1;
assign out = done ? result : 'b0; // 'a_in/b_in' tem NUMBER_CYCLES para ficar pronto - backend pode ser ajustado


always @ (posedge clk) begin
    if(!rst) begin
    	a_in <= 'b0;
		b_in <= 'b0;
		start_div <= 'b0;
    end else if(start) begin
		a_in <= a;
		b_in <= b;
		start_div <= 1'b1;
	end else if (done) begin
		start_div <= 1'b0;
	end else begin
		a_in <= a_in;
		b_in <= b_in;
		start_div <= start_div;
	end
    if (!rst) begin
      cycles_to_finish <= NUMBER_CYCLES - 1;
    end else if(start & !start_div) begin
		cycles_to_finish <= NUMBER_CYCLES - 1;
    end else if(start_div) begin
		cycles_to_finish <= cycles_to_finish - 1;
	end else begin
		cycles_to_finish <= cycles_to_finish;
	end
end


 reg[3:0] reorder_id_to_wb_pipe [NUMBER_CYCLES-2:0];
 reg[4:0] rd_in_pipe       [NUMBER_CYCLES-2:0];
 reg      regwrite_in_pipe [NUMBER_CYCLES-2:0];

//propagar dados
integer i;
always @(posedge clk) begin
	if(start) begin
		reorder_id_to_wb_pipe[0] <= reorder_id;
		rd_in_pipe[0] <= rd_in;
		regwrite_in_pipe[0] <= regwrite_in;
	end else begin
        reorder_id_to_wb_pipe[0] <= 'b0;
		rd_in_pipe[0] <= 'b0;
        regwrite_in_pipe[0] <= 'b0;
    end
	//For para duplicar HW
   for( i = 0; i < NUMBER_CYCLES-1; i++ ) begin
		reorder_id_to_wb_pipe[i+1] <= reorder_id_to_wb_pipe[i];
		rd_in_pipe[i+1] <= rd_in_pipe[i];
		regwrite_in_pipe[i+1] <= regwrite_in_pipe[i];
	end
end

  assign regwrite_out = regwrite_in_pipe[NUMBER_CYCLES-2];
  assign rd_out = rd_in_pipe[NUMBER_CYCLES-2];
  assign reorder_id_to_wb = reorder_id_to_wb_pipe[NUMBER_CYCLES-2];

endmodule

module alu_mult #(
					parameter  DATA_WIDTH = 32,
					parameter  NUMBER_CYCLES = 4
				)
				(
				a,b,busy,cycles_to_finish,out,done,start,clk,rst,
				reorder_id,reorder_id_to_wb,rd_in,rd_out,
				regwrite_in,regwrite_out
				);


input[3:0] reorder_id;
output reg [3:0] reorder_id_to_wb;	 
input [4:0] rd_in;
output reg[4:0] rd_out;
input regwrite_in;
output reg regwrite_out;


input [DATA_WIDTH-1:0] a,b;
input  start,clk,rst;
output reg [DATA_WIDTH-1:0]  out;
output reg [DATA_WIDTH-1:0] cycles_to_finish;
output reg busy,done;
reg [DATA_WIDTH-1:0] result,a_in,b_in;
reg start_mult;


assign busy = start_mult & !done;
assign result =  a_in * b_in;
//mult multiciclo fake
assign done = cycles_to_finish == 1;
assign out = done ? result : 'b0; // 'a_in * b_in' tem NUMBER_CYCLES para ficar pronto - backend pode ser ajustado


always @ (posedge clk) begin
    if(!rst) begin
    	a_in <= 'b0;
		b_in <= 'b0;
		start_mult <= 'b0;
    end else if(start) begin
		a_in <= a;
		b_in <= b;
		start_mult <= 1'b1;
	end else if (done) begin
		start_mult <= 1'b0;
	end else begin
		a_in <= a_in;
		b_in <= b_in;
		start_mult <= start_mult;
	end
    if (!rst) begin
      cycles_to_finish <= NUMBER_CYCLES - 1;
    end else if(start & !start_mult) begin
		cycles_to_finish <= NUMBER_CYCLES - 1;
    end else if(start_mult) begin
		cycles_to_finish <= cycles_to_finish - 1;
	end else begin
		cycles_to_finish <= cycles_to_finish;
	end
end


  reg[3:0] reorder_id_to_wb_pipe [NUMBER_CYCLES-2:0];
  reg[4:0] rd_in_pipe       [NUMBER_CYCLES-2:0];
  reg      regwrite_in_pipe [NUMBER_CYCLES-2:0];

//propagar dados
integer i;
always @(posedge clk) begin
	if(start) begin
		reorder_id_to_wb_pipe[0] <= reorder_id;
		rd_in_pipe[0] <= rd_in;
		regwrite_in_pipe[0] <= regwrite_in;
	end else begin
        reorder_id_to_wb_pipe[0] <= 'b0;
		rd_in_pipe[0] <= 'b0;
        regwrite_in_pipe[0] <= 'b0;
    end
	//For para duplicar HW
  for( i = 0; i < NUMBER_CYCLES-1; i++ ) begin
		reorder_id_to_wb_pipe[i+1] <= reorder_id_to_wb_pipe[i];
		rd_in_pipe[i+1] <= rd_in_pipe[i];
		regwrite_in_pipe[i+1] <= regwrite_in_pipe[i];
	end
end

  assign regwrite_out = regwrite_in_pipe[NUMBER_CYCLES-2];
  assign rd_out = rd_in_pipe[NUMBER_CYCLES-2];
  assign reorder_id_to_wb = reorder_id_to_wb_pipe[NUMBER_CYCLES-2];

endmodule

/////////////////////////////////////////////MEM - Instance//////////////////////////////////////
module memory #(parameter DEPTH = 128)
				(
				input clk,
				input w_en,
				input [31:0] w_data,
				input [31:0] w_addr,
				input [31:0] r_addr,
				output reg [31:0] r_data
				);

  integer i;
  reg [31:0] ram [0:127]; // CRIAR MEMORIA SEPARADO


  // fill the memory
  initial begin
    for (i = 0; i <= DEPTH-1; i++)
      ram[i] <= i;
  end
  
  assign r_data = ram[r_addr];	
  
  
  always @(posedge clk) begin
    if (w_en)
      ram[w_addr] <= w_data;
	else
	  ram[w_addr] <= ram[w_addr];  
  end

endmodule




//ROB
module reorder_buffer (
				input clk,
  				input stall,
				input rstn,
				input [7:0] new_instruction,
				input wb_finish,
  				input [3:0] wb_finish_reorder_id,
				input commit_rd,
				output reg empty, full, //flag de empty e de full
				
  				output reg [3:0] reorder_id_next,
  				output reg [9:0] rob_out
				);
	
	// {State(2bits),S,ST,V,R(5bits)} = 10
    reg [9:0] rob_mem [15:0]; //escolhendo 16 posições a fifo do ROB
	
	//ponteiros fifo
  	reg [3:0] wr_p, rd_p;
	
	assign reorder_id_next = wr_p;
//	assign empty = wr_p == rd_p;
	assign full = (rd_p - 1) == wr_p;
	
	//reading and commit logic
  assign rob_out = rob_mem[rd_p];
	always @ (posedge clk) begin
		if (!rstn) begin
			rd_p <= 0;
        end else if (commit_rd & (rd_p != wr_p)) begin
			rd_p <= rd_p + 1'b1;
		end else begin
			rd_p <= rd_p;
		end
	end
  always @ (posedge clk) begin
    if (new_instruction[7:5]!=2'b0) begin
    	empty <= 1'b0;
    end else if (commit_rd & (rd_p == wr_p)) begin
        empty <= 1'b1;
      rob_mem[rd_p] = 'h0;
    end else
      empty <= empty;
  end
	
	//updating finish flag
  always @(posedge clk) begin
    if (wb_finish) begin
      rob_mem[wb_finish_reorder_id] = rob_mem[wb_finish_reorder_id] | {2'b11,8'b0};
    end
  end
  
	
	//writting to buffer
	always @(posedge clk) begin
		if(!rstn) begin
			wr_p <= 0;
			rob_mem[rd_p] <= 'b0;
		end
		else begin
          if((new_instruction[7:5]!=2'b0) && !stall) begin
			case (new_instruction[7:5])
				// rob_mem = {State(2bits),S,ST,V,Reg(5bits)} 
                3'b001:  //ALU + MULT + DIV + LOAD
                  rob_mem[wr_p] <= {`PENDING,`NOT_SPECULATIVE,`NOT_INST_STORE,`VALID,new_instruction[4:0]};
				3'b010:  //STORE instruction
                  rob_mem[wr_p] <= {`PENDING,`NOT_SPECULATIVE,`INST_STORE,`VALID,5'h1F};
				3'b011:  //BRANCH instruction
                  rob_mem[wr_p] <= {`PENDING,`NOT_SPECULATIVE,`NOT_INST_STORE,`NOT_VALID,new_instruction[4:0]};
				3'b1xx:   //Normal instruction
                  rob_mem[wr_p] <= {`PENDING,`SPECULATIVE,`NOT_INST_STORE,`VALID,new_instruction[4:0]};
              	default:
                  rob_mem[wr_p] <= rob_mem[wr_p];
			endcase
			wr_p <= wr_p + 1'b1;
          end else begin
            wr_p <= wr_p;
            rob_mem[wr_p] <= rob_mem[wr_p]; 
          end
		end
	end
	

endmodule



//FSB
module finish_store_buffer (
							input rstn,
							input clk,
							input [31:0] data_in,
							input [31:0] addr_in,
							input we,
							input commit_rd,
							output reg error_fsb_buffer_full,
							output reg error_fsb_buffer_empty,
							output reg [31:0] data_to_mem,
							output reg [31:0] addr_to_mem
							);
	//fsb_buffer = {VALID, OP, ADDR, DATA}
	reg [65:0] fsb_buffer;
  assign {addr_to_mem, data_to_mem} = fsb_buffer[63:0];
  
	//reading and commit logic
	always @ (posedge clk) begin
		if (!rstn) begin
			fsb_buffer <= 66'h0;
			error_fsb_buffer_full  <= 1'b0;
			error_fsb_buffer_empty <= 1'b0;
		end else if (we) begin
			if(fsb_buffer[65]) begin
				error_fsb_buffer_full  <= 1'b1;
			end else begin
				fsb_buffer <= {1'b1,1'b1,addr_in,data_in};
			end
		end else if (commit_rd) begin
			if(!fsb_buffer[65]) begin
				error_fsb_buffer_empty  <= 1'b1;
			end else begin
                fsb_buffer[65] <= 1'b0;
			end
		end
	end
		
endmodule

//commit

module commit (
			input rstn,
			input clk,
			input [9:0] rob_data,
			output reg commiting,
			output reg copy_to_mem_en,
			output reg copy_prf_to_arf_en,
			output reg [4:0] copy_prf_to_arf_addr
			);
	
	// rob_data = {State(2bits),S,ST,V,R(5bits)}
	assign copy_prf_to_arf_addr = rob_data[4:0];
	reg store;
  assign store = rob_data[6];
  assign commiting = (rob_data[9:8] === 2'bXX) ? 'b0 : (rob_data[9:8] == 2'b11) ;
  always @ (*) begin
		if(rob_data[9:8] == 2'b11) begin // Finished
			if (!rob_data[6]) begin //NOT Store 
				copy_prf_to_arf_en = 1'b1;
				copy_to_mem_en = 1'b0;
			end else begin
				copy_prf_to_arf_en = 1'b0;
				copy_to_mem_en = 1'b1;
			end
		end else begin
			copy_prf_to_arf_en = 1'b0;
			copy_to_mem_en = 1'b0;
		end
	
	end
	
endmodule

//ACM ADICIONADO 
module Register_Bank_1port (input clk, input regwrite, input [4:0] read_reg, writereg, input [31:0] writedata, output [31:0] read_data);
  integer i;
  reg [31:0] memory [0:31]; // 32 registers de 32 bits cada

  // fill the memory
  initial begin
    for (i = 0; i <= 31; i++)
      memory[i] <= i;
  end

  assign read_data = (regwrite && read_reg==writereg) ? writedata : memory[read_reg];
  
  always @(posedge clk) begin
    if (regwrite)
      memory[writereg] <= writedata;
  end


endmodule



module issue_queue (
    input clk, resetn, write_enable, read_enable,
  	input [3:0] reorder_id,
    input [57:0] data_in,
  	output reg [61:0] data_out,
    output full, empty
);
  wire [61:0] data;
  reg error;
  reg [2:0] wptr; 
  reg [61:0] q_table [7:0]; //issue queue com 8 posições só
  reg [61:0] q_table_new [7:0]; 
  reg [3:0] next_intr_pointer;
  reg [7:0] instr_valid;
  assign full = (wptr == 3'b111);
  assign empty = (wptr == 3'b000);
  assign data_out = (wptr!=0) ? ((next_intr_pointer!=8) ? q_table[next_intr_pointer] : 'b0) : 'b0 ;
  assign data = {reorder_id,data_in};
  assign read = (next_intr_pointer !=8) ? read_enable : 1'b0;
  
  integer j;

  reg [8:0] sb_table_data_7;
  
  always @ (*) begin
    for ( j=0 ; j<7; j++) begin
      //instr_valid[i] = (!VSrc0 || !PSrc0) && (!VSrc1 || !PSrc1) 
      instr_valid[j] = ~(q_table[j][13] && q_table[j][12]) && ~(q_table[j][6] && q_table[j][5]) && (q_table[j][6] || q_table[j][13]);
    end
  end
  
  
  always @ (*) begin
    if (instr_valid[0] == 'b1)
      next_intr_pointer <= 0;
    else if (instr_valid[1] == 'b1)
      next_intr_pointer <= 1;
    else if (instr_valid[2] == 'b1)
      next_intr_pointer <= 2;
    else if (instr_valid[3] == 'b1)
      next_intr_pointer <= 3;
    else if (instr_valid[4] == 'b1)
      next_intr_pointer <= 4;
    else if (instr_valid[5] == 'b1)
      next_intr_pointer <= 5;
    else if (instr_valid[6] == 'b1)
      next_intr_pointer <= 6;
    else if (instr_valid[7] == 'b1)
    	next_intr_pointer <= 7;
    else
      	next_intr_pointer <= 8; //NOP -> se next_intr_pointer == 8 nao tem proxima instrução - enviar nop
  end


  integer i; 
  always @(posedge clk) begin
    for (i = 0; i<7; i++) begin
        q_table_new[i] = q_table[i];
      end
    
    if (!resetn) begin
        q_table[0] <= 0; q_table[1] <= 0; q_table[2] <= 0; q_table[3] <= 0;
        q_table[4] <= 0; q_table[5] <= 0; q_table[6] <= 0; q_table[7] <= 0;
        wptr <= 'b0;error = 1'b0;
     
    //read
    end else if(read && ~write_enable) begin     
      if (wptr > 0) begin
        for (i = 0; i<next_intr_pointer; i++) begin
           q_table[i] = q_table_new[i];
        end
        for (i = next_intr_pointer; i<wptr-1; i++) begin
            q_table[i] = q_table_new[i+1];
        end
        wptr = wptr-1; 
      end else begin
        q_table[0] = 'b0;
        wptr = 0; 
      end
       
    
      //write
    end else if (~read && write_enable) begin
      if (!full) begin
         q_table[wptr] = data;         
         wptr = wptr + 1;
      end else begin
         error = 1'b1;
      end
      
     //read and write
    end else if (read && write_enable) begin
      if (wptr > 0) begin
        for (i = 0; i<next_intr_pointer; i++) begin
           q_table[i] = q_table_new[i];
        end
        for (i = next_intr_pointer; i<wptr-1; i++) begin
            q_table[i] = q_table_new[i+1];
        end
        q_table[wptr-1] = data;
      	wptr = wptr;
      end else begin
        q_table[wptr] = data;
      end
            
      
    //!read !write  
    end else begin
      for (i = 0; i<7; i++) begin
         q_table[i] = q_table[i];
      end
      wptr = wptr;
    end
  end
 
 
endmodule
