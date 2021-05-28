//io2i

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
  wire [4:0] scoreboard_line;
  wire [7:0] scoreboard_data;
  wire scoreboard_write, scoreboard_read;
  wire exec_foward_A, exec_foward_B;
  wire [1:0] exec_foward_stage;

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
  reg [3:0] reorder_id_next,reorder_id_to_exe,reorder_id_to_wb,reorder_id_to_issue, wb_finish_reorder_id;

  //STALL LOGIC - Hazard de HW
  //TO-DO Check - Modularizar isso em uma unidade de controle
  
  assign clk_fetch    = (execute.alu_stall_req | execute.stall_wb_request) ? 1'b1 : clk;
  assign clk_decode   = (execute.alu_stall_req | execute.stall_wb_request) ? 1'b1 : clk;
  assign clk_issue    = (execute.alu_stall_req | execute.stall_wb_request) ? 1'b1 : clk;
  assign clk_exec = execute.stall_wb_request ?  1'b1 : clk;
  assign clk_exec_free = clk;
  assign clk_reg_bank = clk;

  //Instruction Memory
  //TO-DO instanciar aqui em vez de no estagio fetch

  scoreboard scoreboard (.clk(clk), .read(scoreboard_read), .sb_write(scoreboard_write), .sb_data(scoreboard_data), .sb_register(scoreboard_line));

  // FETCH STAGE - Adicionado sinais de controle
  fetch fetch (
			.zero(zero), .rst(rst), .clk(clk_fetch), .branch(branch), .sigext(sigext), .d_inst(d_inst), .d_pc(d_pc));

  // DECODE STAGE - Gerar sinais de controle
  decode decode (
    .inst(d_inst), .writedata(wb_writedata), .clk(clk_decode),.clk_register_bank(clk_reg_bank), .data1(data1), .data2(data2), .ImmGen(sigext), .alusrc(alusrc), 
    .memread(memread), .memwrite(memwrite), .memtoreg(memtoreg), .branch(branch), .aluop(aluop), .funct(funct), .regwrite_in(wb_regwrite), 
    .regwrite_decode_out(regwrite_decode_out), .rst(rst), .rs1_out(rs1_address), .rs2_out(rs2_address), .rd_out(d_address), .operation_type_out(op_type),
	
    .wb_regwrite_addr(wb_regwrite_addr)// ACM ADICIONADO
                   //ACM adicionado
               ,.data_to_rob(new_intr_to_rob)
               ,.reorder_id_next(reorder_id_next)
  			   ,.reorder_id(reorder_id_to_issue)
	);

  // ISSUE STAGE 
  issue issue (.clk(clk_issue), .rst(rst), .rd_in(d_address), .rs1_in(rs1_address), .rs2_in(rs2_address), .operation_type(op_type),
        .d_data1(data1), .d_data2(data2), .d_ImmGen(sigext),
        .d_alusrc(alusrc), .d_memread(memread), .d_memwrite(memwrite), .d_memtoreg(memtoreg), .d_branch(branch),
        .d_aluop(aluop), .d_funct(funct), .d_regwrite_decode_out(regwrite_decode_out),
        .sb_register(scoreboard_line),
        .sb_data(scoreboard_data),
        .sb_write(scoreboard_write),
        .foward_A(exec_foward_A), .foward_B(exec_foward_B), .foward_stage(exec_foward_stage),
        .rd_out(d_address_prop), .operation_type_out(op_type_prop),
        .e_data1(data1_prop), .e_data2(data2_prop), .e_ImmGen(sigext_prop),
	    .e_alusrc(alusrc_prop), .e_memread(memread_prop), .e_memwrite(memwrite_prop), .e_memtoreg(memtoreg_prop), .e_branch(branch_prop),
	    .e_aluop(aluop_prop), .e_funct(funct_prop), .e_regwrite_decode_out(regwrite_decode_out_prop)
               //ACM adicionado
               ,.reorder_id_in(reorder_id_to_issue)
               ,.reorder_id_out(reorder_id_to_exe)
		);

  // EXECUTE STAGE
  execute execute (
			//Sinais
			.clk(clk_exec),.in1(data1_prop), .in2(data2_prop), .ImmGen(sigext_prop), .alusrc(alusrc_prop), .aluop(aluop_prop), .funct(funct_prop), .zero(zero), .aluout(aluout), .rst(rst), .clk_free(clk_exec_free),
			//Propagando sinais de controle pelo pipeline
			.memtoreg_in(memtoreg_prop), .regwrite_in(regwrite_decode_out_prop), .memread_in(memread_prop), .memwrite_in(memwrite_prop),
			.memtoreg_out(alu_memtoreg_out), .regwrite_out(alu_regwrite_out), .memwrite_out(alu_memwrite_out),
			//Propagando dado
    
            //Novas entradas e saídas
    .foward_A(exec_foward_A), .foward_B(exec_foward_B), .foward_stage(exec_foward_stage), .operation_type(op_type_prop), .e_rd_in(d_address_prop),
    .e_rd_out(d_address_prop_two),			
			//ACM ADICIONADO 
    .memdata_out(readdata),
    .reorder_id_from_issue(reorder_id_to_exe),
    .reorder_id_to_wb(reorder_id_to_wb),
    .data_to_mem(store_data),
    .writeaddr_to_mem(store_addr)
    //.memread_out(alu_memread_out), 
    //.data2_out(alu_data2),
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
			
	reg wb_finish;
    assign wb_finish = alu_regwrite_out | alu_memwrite_out ; //adicionar store
	//////////////Commit/////////////////
  reorder_buffer rob (
    			.clk(clk),
				.rstn(rst),
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

  //Escreve na memoria
  assign execute.memory.w_data = data_to_mem_fsb; //Preciso ver como isso vai ficar
  assign execute.memory.w_en = copy_to_mem_en;
  assign execute.memory.w_addr = addr_to_mem_fsb;
  //Precisa de Encaminhamento. Caso store + load seguidos.
  
  reg[4:0] copy_prf_to_arf_addr;
  reg copy_prf_to_arf_en;
  commit commit (
    		.rstn(rst),
			.clk,
			.commiting(commiting),
			.rob_data(rob_out),
            .copy_to_mem_en(copy_to_mem_en),
			.copy_prf_to_arf_en(copy_prf_to_arf_en),
			.copy_prf_to_arf_addr(copy_prf_to_arf_addr)
			);
			
  reg[31:0] PRF_data;
  
  assign PRF_data = decode.PRF.memory[copy_prf_to_arf_addr]; //XMR
  assign commit_writedata = copy_prf_to_arf_en ? PRF_data : 'bZ; // Sinal para faciltar o debug apenas

  Register_Bank_1port ARF (  .clk(clk),
							  .regwrite(copy_prf_to_arf_en),
							  .writedata(PRF_data),
							  .writereg(copy_prf_to_arf_addr),
							  .read_data(), //TODO colocar no output
							  .read_reg()
	                 		  ); 
  
  
endmodule

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
      $display("PRF-R%d = %h",j, io2i.decode.PRF.memory[j]);
    end
    for(j=0;j<16;j++) begin
      $display("ARF-R%d = %h",j, io2i.ARF.memory[j]);
    end
    $display("MEM[10] = %h", io2i.execute.memory.ram[10]);
    $display("MEM[1] = %h", io2i.execute.memory.ram[1]);
    @(posedge clock);
    //$display("wb_writedata = %b \n", i4.wb_writedata);
    repeat (50) @(posedge clock);
    @(posedge clock)
    //$display("wb_writedata = %b \n", i4.wb_writedata);
    for(j=0;j<16;j++) begin
      $display("PRF-R%d = %h",j, io2i.decode.PRF.memory[j]);
    end
    for(j=0;j<16;j++) begin
      $display("ARF-R%d = %h",j, io2i.ARF.memory[j]);
    end
    $display("MEM[10] = %h", io2i.execute.memory.ram[10]);
    $display("MEM[1] = %h", io2i.execute.memory.ram[1]);
    
    for(j=0;j<16;j++) begin
      if(io2i.ARF.memory[j] != io2i.decode.PRF.memory[j]) 
        $display("ERROR: PRF != ARF Reg= %d", j);
    end
    $finish();
    
    
    //reset = 1;
    //$monitor("wb_writedata = %h \n", $time, i4.wb_writedata);
  end 
endmodule 

`define AND 10
`define OR  9
`define ADD 3
`define SUB 5
`define MULT 4 
`define DIV 8   
`define LOAD 12  
`define STORE 13  

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
    /*
    //test todas instruções
	inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h00518233;  // add x4 x3 x5
    inst_mem[3] <= 32'h027302b3; //MUL x5 x6 x7
    inst_mem[4] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[5] <= 32'h00000000; // nop
    inst_mem[6] <= 32'h00000000; // nop
    inst_mem[7] <= 32'h00000000; // nop
    inst_mem[8] <= 32'h00000000; // nop
    inst_mem[9] <= 32'h0090a023; //SW x9, 0(x1)
    inst_mem[10] <= 32'h00052183; //LW x3, 0(x10)
    inst_mem[5] <= 32'h00000000; // nop
    */
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
/*    
    //test hazard estrutural - DIV
    inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h00518233;  // add x4 x3 x5
    inst_mem[3] <= 32'h027302b3; //MUL x5 x6 x7
    inst_mem[4] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[5] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[6] <= 32'h00000000; // nop
*/    

    //test hazard estrutural - WB
    inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h0234c433; // DIV x8 x9 x3
    inst_mem[3] <= 32'h00518233;  // add x4 x3 x5
    inst_mem[4] <= 32'h027302b3; //MUL x5 x6 x7
    inst_mem[5] <= 32'h00518233;  // add x4 x3 x5
    inst_mem[6] <= 32'h00000000; // nop
    

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
				input [31:0] inst, writedata,
				input clk,clk_register_bank,rst,
				input regwrite_in,
				output reg [31:0] data1, data2, ImmGen,
				output reg alusrc, memread, memwrite, memtoreg, branch,
  				output reg [2:0] aluop,
				output reg [9:0] funct,
				output reg regwrite_decode_out,
                //AF
                output reg [4:0] rs1_out, rs2_out, rd_out,
                output reg [3:0] operation_type_out,
  
  				// ACM ADICIONADO
  				input[4:0] wb_regwrite_addr,
  				//ACM2 ADICIONADO
  				output reg [9:0] data_to_rob,
  				input  [3:0] reorder_id_next,
            	output reg [3:0] reorder_id
				
				);

  wire [4:0] rs1, rs2, rd;

  reg [31:0] data1_pipe, data2_pipe, ImmGen_pipe, writedata_pipe;
  reg [2:0] aluop_pipe;
  wire [6:0] opcode;
  reg [9:0] funct_pipe;
  reg alusrc_pipe, memtoreg_pipe, regwrite_pipe;
  reg memread_pipe, memwrite_pipe, branch_pipe;
  ///AF
  reg [3:0] aluctrl_pipe;

  assign opcode = inst[6:0];
  assign rs1    = inst[19:15];
  assign rs2    = inst[24:20];
  assign rd     = inst[11:7];
  assign funct_pipe = {inst[31:25],inst[14:12]};

  ControlUnit control (rst,opcode, inst, alusrc_pipe, memtoreg_pipe, regwrite_pipe, memread_pipe, memwrite_pipe, branch_pipe, aluop_pipe, ImmGen_pipe);
  
  
  Register_Bank PRF (clk_register_bank, regwrite_in, rs1, rs2, wb_regwrite_addr, writedata, data1_pipe, data2_pipe);
  //AF
  alucontrol alucontrol (aluop_pipe, funct_pipe, aluctrl_pipe);

  //PIPELINE DEC para EXE
  always @(posedge clk) begin
    if (!rst) begin
		data1 		<= 'b0;
		data2 		<= 'b0;
		ImmGen 		<= 'b0;
		alusrc  	<= 'b0;
		memread  	<= 'b0;
		memwrite  	<= 'b0;
		memtoreg  	<= 'b0;
		branch  	<= 'b0;
		aluop  		<= 'b0;
		funct 		<= funct_pipe;
		regwrite_decode_out	<= 'b0;
        ///AF
        rs1_out <= 'b0;
        rs2_out <= 'b0;
        rd_out  <= 'b0;
        operation_type_out <= 'b0;
    end
    else begin
		data1 		<= data1_pipe;
		data2 		<= data2_pipe;
		ImmGen 		<= ImmGen_pipe;
		alusrc  	<= alusrc_pipe;
		memread  	<= memread_pipe;
		memwrite  	<= memwrite_pipe;
		memtoreg  	<= memtoreg_pipe;
		branch  	<= branch_pipe;
		aluop  		<= aluop_pipe;
		funct 		<= funct_pipe;
		regwrite_decode_out	<= regwrite_pipe;
        ///AF
        rs1_out <= rs1;
        rs2_out <= rs2;
        rd_out  <= rd;
        operation_type_out <= aluctrl_pipe;

    end
  end
  
  
 //ACM2 ADICIONADO
 reg speculative;
 assign speculative = 1'b0; //No caso nao vai ter instrução especulativa, branch causa stalls, mas teria que alterar essa logica pra fazer - o ROB tem suporte pra especulativo 
 always @(posedge clk) begin
    reorder_id <= reorder_id_next;
 end
 always @ * begin
    case (aluctrl_pipe)
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
endmodule

module ControlUnit (
					input rst,
					input [6:0] opcode,
					input [31:0] inst,
					output reg alusrc, memtoreg, regwrite, memread, memwrite, branch,
  					output reg [2:0] aluop,
					output reg [31:0] ImmGen
					);

  always @(opcode) begin
      	alusrc   <= 0;
		memtoreg <= 0;
		regwrite <= 0;
		memread  <= 0;
		memwrite <= 0;
		branch   <= 0;
		aluop    <= 0;
		ImmGen   <= 0;
		case(opcode)
		  7'b0110011: begin // R type == 51
			regwrite <= 1;
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
			regwrite <= 1;
			ImmGen   <= {{20{inst[31]}},inst[31:20]};
		  end
				7'b0000011: begin // lw == 3
            aluop    <= 3;
			alusrc   <= 1;
			memtoreg <= 1;
			regwrite <= 1;
			memread  <= 1;
			ImmGen   <= {{20{inst[31]}},inst[31:20]};
		  end
				7'b0100011: begin // sw == 35
            aluop    <= 4;
			alusrc   <= 1;
			memwrite <= 1;
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
            //carried pipe signals
            input [4:0] rd_in,
            input [4:0] rs1_in, rs2_in,
            input [3:0] operation_type,
            input  [31:0] d_data1, d_data2, d_ImmGen,
            input  d_alusrc, d_memread, d_memwrite, d_memtoreg, d_branch,
  			input  [2:0] d_aluop,
            input  [9:0] d_funct,
            input  d_regwrite_decode_out,
            //end carried pipe signals
            //outputs para o scoreboard
            output logic [4:0] sb_register,
            output logic [7:0] sb_data,
            output reg sb_write,
            //propagando sinais pelo pipe
            output reg foward_A, foward_B,
            output reg [1:0] foward_stage,

            output reg [4:0] rd_out,
            output reg [3:0] operation_type_out,
            output reg [31:0] e_data1, e_data2, e_ImmGen,
	        output reg e_alusrc, e_memread, e_memwrite, e_memtoreg, e_branch,
  			output reg [2:0] e_aluop,
	        output reg [9:0] e_funct,
	        output reg e_regwrite_decode_out,
			
			//ACM2 ADICIONADO
  			input  [3:0] reorder_id_in,
  			output reg [3:0] reorder_id_out
);

wire sb_read;

reg foward_A_pipe, foward_B_pipe;
reg [1:0] foward_stage_pipe;
//scoreboard scoreboard (clk, sb_read, sb_write, sb_data, sb_register);

//assign rd_in = sb_register;
 assign sb_register = rd_in;

  //IQ mudar
  always @(posedge clk) begin
    reorder_id_out <= reorder_id_in;
  end

always @(operation_type) begin
    sb_write = 'b0;
    case (operation_type)
        4'd`ADD, 4'd`SUB, 4'd`AND, 4'd`OR: begin sb_data = 'b10010000; sb_write = 'b1; end       
        4'd2: begin sb_data = 'b10110000; sb_write = 1; end  //LOAD OU STORE
        4'd`MULT: begin sb_data = 'b11010000; sb_write = 1; end
        4'd`DIV: begin sb_data = 'b11110000; sb_write = 1; end
        4'd6: sb_write = 'b0;                                //BRANCH
        default : sb_write = 'b0;
    endcase
end  

always @(rs1_in, rs2_in) begin
    if (scoreboard.sb_table[rs1_in][7] == 1) begin
        case (scoreboard.sb_table[rs1_in][6:5])
            0: begin   //ADD SUB OR AND operation path
                foward_A_pipe = 'b1;
                foward_B_pipe = 'b0;
                foward_stage_pipe = 'b00;
            end
            1: begin  //LW, SW operation path
                foward_A_pipe = 'b1;
                foward_B_pipe = 'b0;
                foward_stage_pipe = 'b01;
            end
            2: begin  //MULT operation path
                foward_A_pipe = 'b1;
                foward_B_pipe = 'b0;
                foward_stage_pipe = 'b10;
            end
            3: begin  //DIV operation path
                foward_A_pipe = 'b1;
                foward_B_pipe = 'b0;
                foward_stage_pipe = 'b11;
            end
          	default: begin
              foward_A_pipe = 'b0;
              foward_B_pipe = 'b0;
              foward_stage_pipe = 'b00;
            end
        endcase
    end
	else begin
      foward_A_pipe = 'b0;
      foward_B_pipe = 'b0;
      foward_stage_pipe = 'b00;
    end

    if (scoreboard.sb_table[rs2_in][7] == 1) begin
        case (scoreboard.sb_table[rs2_in][6:5])
            0: begin   //ADD SUB OR AND operation path
                foward_A_pipe = 'b0;
                foward_B_pipe = 'b1;
                foward_stage_pipe = 'b00;
            end
            1: begin  //LW, SW operation path
                foward_A_pipe = 'b0;
                foward_B_pipe = 'b1;
                foward_stage_pipe = 'b01;
            end
            2: begin  //MULT operation path
                foward_A_pipe = 'b0;
                foward_B_pipe = 'b1;
                foward_stage_pipe = 'b10;
            end
            3: begin   //DIV operation path
                foward_A_pipe = 'b0;
                foward_B_pipe = 'b1;
                foward_stage_pipe = 'b11;
            end
          	default: begin
              foward_A_pipe = 'b0;
              foward_B_pipe = 'b0;
              foward_stage_pipe = 'b00;
            end
        endcase 
    end
  	else begin
      foward_A_pipe = 'b0;
      foward_B_pipe = 'b0;
      foward_stage_pipe = 'b00;
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
		e_aluop  		<= 'b0;
		e_funct 		<= 'b0;
		e_regwrite_decode_out	<= 'b0;
        operation_type_out      <= 'b0;
        foward_A        <= 'b0;
        foward_B        <= 'b0;
        foward_stage    <= 'b0;
        rd_out          <= 'b0;
    end
    else begin
		e_data1 		<= d_data1;
		e_data2 		<= d_data2;
		e_ImmGen 		<= d_ImmGen;
		e_alusrc  	    <= d_alusrc;
		e_memread  	    <= d_memread;
		e_memwrite  	<= d_memwrite;
		e_memtoreg  	<= d_memtoreg;
		e_branch  	    <= d_branch;
		e_aluop  		<= d_aluop;
		e_funct 		<= d_funct;
		e_regwrite_decode_out	<= d_regwrite_decode_out;
        operation_type_out      <= operation_type;
        foward_A        <= foward_A_pipe;
        foward_B        <= foward_B_pipe;
        foward_stage    <= foward_stage_pipe;
        rd_out          <= rd_in;
    end
end
endmodule

module scoreboard (
    input clk,
    input read,
    input sb_write,
    input logic [7:0] sb_data,
    input logic [4:0] sb_register
);
	
  	integer i;
    logic [7:0] sb_table [0:31];
  
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
        if(sb_table[i][7] == 1) begin
          sb_table[i][4:0] <= sb_table[i][4:0] >> 1;
        end
      end
    end
endmodule

///////////////////////////////////////////////EXECUTE///////////////////////////////////////
module execute # (			
					parameter PIPELINE_DEPTH = 6,
					parameter DIV_CYCLES = PIPELINE_DEPTH,
					parameter MULT_CYCLES = 4
				)(
				input [31:0] in1, in2,
                input [2:0] aluop, //dead signal
				input [9:0] funct,
				output reg zero, //TODO CHECK
				output reg [31:0] aluout,

				//control signals
				input [31:0] ImmGen,
				input alusrc,
				input rst, clk, clk_free,
                //AF
                input foward_A, foward_B,
                input [1:0] foward_stage,
                input [3:0] operation_type,

				//control signals no pipe
				input memtoreg_in, regwrite_in, memread_in, memwrite_in,
                input [4:0] e_rd_in, //AF
				output reg memtoreg_out, regwrite_out, memwrite_out,
                

                output reg [4:0] e_rd_out, //AF
//                output reg [31:0] data2_out,
				
				//ACM ADICIONADO
                output reg [31:0] memdata_out,
                input  [3:0]  reorder_id_from_issue,
                output reg [3:0]  reorder_id_to_wb,
                output reg [31:0] data_to_mem,
                output reg [31:0] writeaddr_to_mem
				);

  wire [31:0] alu_B;
  wire [31:0] aluout_pipe; 

  reg zero_pipe;
  
  reg [31:0] writedata_pipe;

  assign alu_B = (alusrc) ? ImmGen : in2 ;
  //TODO: COLOCAR AQUI A LOGICA DO FOWARD.
  
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
  
  //ALU PIPELINED
  alu_div #(.DATA_WIDTH(32), .NUMBER_CYCLES(DIV_CYCLES))
					alu_div (.a(in1),.b(alu_B),
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
					alu_mult (.a(in1),.b(alu_B),
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
							  .a(in1),.b(alu_B),
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
                        .data_in(in2),
                        .A(in1), .B(alu_B),
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
                        .A(in1), .B(alu_B),
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
  
  //HAZARD de HW logica (usar MULT or DIV antes de ter terminado)
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
  
  always @(posedge clk_free) begin
     // case (done)
        if(done[4]) begin // 5'b10000: begin
				reorder_id_to_wb <= reorder_id_to_wb_div; //ALL
				
				aluout       <= out_div;          //MULT + DIV + BASIC
				zero         <= 1'b0;             //BASIC
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
				zero         <= 1'b0;             //BASIC
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
				zero         <= 1'b0;             //BASIC
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
				zero         <= 1'b0;             //BASIC
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
				zero         <= zero_basic;        //BASIC
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
				zero         <= 'b0;             //BASIC
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

//stall hw -> request implementado - precisa adicionar na logica.

//ENCAMINHAMENTOS
//STALLS  - HAZARDS de Dados - ok (scoreboard), mas hazard de controle   (branch)/estrutural
//Qual complexo - harzards
//branch

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
	reg [4:0] wr_p, rd_p;
	
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
          if(new_instruction[7:5]!=2'b0) begin
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
  assign commiting = (rob_data[9:8] == 2'b11);
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
