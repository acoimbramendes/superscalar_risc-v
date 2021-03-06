module i4 (input clk, rst, output [31:0] wb_writedata);

  wire [31:0] sigext, data1, data2, aluout, readdata, inst, data1_prop, data2_prop, sigext_prop;
  wire zero, memread, memwrite, memtoreg, branch, alusrc, alusrc_prop, memread_prop, memwrite_prop, memtoreg_prop, branch_prop;
  wire [9:0] funct, funct_prop;
  wire [1:0] aluop, aluop_prop;
  wire [4:0] rs1_address, rs2_address, d_address, d_address_prop, d_address_prop_two, wb_regwrite_addr;
  wire [3:0] op_type, op_type_prop;
  wire [4:0] scoreboard_line;
  wire [7:0] scoreboard_data;
  wire scoreboard_write, scoreboard_read;
  wire exec_foward_A, exec_foward_B;
  wire [1:0] exec_foward_stage;

  //Adicionado
  wire [31:0] d_inst, d_pc;
  
  wire clk_fetch, clk_decode, clk_reg_bank, clk_issue;
  
  wire wb_regwrite, regwrite_decode_out, regwrite_decode_out_prop;
  wire alu_memtoreg_out, alu_regwrite_out,  alu_memread_out,  alu_memwrite_out;
  wire [31:0] alu_data2;
  wire mem_regwrite_out, mem_memtoreg_out;
  wire [31:0] mem_aluout;

  //STALL LOGIC - Hazard de HW
  //TO-DO Check - Modularizar isso em uma unidade de controle
  assign clk_fetch    = execute.alu.alu_stall_req ? 1'b1 : clk;
  assign clk_decode   = execute.alu.alu_stall_req ? 1'b1 : clk;
  assign clk_issue    = execute.alu.alu_stall_req ? 1'b1 : clk;
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
	    .e_aluop(aluop_prop), .e_funct(funct_prop), .e_regwrite_decode_out(regwrite_decode_out_prop));

  // EXECUTE STAGE
  execute execute (
			//Sinais
			.clk(clk),.in1(data1_prop), .in2(data2_prop), .ImmGen(sigext_prop), .alusrc(alusrc_prop), .aluop(aluop_prop), .funct(funct_prop), .zero(zero), .aluout(aluout), .rst(rst),
			//Propagando sinais de controle pelo pipeline
			.memtoreg_in(memtoreg_prop), .regwrite_in(regwrite_decode_out_prop), .memread_in(memread_prop), .memwrite_in(memwrite_prop),
			.memtoreg_out(alu_memtoreg_out), .regwrite_out(alu_regwrite_out), .memread_out(alu_memread_out), .memwrite_out(alu_memwrite_out),
			//Propagando dado
    .data2_out(alu_data2),
            //Novas entradas e sa??das
    .foward_A(exec_foward_A), .foward_B(exec_foward_B), .foward_stage(exec_foward_stage), .operation_type(op_type_prop), .e_rd_in(d_address_prop),
            .e_rd_out(d_address_prop_two)
			
			//ACM ADICIONADO 
			.memdata_out(readdata);
			);

  /*// MEMORY STAGE
  memory memory (
			//Sinais
			.address(aluout), .writedata(alu_data2), .memread(alu_memread_out), .memwrite(alu_memwrite_out), .clk(clk), .readdata(readdata), .rst(rst),
			//Propagando sinais de controle pelo pipeline
			.regwrite_in(alu_regwrite_out),
			.memtoreg_in(alu_memtoreg_out),
			.regwrite_out(mem_regwrite_out),
			.memtoreg_out(mem_memtoreg_out),
			//Propagando dado
			.mem_aluout_in(aluout),
			.mem_aluout(mem_aluout),
            .m_rd_in(d_address_prop_two),
            .m_rd_out(d_address_prop_three)
			);
	*/

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
			.reg_addr(wb_regwrite_addr)
			);
endmodule

module testbench();
  logic clock;
  logic reset;
  reg [31:0] i4_out;
  
  i4 i4 (.clk(clock), .rst(reset), .wb_writedata(i4_out));
  
  initial begin
    clock = 0;
    #10;
    forever begin 
      clock = ~clock; #10;
    end
  end
  
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
   	reset = 0;
    $monitor("wb_writedata = %h \n", $time, i4.wb_writedata);
    @(posedge clock);
    @(posedge clock);
    @(posedge clock);
    @(posedge clock);
    reset = 1;
    @(posedge clock);
    //$display("wb_writedata = %b \n", i4.wb_writedata);
    #300ns;
    @(posedge clock)
    //$display("wb_writedata = %b \n", i4.wb_writedata);
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

///////////FETCH//////////////////////////////////////////////////

module fetch (input zero, rst, clk, branch, input [31:0] sigext, output reg [31:0] d_inst,d_pc);
  wire [31:0] pc, pc_4, new_pc;
  wire [31:0] inst;
  assign pc_4 = 4 + pc; // pc+4  Adder
  assign new_pc = (branch & zero) ? pc_4 + sigext : pc_4; // new PC Mux

  reg [31:0] inst_mem;
  assign inst = inst_mem;

  //M??dulo de PC
  PC program_counter(new_pc, clk, rst, pc);
  //Criando um modulo s?? para a memoria de instru????o
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
	inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok
    inst_mem[2] <= 32'h00210233; // add  x4, x2, x2  ok
	//inst_mem[3] <= 32'h027302b3; 
	//inst_mem[4] <= 32'h00170793; 
	//inst_mem[5] <= 32'h028284b3; 
	//inst_mem[6] <= 32'h02a483b3; 
	//inst_mem[7] <= 32'h00158613; 
	//inst_mem[8] <= 32'h00160693; 
	//inst_mem[9] <= 32'h00260713; 
	
  end
endmodule

/////////////////////////////////////////DECODE//////////////////////////////////////////////////

module decode (
				input [31:0] inst, writedata,
				input clk,clk_register_bank,rst,
				input regwrite_in,
				output reg [31:0] data1, data2, ImmGen,
				output reg alusrc, memread, memwrite, memtoreg, branch,
				output reg [1:0] aluop,
				output reg [9:0] funct,
				output reg regwrite_decode_out,
                //AF
                output reg [4:0] rs1_out, rs2_out, rd_out,
                output reg [3:0] operation_type_out
				);

  wire [4:0] rs1, rs2, rd;

  reg [31:0] data1_pipe, data2_pipe, ImmGen_pipe, writedata_pipe;
  reg [1:0] aluop_pipe;
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
  Register_Bank Registers (clk_register_bank, regwrite_in, rs1, rs2, rd, writedata, data1_pipe, data2_pipe);
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
		//TODO- Checar - Por mim precisa de flop aqui, mas pelo exemplo do mips nao.
		//regwrite_in_pipe		<= regwrite_in
		//writedata_pipe	<= writedata;
    end
  end
endmodule

module ControlUnit (
					input rst,
					input [6:0] opcode,
					input [31:0] inst,
					output reg alusrc, memtoreg, regwrite, memread, memwrite, branch,
					output reg [1:0] aluop,
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
			alusrc   <= 1;
			memtoreg <= 1;
			regwrite <= 1;
			memread  <= 1;
			ImmGen   <= {{20{inst[31]}},inst[31:20]};
		  end
				7'b0100011: begin // sw == 35
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

module alucontrol (input [1:0] aluop, input [9:0] funct, output reg [3:0] alucontrol);
  wire [7:0] funct7;
  wire [2:0] funct3;

  assign funct3 = funct[2:0];
  assign funct7 = funct[9:3];

  always @(aluop) begin
    case (aluop)
      0: alucontrol <= 4'd2; // ADD to SW and LW
      1: alucontrol <= 4'd6; // SUB to branch
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
          //39: alucontrol <= 4'd12; // NOR
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
            input reg [31:0] d_data1, d_data2, d_ImmGen,
            input reg d_alusrc, d_memread, d_memwrite, d_memtoreg, d_branch,
            input reg [1:0] d_aluop,
            input reg [9:0] d_funct,
            input reg d_regwrite_decode_out,
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
	        output reg [1:0] e_aluop,
	        output reg [9:0] e_funct,
	        output reg e_regwrite_decode_out
);

wire sb_read;

reg foward_A_pipe, foward_B_pipe;
reg [1:0] foward_stage_pipe;
//scoreboard scoreboard (clk, sb_read, sb_write, sb_data, sb_register);

//assign rd_in = sb_register;
 assign sb_register = rd_in;

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
module execute (
				input [31:0] in1, in2,
				input [1:0] aluop, //dead signal
				input [9:0] funct,
				output reg zero, //TODO CHECK
				output reg [31:0] aluout,

				//control signals
				input [31:0] ImmGen,
				input alusrc,
				input rst, clk,
                //AF
                input foward_A, foward_B,
                input [1:0] foward_stage,
                input [3:0] operation_type,

				//control signals no pipe
				input memtoreg_in, regwrite_in, memread_in, memwrite_in,
                input [4:0] e_rd_in, //AF
				output reg memtoreg_out, regwrite_out, memread_out, memwrite_out,
                

                output reg [4:0] e_rd_out, //AF
                output reg [31:0] data2_out,
				
				//ACM ADICIONADO
				output [31:0] memdata_out
				);

  wire [31:0] alu_B;
  wire [31:0] aluout_pipe;
  //wire [3:0] aluctrl;

  reg zero_pipe;

  reg [31:0] writedata_pipe;
  //reg [4:0] e_rd_out_pipe;

  assign alu_B = (alusrc) ? ImmGen : in2 ;
  //TODO: COLOCAR AQUI A LOGICA DO FOWARD.
  
  //ACM Adicionado
  wire [31:0] data_to_mem;
  wire [31:0] writeaddr_to_mem;
  wire memwrite_en;
  
  wire [31:0] data_from_mem;
  wire [31:0] memdata;
  wire [31:0] readaddr_to_mem;

  //Unidade L??gico Aritim??tica
  ALU alu (.alucontrol(operation_type), .A(in1),.B(alu_B),.clk(clk), .aluout(aluout_pipe), .zero(zero_pipe));
  
  store_unit store_unit(.clk(clk),
						.rst(rst),
						.memwrite(memwrite_in) //store control signal
						.A(in1),B(alu_B),
						.memwrite_en(memwrite_en)
						.data_to_mem(data_to_mem),
						.writeaddr(writeaddr_to_mem),
						);
  
  load_unit  load_unit (.clk(clk),
						.(rst),
						.memread(memread_in) //load control signal
						.data_in(in2),
						.A(in1),B(alu_B),
						.data_from_mem(data_from_mem),
						.data_out(memdata),
						.readaddr(readaddr_to_mem),
						);
						
  memory #(.DEPTH(128)) memory (
							.clk(clk),
							.w_en(memwrite_en),
							.w_data(data_to_mem),
							.w_addr(writeaddr_to_mem),
							.r_addr(readaddr_to_mem),
							.r_data(data_from_mem)
					)
  

  //PIPELINE EXE TO MEM
  //Completando o pipeline para ser in-order
  integer i;
  reg memwrite_pipe [PIPELINE_DEPTH-1:0];
  reg memread_pipe  [PIPELINE_DEPTH-1:0];
  reg [31:0] memdata_out_pipe [PIPELINE_DEPTH-3:0];
  reg regwrite_out_pipe [PIPELINE_DEPTH-1:0];
  reg e_rd_out_pipe [PIPELINE_DEPTH-1:0];

  
  always @ (posedge clk) begin
	memwrite_pipe[0] <= memwrite;
	memread_pipe[0]  <= memread;
	memdata_out_pipe[0] <= memdata;
	regwrite_out_pipe[0] <= regwrite_in;
	e_rd_out_pipe[0] <= e_rd_in;
	for( i = 0; i < PIPELINE_DEPTH; i++ ) begin
		memwrite_pipe[i+1] <= memwrite_pipe[i];
		memread_pipe[i+1]  <= memread_pipe[i];
		regwrite_out_pipe[i+1] <= regwrite_out_pipe[i];
		e_rd_out_pipe[i+1] <= e_rd_out_pipe[i];
	end
	for( i = 0; i < PIPELINE_DEPTH-2; i++ ) begin
		memdata_out_pipe[i+1]   <= memdata_out_pipe[i];
	end
	
  end
	
  assign memtoreg_out = memread_pipe[PIPELINE_DEPTH-1];
  assign memwrite_out = memwrite_pipe[PIPELINE_DEPTH-1]; 
  assign memdata_out = memdata_out_pipe[PIPELINE_DEPTH-1];
  assign regwrite_out = regwrite_out_pipe[PIPELINE_DEPTH-1];
  assign e_rd_out = e_rd_out_pipe[PIPELINE_DEPTH-1];
  
  always @(posedge clk) begin
    if (!rst) begin
      zero    <= 0;
      aluout  <= 0;
    end
    else begin
      zero    <= zero_pipe;
      aluout  <= aluout_pipe;
    end
  end

endmodule



//ACM ADICIONADO
//1 coclo: 1 para calculo do endere??o, no proximo o sinal j?? ?? despachado
module store_unit (
				input clk,
				input memwrite,
				input [31:0] data_in,
				input [31:0] A,B,
				output memwrite_en,
				output [31:0] data_to_mem,
				output [31:0] writeaddr,
			) 
			
	reg memwrite;
	assign data_in = data_to_mem;
	always @(posedge clk) begin
		address <= A + B;
		memwrite_en <=memwrite;
	end
endmodule

//2 ciclos: 1 para calculo do endere??o e outro para receber o data
module load_unit (
				input clk,
				input memread,
				input [31:0] A,B,
				input [31:0] data_from_mem,
				output [31:0] data_out,
				output [31:0] readaddr,
			) 
	reg memread_pipe;
	assign data_out = data_from_mem;
	always @(posedge clk) begin
		readaddr <= A + B;
		memread_pipe <= memread;
	end
endmodule



module ALU #(
			parameter DIV_CYCLES = 12,
			parameter MULT_CYCLES = 4,
			parameter SUM_CYCLES = 1,
			parameter PIPELINE_DEPTH = DIV_CYCLES
			)
		    (
			input [3:0] alucontrol,
			input [31:0] A, B,
			input clk,
			output reg [31:0] aluout,
			output reg zero
			);

  reg alu_stall_req;
  reg start_mult;
  reg start_div;
  reg alu_control_pipe[PIPELINE_DEPTH-1:0];
  reg alu_div_pipe[PIPELINE_DEPTH-1:0];
  reg alu_sum_pipe[PIPELINE_DEPTH-1:0];
  reg alu_mult_pipe[PIPELINE_DEPTH-1:0];
  reg busy_sum, busy_mult, busy_div;
  reg done_sum, done_mult, done_div;
  reg [31:0] cycles_to_finish_sum, cycles_to_finish_mult, cycles_to_finish_div;
  reg [31:0] out_sum, out_mult, out_div;

  assign zero = (aluout == 0); // Zero recebe um valor l??gico caso aluout seja igual a zero.

  //INPUT START LOGIC
  always @(*) begin
      case (alu_control_pipe[0])
		`MULT: begin
			start_mult <= 1'b1;
			start_div  <= 1'b0;
		end
		`DIV:  begin
			start_mult <= 1'b0;
			start_div  <= 1'b1;
		end
	    default: begin
			start_mult <= 1'b0;
			start_div  <= 1'b0;
		end
    endcase
  end

  //HAZARD de HW logica (usar MULT or DIV antes de ter terminado)
  always @(posedge clk) begin
	case (alu_control_pipe[0])
		`MULT: begin
			if(busy_mult)
				alu_stall_req <= 1'b1; //hw de mult sendo utilizado, gera stall de hw
		end
		`DIV: begin
			if(busy_div)
				alu_stall_req <= 1'b1; //hw de mult sendo utilizado, gera stall de hw
		end
	    default: alu_stall_req <= 1'b0; //default 0, Nada acontece;
	endcase
  end

  //ALU PIPELINED
  alu_div #(.DATA_WIDTH(32), .NUMBER_CYCLES(DIV_CYCLES))
		alu_div (.a(A),.b(B),.busy(busy_div),.cycles_to_finish(cycles_to_finish_div),.out(out_div),.done(done_div),.clk(clk),.start(start_div));
  alu_mult #(.DATA_WIDTH(32), .NUMBER_CYCLES(MULT_CYCLES))
		alu_mult (.a(A),.b(B),.busy(busy_mult),.cycles_to_finish(cycles_to_finish_mult),.out(out_mult),.done(done_mult),.clk(clk),.start(start_mult));
  alu_sum  #(.DATA_WIDTH(32))
		alu_sum(.alucontrol(alucontrol),.a(A),.b(B),.out(out_sum));
/*
  genvar i;
  generate
	//For para duplicar HW
	for( i = 0; i < PIPELINE_DEPTH; i++ ) begin
      always @ (posedge clk) begin
		alu_control_pipe[i+1] = alu_control_pipe[i];
		alu_sum_pipe[i+1] = alu_sum_pipe[i];
        end
     end
  endgenerate
  generate
    always @ (posedge clk) begin
        for( i = DIV_CYCLES-1; i < PIPELINE_DEPTH; i++ ) begin
            alu_div_pipe[i+1] = alu_div_pipe[i];
        end
	end
  endgenerate

  generate
    always @ (posedge clk) begin
        for( i = MULT_CYCLES-1; i < PIPELINE_DEPTH; i++ ) begin
            alu_mult_pipe[i+1] = alu_mult_pipe[i];
        end
    end
  endgenerate
  */
  //Completando o pipeline para ser in-order
  integer i;
  always @ (posedge clk) begin
    alu_control_pipe[0] <= alucontrol;
    alu_div_pipe[DIV_CYCLES-1] <= out_div;
    alu_sum_pipe[0] <= out_sum;
    alu_mult_pipe[MULT_CYCLES-1] <= out_mult;
	//For para duplicar HW
	for( i = 0; i < PIPELINE_DEPTH; i++ ) begin
		alu_control_pipe[i+1] <= alu_control_pipe[i];
		alu_sum_pipe[i+1] <= alu_sum_pipe[i];
	end

	for( i = DIV_CYCLES-1; i < PIPELINE_DEPTH; i++ ) begin
		alu_div_pipe[i+1] <= alu_div_pipe[i];
	end

	for( i = MULT_CYCLES-1; i < PIPELINE_DEPTH; i++ ) begin
		alu_mult_pipe[i+1] <= alu_mult_pipe[i];
	end

  end
    // mux output
  always @(*) begin
      case (alu_control_pipe[PIPELINE_DEPTH-1])
        `AND: aluout <= alu_sum_pipe[PIPELINE_DEPTH-1]; // AND
        `OR:  aluout <= alu_sum_pipe[PIPELINE_DEPTH-1]; // OR
        `ADD: aluout <= alu_sum_pipe[PIPELINE_DEPTH-1]; // ADD
        `SUB: aluout <= alu_sum_pipe[PIPELINE_DEPTH-1]; // SUB
		`MULT: aluout <= alu_mult_pipe[PIPELINE_DEPTH-1]; // MULT
		`DIV:  aluout <= alu_div_pipe[PIPELINE_DEPTH-1];

		//TODO
        //7: aluout <= A < B ? 32'd1:32'd0; //SLT
        //12: aluout <= ~(A | B); // NOR

	  default: aluout <= 0; //default 0, Nada acontece;
    endcase
  end
endmodule

/////////////////////////////////////////////MEM//////////////////////////////////////
module memory (
				input [31:0] address, writedata,
				input memread, memwrite, clk, rst,
				output reg [31:0] readdata,

				//control signals no pipe
        input [4:0] m_rd_in,
				input  regwrite_in,
				output reg regwrite_out,
				input  memtoreg_in,
				output reg memtoreg_out,

                input [31:0] mem_aluout_in,
				output reg [31:0] mem_aluout,
        output reg [4:0] m_rd_out
				);

  integer i;
  reg [31:0] memory [0:127];

  reg [31:0] readdata_pipe;

  // fill the memory
  initial begin
    for (i = 0; i <= 127; i++)
      memory[i] <= i;
  end

  assign readdata_pipe = (memread) ? memory[address[31:2]] : 0;

  always @(posedge clk) begin
    if (memwrite)
      memory[address[31:2]] <= writedata;
	end

  //PIPELINE MEM para WB
  always @(posedge clk) begin
    if (!rst) begin
      readdata  <= 0;
    end
    else begin
      readdata <= readdata_pipe;
    end
    mem_aluout <= mem_aluout_in;
	  memtoreg_out <= memtoreg_in;
	  regwrite_out <= regwrite_in;
      m_rd_out <= m_rd_in;
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
                  output reg [4:0] reg_addr
				  );


  assign regwrite = rst ? 'b0 : regwrite_in;

  always @(memtoreg) begin
	write_data <= (memtoreg) ? readdata : aluout;
  end

 //ACM ADICIONADO
  assign reg_addr = w_rd_in;

endmodule

//stall hw -> request implementado - precisa adicionar na logica.

//ENCAMINHAMENTOS
//STALLS  - HAZARDS de Dados - ok (scoreboard), mas hazard de controle   (branch)/estrutural
//Qual complexo - harzards
//branch

/////////////////////////////////////ALU///////////////////////////////////

module alu_sum #(
					parameter  DATA_WIDTH = 32
				)
				(
				a,b,out,alucontrol
				);

input [3:0] alucontrol;
input [DATA_WIDTH-1:0] a,b;
output reg [DATA_WIDTH-1:0] out;


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

endmodule

module alu_div #(
					parameter  DATA_WIDTH = 32,
					parameter  NUMBER_CYCLES = 12
				)
				(
				a,b,busy,cycles_to_finish,out,done,start,clk
				);

input [DATA_WIDTH-1:0] a,b;
reg [DATA_WIDTH-1:0] a_in,b_in;
input  start,clk;
reg start_div;
output reg [DATA_WIDTH-1:0] out;
output reg [DATA_WIDTH-1:0] cycles_to_finish;
output reg busy,done;

assign busy = start_div & !done;

//divisao multiciclo fake
assign done = (cycles_to_finish == 0);
assign out = done ? a_in/b_in : 'b0; // 'a_in/b_in' tem NUMBER_CYCLES para ficar pronto - backend pode ser ajustado


always @ (posedge clk) begin
	if(start) begin
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

	if(start & !start_div) begin
		cycles_to_finish <= NUMBER_CYCLES - 1;
    end else if(start_div) begin
		cycles_to_finish <= cycles_to_finish - 1;
	end else begin
		cycles_to_finish <= cycles_to_finish;
	end

end
endmodule

module alu_mult #(
					parameter  DATA_WIDTH = 32,
					parameter  NUMBER_CYCLES = 4
				)
				(
				a,b,busy,cycles_to_finish,out,done,start,clk
				);

input [DATA_WIDTH-1:0] a,b;
input  start,clk;
output reg [DATA_WIDTH-1:0] out;
output reg [DATA_WIDTH-1:0] cycles_to_finish;
output reg busy,done;
reg [DATA_WIDTH-1:0] a_in,b_in;
reg start_mult;


assign busy = start_mult & !done;

//mult multiciclo fake
assign done = (cycles_to_finish == 0);
assign out = done ? a_in * b_in : 'b0; // 'a_in * b_in' tem NUMBER_CYCLES para ficar pronto - backend pode ser ajustado


always @ (posedge clk) begin
	if(start) begin
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

	if(start & !start_mult) begin
		cycles_to_finish <= NUMBER_CYCLES - 1;
  end else if(start_mult) begin
		cycles_to_finish <= cycles_to_finish - 1;
	end else begin
		cycles_to_finish <= cycles_to_finish;
	end

end
endmodule