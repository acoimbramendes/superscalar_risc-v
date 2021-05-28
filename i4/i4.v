// TOP -------------------------------------------
module i4 (input clk, rst, output [31:0] wb_writedata);

  wire [31:0] sigext, data1, data2, aluout, readdata, inst;
  wire zero, memread, memwrite, memtoreg, branch, alusrc;
  wire [9:0] funct;
  wire [1:0] aluop;

  //Adicionado
  wire [31:0] d_inst, d_pc;
  
  wire clk_fetch, clk_decode, clk_reg_bank;
  
  wire wb_regwrite,regwrite_decode_out;
  wire alu_memtoreg_out, alu_regwrite_out,  alu_memread_out,  alu_memwrite_out;
  wire [31:0] alu_data2;
  wire mem_regwrite_out, mem_memtoreg_out;
  wire [31:0] mem_aluout;

  //STALL LOGIC - Hazard de HW
  //TO-DO Check - Modularizar isso em uma unidade de controle
  assign clk_fetch    = execute.alu.alu_stall_req ? 1'b1 : clk;
  assign clk_decode   = execute.alu.alu_stall_req ? 1'b1 : clk;
  assign clk_reg_bank = clk;

  //Instruction Memory
  //TO-DO instanciar aqui em vez de no estagio fetch

  // FETCH STAGE - Adicionado sinais de controle
  fetch fetch (
			.zero(zero), .rst(rst), .clk(clk_fetch), .branch(branch), .sigext(sigext), .d_inst(d_inst), .d_pc(d_pc));

  // DECODE STAGE - Gerar sinais de controle
  decode decode (
    .inst(inst), .writedata(wb_writedata), .clk(clk_decode),.clk_register_bank(clk_reg_bank), .data1(data1), .data2(data2), .ImmGen(sigext), .alusrc(alusrc), .memread(memread), .memwrite(memwrite), .memtoreg(memtoreg), .branch(branch), .aluop(aluop), .funct(funct), .regwrite_in(wb_regwrite), .regwrite_decode_out(regwrite_decode_out), .rst(rst));

  // EXECUTE STAGE
  execute execute (
			//Sinais
			.clk(clk),.in1(data1), .in2(data2), .ImmGen(sigext), .alusrc(alusrc), .aluop(aluop), .funct(funct), .zero(zero), .aluout(aluout), .rst(rst),
			//Propagando sinais de controle pelo pipeline
			.memtoreg_in(memtoreg), .regwrite_in(regwrite_decode_out), .memread_in(memread), .memwrite_in(memwrite),
			.memtoreg_out(alu_memtoreg_out), .regwrite_out(alu_regwrite_out), .memread_out(alu_memread_out), .memwrite_out(alu_memwrite_out),
			//Propagando dado
			.data2_out(alu_data2)
			);


  // MEMORY STAGE
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
			.mem_aluout(mem_aluout)
			);


  // WRITEBACK STAGE
  writeback writeback (
			.rst(rst), .clk(clk),.aluout(mem_aluout),
			.readdata(readdata),
			.memtoreg(mem_memtoreg_out),
			.regwrite_in(mem_regwrite_out),
			.regwrite(wb_regwrite),
			.write_data(wb_writedata)
			);

endmodule

`define AND 0
`define OR  1
`define ADD 2
`define SUB 6
`define MULT 4 //TODO
`define DIV 4  //TODO
///////////FETCH//////////////////////////////////////////////////
module fetch (input zero, rst, clk, branch, input [31:0] sigext, output reg [31:0] d_inst,d_pc);

  wire [31:0] pc, pc_4, new_pc;
  wire [31:0] inst;
  assign pc_4 = 4 + pc; // pc+4  Adder
  assign new_pc = (branch & zero) ? pc_4 + sigext : pc_4; // new PC Mux

  //PC program_counter(new_pc, clk, rst, pc);

  reg [31:0] inst_mem;

  assign inst = inst_mem;

  //Criando um modulo só para a memoria de instrução
  InstructionMem instruction_memory (
    .rst(rst),
    .addr({2'b00,pc[31:2]}), //TODO CHECAR
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
	  d_pc     <= pc;
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
  input [31:0] addr,
  output [31:0] inst_out
);

  reg [31:0] inst_mem [0:31];

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
	inst_mem[3] <= 32'h027302b3; 
	inst_mem[4] <= 32'h00170793; 
	inst_mem[5] <= 32'h028284b3; 
	inst_mem[6] <= 32'h02a483b3; 
	inst_mem[7] <= 32'h00158613; 
	inst_mem[8] <= 32'h00160693; 
	inst_mem[9] <= 32'h00260713; 
	
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
				output reg regwrite_decode_out
				);

  wire [4:0] rs1, rs2, rd;

  reg [31:0] data1_pipe, data2_pipe, ImmGen_pipe, writedata_pipe;
  reg [1:0] aluop_pipe;
  wire [6:0] opcode;
  reg [9:0] funct_pipe;
  reg alusrc_pipe, memtoreg_pipe, regwrite_pipe;
  reg memread_pipe, memwrite_pipe, branch_pipe;

  assign opcode = inst[6:0];
  assign rs1    = inst[19:15];
  assign rs2    = inst[24:20];
  assign rd     = inst[11:7];
  assign funct_pipe = {inst[31:25],inst[14:12]};

  ControlUnit control (rst,opcode, inst, alusrc_pipe, memtoreg_pipe, regwrite_pipe, memread_pipe, memwrite_pipe, branch_pipe, aluop_pipe, ImmGen_pipe);

  Register_Bank Registers (clk_register_bank, regwrite_in, rs1, rs2, rd, writedata, data1_pipe, data2_pipe);

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

		alusrc 		<= 'b0;
		memtoreg	<= 'b0;
		regwrite_decode_out	<= 'b0;
		memread 	<= 'b0;
		memwrite 	<= 'b0;
		branch 		<= 'b0;

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
		memread 	<= memread_pipe;
		memwrite 	<= memwrite_pipe;
		branch 		<= branch_pipe;

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
	if(rst) begin
		alusrc   <= 0;
		memtoreg <= 0;
		regwrite <= 0;
		memread  <= 0;
		memwrite <= 0;
		branch   <= 0;
		aluop    <= 0;
		ImmGen   <= 0;
	end else begin
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

///////////////////////////////////////////////EXECUTE///////////////////////////////////////
module execute (
				input [31:0] in1, in2,
				input [1:0] aluop,
				input [9:0] funct,
				output reg zero, //TODO CHECK
				output reg [31:0] aluout,

				//control signals
				input [31:0] ImmGen,
				input alusrc,
				input rst, clk,

				//control signals no pipe
				input  memtoreg_in, regwrite_in, memread_in, memwrite_in,
				output reg memtoreg_out, regwrite_out, memread_out, memwrite_out,

                output reg [31:0] data2_out
				);

  wire [31:0] alu_B;
  wire [31:0] aluout_pipe;
  wire [3:0] aluctrl;

  reg zero_pipe;

  reg [31:0] writedata_pipe;

  assign alu_B = (alusrc) ? ImmGen : in2 ;

  //Unidade Lógico Aritimética
  ALU alu (.alucontrol(aluctrl), .A(in1),.B(alu_B),.clk(clk), .aluout(aluout_pipe), .zero(zero_pipe));
			

  //PIPELINE EXE TO MEM
  always @(posedge clk) begin
    if (!rst) begin
      zero    <= 0;
      aluout  <= 0;
    end
    else begin
      zero    <= zero_pipe;
      aluout  <= aluout_pipe;
    end
	//
    data2_out <= in2;
    memtoreg_out <= memtoreg_in;
    regwrite_out <= regwrite_in;
    memread_out <= memread_in;
    memwrite_out <= memwrite_in;

//	writedata	<= writedata_in;
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
          0: alucontrol <= (funct7 == 0) ? /*ADD*/ 4'd2 : /*SUB*/ 4'd6;
          2: alucontrol <= 4'd7; // SLT
          6: alucontrol <= 4'd1; // OR
          //39: alucontrol <= 4'd12; // NOR
          7: alucontrol <= 4'd0; // AND
          default: alucontrol <= 4'd15; // Nop
        endcase
      end
    endcase
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



  assign zero = (aluout == 0); // Zero recebe um valor lógico caso aluout seja igual a zero.

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
				input  regwrite_in,
				output reg regwrite_out,
				input  memtoreg_in,
				output reg memtoreg_out,

                input [31:0] mem_aluout_in,
				output reg [31:0] mem_aluout
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


  end

endmodule


/////////////////////////////////////////////WB//////////////////////////////////////
module writeback (
				  input rst, clk,
				  input [31:0] aluout, readdata,
				  input memtoreg,
				  input  regwrite_in,
				  output reg [31:0] write_data,
				  output reg regwrite
				  );


  assign regwrite = rst ? 'b0 : regwrite_in;

  always @(memtoreg) begin
	write_data <= (memtoreg) ? readdata : aluout;
  end

/*
  always @(clk) begin
	if (rst) begin
		write_data <= 'b0;
	end else begin
		write_data <= (memtoreg) ? readdata : aluout;
	end
  end
 */

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
