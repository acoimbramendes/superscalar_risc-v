module commit ();


	reg [31:0] arf [5:0];

	always @(posedge clk) begin
		else if (first_status == 2'b11) begin
			//executar o commit
			arf = tb.i4.decode.prf.mem[reg_addr];
		end
	end

endmodule

module reorder_buffer (
				input clk,
				input rstn,
				input new_instruction,
				input wb_finish,
				input wb_finish_addr,
				input commit_rd,
				output empty, full,
				
				output reg [1:0] first_status
				);
	
	// {State(2bits),S,ST,V,R(5bits)} = 10
	reg [9:0] rob_mem [15:0] //escolhendo 16 posições a fifo do ROB
	
	//ponteiros fifo
	reg [4:0] wr_p, rd_p;
	
	//flag de empty e de full
	reg empty, full;
	
	assign empty = wr_p == rd_p;
	assign full = (rd_p - 1) == wr_p;
	
	//reading and commit logic
	assign first_status = rob_mem[rd_p][15:14];
	always @ (posedge clk) begin
		if (!rstn) begin
			rd_p <= 0;
		end else if (commit_rd) begin
			rd_p <= rd_p + 1'b1;
		end else begin
			rd_p <= rd_p;
		end
	end
	
	//updating finish flag
	assign rob_mem[wb_finish_addr] = wb_finish ?  2'b11 : rob_mem[wb_finish_addr];
	
	//writting to buffer
	always @(posedge clk) begin
		if(!rstn) begin
			wr_p <= 0;
			
		end
		else if (instruction != 0) begin
			case (instruction[6:5]):
				// {State(2bits),S,ST,V,Reg(5bits)} 
				0: begin
					rob_mem[wr_p] = {`PENDING,`NOT_SPECULATIVE,`NOT_STORE,`VALID,instruction[4:0]}
				//...
				end
				
			
			endcase
			wr_p <= wr_p + 1'b1;
		end
	end
	

endmodule