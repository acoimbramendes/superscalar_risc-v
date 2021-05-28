
module alu_div #(
					parameter  DATA_WIDTH = 32,
					parameter  NUMBER_CYCLES = 12
				)
				(
				a,b,busy,cycles_to_finish,out,done,start;
				);

input [DATA_WIDTH-1:0] a,b;
input  done,start;
output [DATA_WIDTH-1:0] out;
output [DATA_WIDTH-1:0] cycles_to_finish;


assign busy = start_div & !done;

//divisao multiciclo fake
assign done = (cycles_to_finish == 0);
assign out = done ? a_in/b_in : 'b0; // 'a_in/b_in' tem NUMBER_CYCLES para ficar pronto - backend pode ser ajustado


always (posedge clk) begin
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
	if(start_div) begin
		cycles_to_finish <= cycles_to_finish - 1;
	end else begin
		cycles_to_finish <= cycles_to_finish;
	end
	
end

endmodule


/*
//divisor de clock - clock usado no caminho critico (divisao)
always (posedge clk) begin	
	if (count == NUMBER_CYCLES/2) begin
		clk_divided <= 0;
	end else if (count == NUMBER_CYCLES)begin
		clk_divided <= 1;
		count <= 0;
	end
	end else if (start || start_div) begin
		count += count + 1;
	end else begin
		clk_divided <= 0;
		count <= 0;
	end
end


//divisor de clock - clock usado no caminho critico (divisao)
always (posedge clk_divided) begin	
	if(start_div) begin
		out = a_in/b_in;
	end
		
end
*/