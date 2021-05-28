
module alu_mult #(
					parameter  DATA_WIDTH = 32,
					parameter  NUMBER_CYCLES = 4
				)
				(
				a,b,busy,cycles_to_finish,out,done,start;
				);

input [DATA_WIDTH-1:0] a,b;
input  done,start;
output [DATA_WIDTH-1:0] out;
output [DATA_WIDTH-1:0] cycles_to_finish;


assign busy = start_div & !done;

//mult multiciclo fake
assign done = (cycles_to_finish == 0);
assign out = done ? a_in * b_in : 'b0; // 'a_in * b_in' tem NUMBER_CYCLES para ficar pronto - backend pode ser ajustado


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
