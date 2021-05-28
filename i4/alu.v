
module alu ();


//Adicionar 
alu_div(.a,.b,.busy,.cycles_to_finish,.out,.finish,.done);
alu_mult(.a,.b,.busy,.cycles_to_finish,.out,.finish,.done);
alu_sum(.a,.b,.busy,.cycles_to_finish,.out,.finish,.en);



endmodule