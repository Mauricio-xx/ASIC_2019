module register_bank #(
parameter WIDTH = 8
 ) (

	input  clk,
	input  rst,
	input  wr_en,
	input [WIDTH-1:0] in,
	output reg [WIDTH-1:0] out
);
always @(posedge clk && wr_en)
	  
       
	 if(rst)
         out <= 8'b0000_0000;
 	 else
		out <= in;	 
endmodule	

	
