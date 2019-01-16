module mux4_registered #(
parameter WIDTH = 8 
) (
input [0:0] clk,
input [0:0] rst,
input [1:0] sel,
input [WIDTH-1:0] in1, in2, in3, in4,
output reg[WIDTH-1:0] out
);
always @(posedge clk,sel,in1,in2,in4)
begin
	if(rst)
		out <= 8'b0000_0000; 
	else case(sel)
		2'b00 : out = in1;
		2'b01 : out = in2;
		2'b10 : out = in3;
		2'b11 : out = in4;  
		default $display ("error!");
	endcase
end
endmodule 
 
