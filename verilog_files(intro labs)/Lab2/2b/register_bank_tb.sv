`timescale 10ns/1ps
module test_bench;

reg clk, rst, wr_en;
reg[7:0] in;
wire [7:0] out;

register_bank U2 (.clk(clk), .rst(rst), .wr_en(wr_en), .in(in), .out(out));
	initial
		begin
			clk = 1;
			wr_en = 1; rst = 0; in = 8'b0000_0001; #10;
			wr_en = 1; rst = 1; in = 8'b1111_1111; #10;
			wr_en = 1; rst = 0; in = 8'b1111_0000; #10;
     			#20 $finish;
		end
		
initial
	begin
		forever #5 clk = ~clk;
end 

initial
begin
	$dumpvars;
	$display("wr_en | clk | rst | in | out ");
	$monitor(" %b   | %b  | %b  | %b | %b  ",wr_en,clk,rst,in,out);
end
endmodule

