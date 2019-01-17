`timescale 10ns/1ps
module test_bench;

reg[0:7] a, b, c, d;
reg [0:1] sel;
wire[0:7] out;

mux4_registered U1 (.din1(a), .din2(b), .din3(c), .din4(d), .dout(out), .select(sel));

initial 
begin

	a=8'b0000_0001; b=8'b0000_0010; c = 8'b0000_0011; d=8'b0000_0100; sel = 2'b10;

end 

initial
begin
	$dumpvars;
	$display ("din1 | din2 | din3 | din4 | dout   | select ");
	$monitor (" %b  | %b   | %b  |  %b  |  %b    | %b  "    , a, b, c, d, out ,sel);
end
endmodule


