`timescale 10ns/1ps
module test_bench;

reg clk,rst;
reg [0:1] sel;
reg [0:7] in1,in2,in3,in4;
wire [0:7] out;
mux4_registered U2 (.clk(clk), .sel(sel), .rst(rst), .in1(in1), .in2(in2), .in3(in3), .in4(in4), .out(out)); 

	initial 
		begin 
			clk = 1;
			rst = 0; sel = 00; in1 = 8'b0000_0000; in2 = 8'b0000_0110; in3 = 8'b0000_1110; in4 = 8'b0001_0010; #10
			rst = 0; sel = 11; in1 = 8'b0000_0000; in2 = 8'b0000_0110; in3 = 8'b0000_1110; in4 = 8'b0001_0010; #10
			rst = 0; sel = 10; in1 = 8'b0000_0000; in2 = 8'b0000_0110; in3 = 8'b0000_1110; in4 = 8'b0001_0010; #10
			rst = 1; sel = 10; in1 = 8'b0000_0000; in2 = 8'b0000_0110; in3 = 8'b0000_1110; in4 = 8'b0001_0010; #10
                        #20 $finish;
		end

initial 
	begin 
		forever #5 clk = ~clk;
	end
initial
	begin
		$dumpvars;
	$display("rst |clk  | sel | in1 | in2| in3 | in4 | out  ");
	$monitor(" %b |%b   | %b  | %b  | %b  | %b | %b  | %b   ",rst,clk,sel,in1,in2,in3,in4,out);
end
endmodule
