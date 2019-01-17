module fulladder #(
  parameter CLOCKED = 1
  )(
  input wire [0:0] a, b, cin, clk, rst,
  output reg [0:0] sum, cout
);

wire [0:0] s1,c1,c2;

assign s1 = a ^ b;
assign c1 = a & b;
assign c2 = s1 & cin;

if(CLOCKED>0) begin:seqadder
  always @(posedge clk or posedge rst) begin
    if(rst) begin
      sum  <= 1'b0;
      cout <= 1'b0;
    end else begin
      sum  <= s1 ^ cin;
      cout <= c1 | c2;
    end
  end
end else begin:combadder
  always @(*) begin
    sum  <= s1 ^ cin;
    cout <= c1 | c2;
  end
end

endmodule
