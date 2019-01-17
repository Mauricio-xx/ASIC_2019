module register_bank #(
  parameter WIDTH = 8
  )(
  input wire [0:0] clk,
  input wire [0:0] rst,
  input wire [0:0] wr_en,
  input wire [WIDTH-1:0] din,
  output reg [WIDTH-1:0] dout
  );

always @(posedge clk) begin
  if(rst)
    dout <= {WIDTH{1'b0}};
  else if(wr_en)
    dout <= din;
end

endmodule

