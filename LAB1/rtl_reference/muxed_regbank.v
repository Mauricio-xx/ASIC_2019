module mux4_register_bank #(
  parameter WIDTH   = 8,
            SELSIZE = 2,
            IN1 = 2'b00,
            IN2 = 2'b01,
            IN3 = 2'b10,
            IN4 = 2'b11
  )(
  input wire [0:0] clk,
  input wire [0:0] rst,
  input wire [0:0] wr_en,
  input wire [SELSIZE-1:0] select,
  input wire [WIDTH-1:0]   din_1,
  input wire [WIDTH-1:0]   din_2,
  input wire [WIDTH-1:0]   din_3,
  input wire [WIDTH-1:0]   din_4,
  output reg [WIDTH-1:0]   dout
  );

reg [WIDTH-1:0] muxout;

always @(posedge clk) begin
  if(rst)
    dout <= {WIDTH{1'b0}};
  else if(wr_en)
    dout <= muxout;
end

always @(*) begin
  case (select)
    IN1: muxout = din_1;
    IN2: muxout = din_2;
    IN3: muxout = din_3;
    IN4: muxout = din_4;
    //default: ;
  endcase
end

endmodule

