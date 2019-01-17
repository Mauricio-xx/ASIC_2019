module alu #(
  parameter WIDTH = 8,
  parameter NOPS = 4
  )(
  input wire        [NOPS:0]    opcode,
  input wire signed [WIDTH-1:0] in_a,
  input wire signed [WIDTH-1:0] in_b,
  input wire        [0:0]       nvalid_data,
  output wire       [0:0]       negative,
  output wire       [0:0]       zero,
  output wire       [0:0]       error,
  output wire       [WIDTH-1:0] out_low,
  output wire       [WIDTH-1:0] out_high
  );

localparam  SUM = 5'b00001,
            SUB = 5'b00010,
            MUL = 5'b00100,
            DIV = 5'b01000,
            NOP = 5'b10000,
            SIZE = WIDTH*2;

reg signed [SIZE-1:0] out;
wire [0:0] div_zero;

always @(*) begin
  //ALU_OP_HOT: assert final ($onehot(opcode)) else $error("The ALU opcode is badly coded!");
  case (opcode)
    SUM: out = in_a + in_b;
    SUB: out = in_a - in_b;
    MUL: out = in_a * in_b;
    DIV: out = in_a / in_b;
    NOP: out = 1'sb0 <<< SIZE; //{SIZE{1'b0}};
    default: out = 1'sb0 <<< SIZE; //{SIZE{1'b0}};
  endcase
end

assign div_zero = (opcode == DIV) ? (~|in_b ? 1'b1 : 1'b0) : 1'b0;
assign error = (div_zero | nvalid_data) ? 1'b1 : 1'b0;

assign negative = out[SIZE-1];
assign zero = ~|{out_high, out_low}; // Affects timming

assign out_high = error ? {SIZE{1'b1}} : out[SIZE-1:WIDTH];
assign out_low  = error ? {SIZE{1'b1}} : out[WIDTH-1:0];
//assign out_high = error ? -1 : out[SIZE-1:WIDTH];
//assign out_low  = error ? -1 : out[WIDTH-1:0];

endmodule
