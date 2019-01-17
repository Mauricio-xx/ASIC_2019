module control #(
  parameter WIDTH   = 8,
  parameter NINP    = 3,
  parameter NOPS    = 4,
  parameter OPSIZE  = 6,
  parameter SELSIZE = 2
  )(
  input wire  [0:0]       clk,
  input wire  [0:0]       rst,
  input wire  [OPSIZE-1:0]cmd_in,
  input wire  [0:0]       p_error,
  output reg  [0:0]       aluin_reg_en,
  output reg  [0:0]       datain_reg_en,
  output reg  [0:0]       aluout_reg_en,
  output reg  [0:0]       nvalid_data,
  output wire [NINP-2:0]  in_select_a,
  output wire [NINP-2:0]  in_select_b,
  output reg  [NOPS:0]    opcode
  );

localparam  RST     = 4'b0001,
            FETCH   = 4'b0010,
            LOAD    = 4'b0100,
            EXEC1   = 4'b1000,
            CTRLSTATES  = 4,
            //ALU NOP
            NOP     = 5'b10000;

wire [0:0] fb_data;

reg [CTRLSTATES-1:0]  current_state,
                      next_state;

always @(posedge clk) begin
  if(rst)
    current_state <= RST;
  else
    current_state <= next_state;
end

always @(current_state) begin
  opcode         = NOP;
  aluin_reg_en   = 1'b0;
  datain_reg_en  = 1'b0;
  aluout_reg_en  = 1'b0;
  nvalid_data    = 1'b0;
  next_state     = FETCH;
  case (current_state)
    RST: begin
     opcode         = NOP;
     next_state     = FETCH;
    end
    FETCH: begin
     opcode         = NOP;
     datain_reg_en  = 1'b1;
     next_state     = LOAD;
    end
    LOAD: begin
     opcode         = NOP;
     aluin_reg_en   = 1'b1;
     next_state     = EXEC1;
    end
    EXEC1: begin
     opcode         = 1'b1<<cmd_in[1:0];
     nvalid_data    = (fb_data & p_error) ? 1'b1 : 1'b0;
     aluout_reg_en  = 1'b1;
     next_state     = FETCH;
    end
    /*default: begin
     opcode         = NOP; //opcode;
     next_state     = FETCH; //next_state;
    end*/
  endcase
end

assign in_select_a = cmd_in[OPSIZE-1:OPSIZE-2];
assign in_select_b = cmd_in[OPSIZE-3:OPSIZE-4];
assign fb_data     = (&in_select_a | &in_select_b);

endmodule
