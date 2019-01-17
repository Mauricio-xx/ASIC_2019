/*
 *   Test case:   Lapidary ALU
 *   Author:      gonzalof
 *   Version:     Basic no multistage
 *                Synchronus reset
 *   Rev:         1.2 07.10.2017
 */

module top #(
  parameter WIDTH = 8
  )(
  input wire  [0:0]        clk,
  input wire  [0:0]        rst,
  input wire  [5:0]        cmdin,
  input wire  [WIDTH-1:0]  din_1,
  input wire  [WIDTH-1:0]  din_2,
  input wire  [WIDTH-1:0]  din_3,
  //input wire  [WIDTH-1:0]  din_4,
  output wire [WIDTH-1:0]  dout_low,
  output wire [WIDTH-1:0]  dout_high,
  output wire [0:0]        zero,
  output wire [0:0]        error
  );

localparam  NOPS = 4,
            OPSIZE  = 6,
            SELSIZE = 2,
            IN1 = 2'b00,
            IN2 = 2'b01,
            IN3 = 2'b10,
            IN4 = 2'b11;

wire [0:0]          cmd_reg_en, din_reg_en, alu_reg_en,
                    nvalid_data;
wire [NOPS:0]       opcode;
wire [OPSIZE-1:0]   cmd_reg;
wire [SELSIZE-1:0]  select_a, select_b;
wire [WIDTH-1:0]    out_low, out_high,
                    data_a, data_b,
                    output_din4;

// OP register
register_bank #(
  .WIDTH(OPSIZE)
  ) op_bank (
  .clk(clk),
  .rst(rst),
  .wr_en(cmd_reg_en),
  .din(cmdin),
  .dout(cmd_reg)
  );

// DATA IN MUX and REG
mux4_register_bank #(
  .WIDTH(WIDTH)
  ) datain_a_bank (
  .clk(clk),
  .rst(rst),
  .wr_en(din_reg_en),
  .select(select_a),
  .din_1(din_1),
  .din_2(din_2),
  .din_3(din_3),
  .din_4(dout_high),
  .dout(data_a)
  );
mux4_register_bank #(
  .WIDTH(WIDTH)
  ) datain_b_bank (
  .clk(clk),
  .rst(rst),
  .wr_en(din_reg_en),
  .select(select_b),
  .din_1(din_1),
  .din_2(din_2),
  .din_3(din_3),
  .din_4(dout_low),
  .dout(data_b)
  );

// Main controller
control #(
  .WIDTH(WIDTH),
  .NOPS(NOPS)
  ) the_controler (
  .clk(clk),
  .rst(rst),
  .cmd_in(cmd_reg),
  .p_error(error),
  .aluin_reg_en(din_reg_en),
  .datain_reg_en(cmd_reg_en),
  .aluout_reg_en(alu_reg_en),
  .nvalid_data(nvalid_data),
  .in_select_a(select_a),
  .in_select_b(select_b),
  .opcode(opcode)
  );

// The ALU
alu #(
  .WIDTH(WIDTH),
  .NOPS(NOPS)
  ) calculator(
  .opcode(opcode),
  .in_a(data_a),
  .in_b(data_b),
  .nvalid_data(nvalid_data),
  .negative(alu_negative),
  .zero(alu_zero),
  .error(alu_error),
  .out_low(out_low),
  .out_high(out_high)
  );

// ALUOUT REG
register_bank #(
  .WIDTH(WIDTH*2+2)
  ) aluout_bank (
  .clk(clk),
  .rst(rst),
  .wr_en(alu_reg_en),
  .din({alu_error, alu_zero, out_high, out_low}),
  .dout({error, zero, dout_high, dout_low})
  );

endmodule

