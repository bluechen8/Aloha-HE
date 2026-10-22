`timescale 1ns / 1ps

module ComputeCoreWrapper #(
    parameter SRAM_REUSE = 0, // enabled only by the integrated secret-key walker
    parameter FFT_ON_THE_FLY_GENERATION = 0,
    parameter PROVIDE_DEBUG_IO = 0,
    parameter LOGN = 13,
    parameter N    = 1 << LOGN,
    parameter SCHEME = 0,  // 0 = public-key (vendor PWM ref); 1 = secret-key (PWMSk)
    parameter SEED_SIDEBAND = 0, // 1 = sampler seed from prng_seed, not dina_ext
    // 1 = the integrated walker holds ComputeCore's command for each pass
    // (ext_command, 0 between passes, so its opcode rising marks the start) and
    // ISA_control with its INS_RAM is not built; status[0] is then "this pass
    // is done".
    parameter EXT_COMMAND = 0
  ) (clk,
                    reseed_en, prng_rst_i,   // C'-2 free-run PRNG controls (pass-through to CORE)
                    prng_seed,               // 160-bit sampler seed (SEED_SIDEBAND=1)
                    // AXI registers for controlling the co-processor
                    control_low_word, control_high_word,
                    dina_ext_low_word, dina_ext_high_word, 
                    dout_ext_low_word, dout_ext_high_word, 
                    status,
                    // DMA interface: 
                    dma_bram_byte_wea, 
                    dma_bram_abs_addr, 
                    dma_bram_dina,
                    dma_bram_doutb,
                    dma_bram_en,
                    reuse_keygen, reuse_limb, m_aloha_mem,   // C'-3: Aloha storage banks lifted to top
                    ext_command);
input [39:0] ext_command;   // EXT_COMMAND=1 only
input clk;
input reuse_keygen;
input [3:0] reuse_limb;
input reseed_en, prng_rst_i;   // C'-2 free-run PRNG controls
input [159:0] prng_seed;       // Trivium key [79:0], IV [159:80]
input [31:0] control_low_word, control_high_word;
input [31:0] dina_ext_low_word, dina_ext_high_word;

output [31:0] dout_ext_low_word, dout_ext_high_word;
output [31:0] status;

input [7:0] dma_bram_byte_wea;
input [17:0] dma_bram_abs_addr;
input [63:0] dma_bram_dina;
output [63:0] dma_bram_doutb;
input dma_bram_en;

fhe_aloha_mem_if.req m_aloha_mem;   // C'-3: pass-through to CORE

wire [63:0] dina_ext = {dina_ext_high_word,dina_ext_low_word};
wire [63:0] dout_ext;
reg [31:0] status;
wire rst_core;

wire [LOGN:0] address_ext;
wire [2:0] bram_sel;
wire wea_ext, grant_ext, wea_ext_core;

wire [39:0] command_in;
wire command_we;

wire done_ins_computation; 
wire done_all_computation; // This becomes 1 when the cryptoprocessor has finished executing all instructions.

wire [31:0] status_wire;

assign address_ext = control_low_word[LOGN:0];
// Flags at 17 (wea), 18 (ins), 19 (grant): fhe_params_pkg FHE_CW_*.
assign wea_ext = control_low_word[17];
assign grant_ext = control_low_word[19];
assign bram_sel = control_low_word[31:29];

assign {dout_ext_high_word,dout_ext_low_word} = dout_ext;

always @(posedge clk)
    status <= status_wire;
    
ComputeCore #(
    .FFT_ON_THE_FLY_GENERATION(FFT_ON_THE_FLY_GENERATION),
    .PROVIDE_DEBUG_IO(PROVIDE_DEBUG_IO),
    .LOGN(LOGN),
    .N(N),
    .SRAM_REUSE(SRAM_REUSE),
    .SCHEME(SCHEME),
    .SEED_SIDEBAND(SEED_SIDEBAND),
    .EXT_COMMAND(EXT_COMMAND)
  ) CORE(
    .clk(clk),
    .reuse_keygen(reuse_keygen), .reuse_limb(reuse_limb),
    .rst(rst_core),
    .reseed_en(reseed_en),
    .prng_rst_i(prng_rst_i),
    .prng_seed(prng_seed),
    .address_ext(address_ext), 
    .bram_sel(bram_sel), 
    .dina_ext(dina_ext), 
    .doutb_ext(dout_ext),
    .wea_ext(wea_ext_core),
		.command_in(command_in), 
    .command_we(command_we),
		.done_ins_computation(done_ins_computation),
		.grant_ext_io(grant_ext),
    .dma_bram_byte_wea(dma_bram_byte_wea), 
    .dma_bram_abs_addr(dma_bram_abs_addr), 
    .dina_dma(dma_bram_dina),
    .doutb_dma(dma_bram_doutb),
    .dma_bram_en(dma_bram_en),
    .m_aloha_mem(m_aloha_mem));


assign rst_core = control_high_word[0];

generate if (EXT_COMMAND) begin : ext_command_path
  // Same timing as ISA_control: done counts from the third cycle after the
  // command is loaded, and holds until the core is reset.
  reg [1:0] issued;
  reg armed, pass_done;
  always @(posedge clk) begin
    if (rst_core) begin
      issued <= 2'd0; armed <= 1'b0; pass_done <= 1'b0;
    end else begin
      issued <= {issued[0], |ext_command[2:0]};
      if (issued[1]) armed <= 1'b1;
      if (armed && done_ins_computation) pass_done <= 1'b1;
    end
  end
  assign command_in = ext_command;
  assign command_we = 1'b0;  // the core has no command register here
  assign wea_ext_core = wea_ext;
  assign done_all_computation = pass_done;
  assign status_wire = {31'd0, pass_done};
end else begin : isa_path
wire [30:0] cycle_count;
wire rst_ISA = control_high_word[0];
wire start_ISA = control_high_word[1];
assign wea_ext_core = wea_ext & ~control_low_word[18];
wire wea_ext_ISA = wea_ext & control_low_word[18];
assign status_wire = {cycle_count[29:0], 1'd0, done_all_computation};
ISA_control ISA_CTRL(clk, rst_ISA, start_ISA, done_ins_computation,
                  address_ext[3:0], dina_ext[41:0], wea_ext_ISA,
									command_in,	command_we,
									done_all_computation,
									cycle_count);
end endgenerate

endmodule
