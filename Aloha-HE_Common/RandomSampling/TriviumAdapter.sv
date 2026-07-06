`timescale 1ns / 1ps
`include "CommonDefinitions.vh"

// C'-2 step 1: the sampler PRNG is the audited OpenTitan/Caliptra Trivium
// (`caliptra_prim_trivium`), replacing the vendored `trivium64_update`.  Both are
// the SAME eSTREAM Trivium cipher (taps 66/93/162/177/243/288); the Caliptra
// primitive is the in-tree, verified implementation AES/SHA3 already use.
// "One PRNG, not two" -- the whole design standardises on caliptra_prim_trivium.
//
// The Caliptra primitive uses a different internal state representation and
// key/IV seed mapping, so its keystream differs bit-for-bit from trivium64_update;
// the sampling goldens are regenerated from a SW model of THIS primitive
// (tvgen/trivium.py CaliptraPrimTrivium + gen_sampling.py).
//
// This module preserves the original (clk, rst, seed, random_out, random_valid)
// contract, and times `random_valid` to rise coincident with the first
// post-warmup keystream word -- byte-identical protocol timing to the retired
// Trivium64, so `RandomSampling` and the whole datapath are unchanged; only the
// keystream VALUES differ.  The 64-bit `seed` is loaded as the Trivium key (low
// 64 of 80, iv=0) via SeedTypeKeyIv, which performs the 1152-bit (18 x 64)
// warmup automatically before asserting `seed_done_o`.  A one-shot `seed_en`
// pulse on the first cycle out of `rst` kicks off the (self-acked) reseed.
//
// NOTE: the free-run evolution (Trivium persists across passes, reseed decoupled
// from the per-pass rst; for keygen-deterministic-s + fresh a/e0) is C'-2 Step 2,
// staged under src/fhe/design/step2_freerun/ and NOT integrated here.
module TriviumAdapter(
    input clk,
    input rst,
    input [63:0] seed,
    output [63:0] random_out,
    output        random_valid
  );

  // Active-low reset for the Caliptra primitive.
  logic rst_n;
  assign rst_n = ~rst;

  // One-shot reseed pulse: assert seed_en on the first cycle after rst deasserts.
  logic seeded_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) seeded_q <= 1'b0;
    else        seeded_q <= 1'b1;
  end
  logic seed_en;
  assign seed_en = rst_n & ~seeded_q;

  // Key/IV seed: 64-bit Aloha seed -> Trivium key (low 64 bits), iv = 0.
  logic [79:0] seed_key;
  assign seed_key = {16'd0, seed};

  logic seed_done;

  // Free-run once the keystream is usable (post-warmup); hold random_valid high.
  logic valid_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) valid_q <= 1'b0;
    else if (seed_done) valid_q <= 1'b1;
  end

  caliptra_prim_trivium #(
    .BiviumVariant (1'b0),
    .OutputWidth   (64),
    .SeedType      (caliptra_prim_trivium_pkg::SeedTypeKeyIv)
  ) u_trivium (
    .clk_i                (clk),
    .rst_ni               (rst_n),
    .en_i                 (valid_q),   // advance once warmup done; init updates are automatic
    .allow_lockup_i       (1'b0),
    .seed_en_i            (seed_en),
    .seed_done_o          (seed_done),
    .seed_req_o           (),          // (KeyIv holds this high after seed_en; unused)
    .seed_ack_i           (seed_en),   // one-cycle ack coincident with the request:
                                       // loads the state exactly once, then the
                                       // automatic 1152-bit init-updates run to seed_done.
    .seed_key_i           (seed_key),
    .seed_iv_i            (80'd0),
    .seed_state_full_i    ('0),
    .seed_state_partial_i ('0),
    .key_o                (random_out),
    .err_o                ()
  );

  assign random_valid = valid_q;

endmodule
