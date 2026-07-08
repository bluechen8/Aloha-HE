`timescale 1ns / 1ps
`include "CommonDefinitions.vh"

// C'-2: the sampler PRNG is the audited OpenTitan/Caliptra Trivium
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
// -------------------------------------------------------------------------
// C'-2 STEP 2 (free-run) -- INTEGRATED.  The Trivium is FREE-RUNNING: its state
// persists across sampling passes and is reloaded ONLY on an explicit `reseed`
// pulse, decoupled from the per-pass sampling-FSM reset.  This lets one instance
// serve both:
//   * keygen's deterministic ternary `s`  -> reseed with the KV/keygen seed
//     right before the keygen sampling pass (same KV root => same sk), and
//   * per-ciphertext fresh `a`/`e0`        -> NO reseed between encrypt passes,
//     so the keystream continues and a/e0 never repeat by construction
//     (Trivium period ~2^64+ blocks).
//
// Two cycle-accurate integration hazards are handled here so the sampler is
// bit-exact vs the per-pass-reload behaviour whenever a reseed DOES fire:
//   (1) IDLE DRIFT -- the stream must FREEZE between passes, else it advances
//       during the post-warm idle gap and a reseeded pass no longer lands at
//       word0.  FIX: `en_i = valid_q & active`; the caller drives `active` low
//       between passes (RandomSampling ties it to ~rst), so the state (and hence
//       key_o) holds while a pass is not consuming, and advances only during a
//       pass.  The Caliptra prim's key_o is combinational from state_q, so a
//       frozen state presents a stable word0 at pass release.
//   (2) STALE-VALID RACE -- because `valid` persists across passes, a reseed that
//       coincides with a consuming cycle would let the sampler eat one stale word
//       before valid drops.  FIX: `random_valid` is masked low on the reseed
//       cycle (`valid_q & ~reseed_pulse`); valid_q itself then drops on the same
//       pulse and re-rises on seed_done, so the consumer resumes at word0.
//
// Ports:
//   rst    - active-high power-on/global reset (NOT pulsed per sampling pass).
//            Holds the primitive in its default state; random_valid stays low
//            until the first reseed completes.
//   reseed - pulse (>=1 cycle; internally edge-detected to 1 cycle) to load
//            `seed` as the Trivium key (iv=0) and run the automatic 1152-bit
//            (18 x 64) KeyIv warmup; random_valid drops during warmup and rises
//            when warmup completes.
//   active - 1 while a sampling pass is actively consuming the keystream; 0
//            freezes the stream (state held) between passes.
//   seed   - 64-bit reseed value (sampled at the reseed pulse).
// Between reseeds, while `active`, the Trivium free-runs one 64-bit word per
// cycle and random_valid stays high, so the keystream is continuous across passes.
module TriviumAdapter(
    input        clk,
    input        rst,
    input        reseed,
    input        active,
    input [63:0] seed,
    output [63:0] random_out,
    output        random_valid
  );

  logic rst_n;
  assign rst_n = ~rst;

  // Edge-detect `reseed` -> a clean 1-cycle load pulse.  A 1-cycle pulse is
  // REQUIRED: for SeedTypeKeyIv the primitive ties last_state_part=0, so
  // seed_req latches high; holding seed_ack (=reseed) high would re-load the
  // state every cycle and the init-updates would never complete (seed_done
  // never fires).  Edge detection makes a level-held `reseed` safe too.
  logic reseed_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) reseed_q <= 1'b0;
    else        reseed_q <= reseed;
  end
  logic reseed_pulse;
  assign reseed_pulse = reseed & ~reseed_q;

  // Key/IV seed: 64-bit Aloha seed -> Trivium key (low 64 of 80), iv = 0.
  logic [79:0] seed_key;
  assign seed_key = {16'd0, seed};

  logic        seed_done;

  // valid_q: low until the first reseed's warmup completes; drops again while a
  // subsequent reseed re-warms; otherwise held high (free-run).
  logic valid_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)            valid_q <= 1'b0;
    else if (reseed_pulse) valid_q <= 1'b0;   // re-warming: invalidate
    else if (seed_done)    valid_q <= 1'b1;    // warmup complete
  end

  caliptra_prim_trivium #(
    .BiviumVariant (1'b0),
    .OutputWidth   (64),
    .SeedType      (caliptra_prim_trivium_pkg::SeedTypeKeyIv)
  ) u_trivium (
    .clk_i                (clk),
    .rst_ni               (rst_n),
    // Advance only while the keystream is valid AND a pass is consuming
    // (`active`).  valid_q PERSISTS across passes (only a reseed or rst drops
    // it); `active` freezes the state between passes so the reseeded/continuing
    // stream lands at the right word.  NOT a constant 1: that would drift the
    // stream during the inter-pass idle gap and race past word0.
    .en_i                 (valid_q & active),
    .allow_lockup_i       (1'b0),
    .seed_en_i            (reseed_pulse),
    .seed_done_o          (seed_done),
    .seed_req_o           (),              // (KeyIv holds this high after seed_en; unused)
    .seed_ack_i           (reseed_pulse),  // one-cycle ack coincident with the request
    .seed_key_i           (seed_key),
    .seed_iv_i            (80'd0),
    .seed_state_full_i    ('0),
    .seed_state_partial_i ('0),
    .key_o                (random_out),
    .err_o                ()
  );

  // Mask valid on the reseed cycle so a consumer never eats a stale word when a
  // reseed coincides with an active pass (hazard 2 above); valid_q drops the
  // next cycle and re-rises on seed_done.
  assign random_valid = valid_q & ~reseed_pulse;

endmodule
