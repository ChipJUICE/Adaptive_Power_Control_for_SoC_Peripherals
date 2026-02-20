// =============================================================================
// Module: pwr_ctrl_top
// Description: Top-level wrapper integrating power control subsystem
//              - Config registers
//              - Activity monitoring
//              - Idle prediction
//              - Power FSM
//              - Clock gating
//              - Performance counters
// =============================================================================

module pwr_ctrl_top #(
  parameter int N = 4,   // Number of peripherals
  parameter int W = 16   // Counter width
)(
  // System signals
  input  logic              clk,
  input  logic              rst_n,
  
  // Activity monitoring inputs
  input  logic [N-1:0]      activity_pulse,   // 1-cycle activity pulses
  input  logic [N-1:0]      wake_evt,         // wake-up events
  
  // Configuration bus interface
  input  logic [31:0]       cfg_addr,
  input  logic [31:0]       cfg_wdata,
  input  logic              cfg_we,
  input  logic              cfg_re,
  output logic [31:0]       cfg_rdata,
  
  // Clock gating control
  input  logic              scan_en,
  output logic [N-1:0]      gclk_out,
  
  // Status outputs
  output logic [N-1:0][1:0] state            // per-peripheral power state
);

  // ===========================================================================
  // Internal Signal Declarations
  // ===========================================================================
  
  // Configuration register outputs
  logic [N-1:0]        periph_en;
  logic [N-1:0][W-1:0] idle_base_th;
  logic [3:0]          alpha;
  logic [N-1:0]        wake_mask;
  
  // Activity counter outputs
  logic [N-1:0][W-1:0] idle_count;
  logic [N-1:0]        recent_activity;
  
  // Idle predictor outputs
  logic [N-1:0]        sleep_eligible;
  
  // Power FSM outputs
  logic [N-1:0]        clk_req;
  
  // Performance counter outputs (internal only, not exposed at top)
  logic [N-1:0]        sleep_count;
  logic [N-1:0]        active_cycles;
  logic [N-1:0]        idle_cycles;
  
  // Glue logic signals
  logic [N-1:0]        wake_evt_masked;
  
  // Unused internal signals
  logic                irq;  // IRQ from cfg_regs (not used at top level)

  // ===========================================================================
  // Glue Logic: Wake Event Masking
  // ===========================================================================
  
  always_comb begin
    wake_evt_masked = wake_evt & wake_mask;
  end

  // ===========================================================================
  // Submodule Instantiations
  // ===========================================================================

  // ---------------------------------------------------------------------------
  // 1) Configuration Registers
  // ---------------------------------------------------------------------------
  cfg_regs #(
    .N(N),
    .W(W)
  ) u_cfg_regs (
    .clk            (clk),
    .rst_n          (rst_n),
    .cfg_addr       (cfg_addr[7:0]),  // cfg_regs uses 8-bit address
    .cfg_wdata      (cfg_wdata),
    .cfg_we         (cfg_we),
    .cfg_re         (cfg_re),
    .state          (state),
    .periph_en      (periph_en),
    .idle_base_th   (idle_base_th),
    .alpha          (alpha),
    .wake_mask      (wake_mask),
    .cfg_rdata      (cfg_rdata),
    .irq            (irq)
  );

  // ---------------------------------------------------------------------------
  // 2) Activity Counter
  // ---------------------------------------------------------------------------
  activity_counter #(
    .N(N),
    .W(W)
  ) u_activity_counter (
    .clk              (clk),
    .rst_n            (rst_n),
    .activity_pulse   (activity_pulse),
    .periph_en        (periph_en),
    .idle_count       (idle_count),
    .recent_activity  (recent_activity)
  );

  // ---------------------------------------------------------------------------
  // 3) Idle Predictor
  // ---------------------------------------------------------------------------
  idle_predictor #(
    .N(N),
    .W(W)
  ) u_idle_predictor (
    .clk              (clk),
    .rst_n            (rst_n),
    .idle_count       (idle_count),
    .idle_base_th     (idle_base_th),
    .alpha            (alpha),
    .recent_activity  (recent_activity),
    .sleep_eligible   (sleep_eligible)
  );

  // ---------------------------------------------------------------------------
  // 4) Power FSM
  // ---------------------------------------------------------------------------
  power_fsm #(
    .N(N)
  ) u_power_fsm (
    .clk              (clk),
    .rst_n            (rst_n),
    .sleep_eligible   (sleep_eligible),
    .wake_evt         (wake_evt_masked),   // Masked wake events
    .periph_en        (periph_en),
    .state            (state),
    .clk_req          (clk_req)
  );

  // ---------------------------------------------------------------------------
  // 5) Clock Gater
  // ---------------------------------------------------------------------------
  clock_gater #(
    .N(N)
  ) u_clock_gater (
    .clk_in     (clk),
    .rst_n      (rst_n),
    .clk_req    (clk_req),
    .scan_en    (scan_en),
    .gclk_out   (gclk_out)
  );

  // ---------------------------------------------------------------------------
  // 6) Performance Counters
  // ---------------------------------------------------------------------------
  perf_counters #(
    .N(N)
  ) u_perf_counters (
    .clk            (clk),
    .rst_n          (rst_n),
    .state          (state),
    .sleep_count    (sleep_count),
    .active_cycles  (active_cycles),
    .idle_cycles    (idle_cycles)
  );

endmodule
