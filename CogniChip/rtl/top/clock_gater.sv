// =============================================================================
// Module: clock_gater
// Description: Parameterized clock gating module with glitch-free latch-based
//              gating mechanism for N peripherals. Each peripheral has an
//              independent clock gate with scan mode support.
//
// Parameters:
//   N - Number of peripherals (default: 4)
//
// Inputs:
//   clk_in      - Input clock
//   rst_n       - Active-low asynchronous reset
//   scan_en     - Scan enable for test mode (bypass gating when high)
//   clk_req     - Clock request bus [N-1:0], one bit per peripheral
//
// Outputs:
//   gclk_out    - Gated clock output bus [N-1:0], one clock per peripheral
//
// =============================================================================

module clock_gater #(
    parameter int N = 4  // Number of peripherals
) (
    input  logic           clk_in,      // Input clock
    input  logic           rst_n,       // Active-low reset
    input  logic           scan_en,     // Scan enable for test mode
    input  logic [N-1:0]   clk_req,     // Clock request per peripheral
    output logic [N-1:0]   gclk_out     // Gated clock output per peripheral
);

    // =========================================================================
    // Internal Signals
    // =========================================================================
    logic [N-1:0] enable_latched;  // Latched enable signals

    // =========================================================================
    // Clock Gating Logic for Each Peripheral
    // =========================================================================
    generate
        for (genvar i = 0; i < N; i++) begin : gen_clock_gates
            
            // Enable logic: allow clock when requested OR in scan mode
            logic enable_in;
            assign enable_in = clk_req[i] | scan_en;
            
            // Glitch-free latch-based clock gating
            // Latch captures enable during clock low phase
            // This prevents glitches on the gated clock output
            always_latch begin
                if (!clk_in) begin
                    enable_latched[i] <= enable_in;
                end
            end
            
            // AND gate: gated clock = clock AND latched_enable
            // The enable is stable during clock high phase, preventing glitches
            assign gclk_out[i] = clk_in & enable_latched[i];
            
        end : gen_clock_gates
    endgenerate

    // =========================================================================
    // Assertions for Verification
    // =========================================================================
    `ifdef SIMULATION
        // Check that parameter is valid
        initial begin
            assert (N > 0) else $fatal(1, "Parameter N must be greater than 0");
        end
        
        // SVA properties (not supported by Verilator)
        `ifndef VERILATOR
            // Property: In scan mode, all clocks should be ungated
            property p_scan_mode_ungates;
                @(posedge clk_in) disable iff (!rst_n)
                scan_en |-> ##1 (gclk_out == {N{1'b1}} || gclk_out == {N{1'b0}});
            endproperty
            
            // Property: When clock is requested and not in reset, clock should eventually be enabled
            generate
                for (genvar i = 0; i < N; i++) begin : gen_assertions
                    property p_req_enables_clock;
                        @(posedge clk_in) disable iff (!rst_n)
                        clk_req[i] && !scan_en |-> ##[0:1] gclk_out[i];
                    endproperty
                    
                    assert_req_enables: assert property (p_req_enables_clock)
                        else $warning("Clock request %0d did not enable gated clock", i);
                end
            endgenerate
        `endif
    `endif

endmodule : clock_gater
