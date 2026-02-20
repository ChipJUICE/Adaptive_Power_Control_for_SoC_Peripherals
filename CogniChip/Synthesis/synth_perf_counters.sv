// ==============================================================================
// perf_counters.sv
// Performance counter module for power management FSM
// ==============================================================================
// Collects per-peripheral metrics:
//   - active_cycles: count of cycles spent in ACTIVE state
//   - idle_cycles:   count of cycles spent in IDLE state
//   - sleep_count:   count of transitions into SLEEP state
// ==============================================================================

module perf_counters #(
    parameter int N  = 4,   // Number of peripherals
    parameter int CW = 32   // Counter width for cycle counters and sleep count
) (
    input  logic                clk,
    input  logic                rst_n,          // Active-low synchronous reset
    input  logic [N-1:0][1:0]   state,          // Per-peripheral power state

    output logic [N-1:0][CW-1:0] sleep_count,   // Count of entries into SLEEP
    output logic [N-1:0][CW-1:0] active_cycles, // Cycles spent in ACTIVE state
    output logic [N-1:0][CW-1:0] idle_cycles    // Cycles spent in IDLE state
);

    // State encoding
    localparam logic [1:0] ACTIVE = 2'b00;
    localparam logic [1:0] IDLE   = 2'b01;
    localparam logic [1:0] SLEEP  = 2'b10;

    // Previous state register for transition detection
    logic [N-1:0][1:0] prev_state;

    // Sequential logic: counter updates
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            // Reset: clear all counters and initialize prev_state to ACTIVE
            for (int i = 0; i < N; i++) begin
                sleep_count[i]   <= '0;
                active_cycles[i] <= '0;
                idle_cycles[i]   <= '0;
                prev_state[i]    <= ACTIVE;
            end
        end else begin
            // Per-peripheral counter updates
            for (int i = 0; i < N; i++) begin
                // Cycle counting: increment based on current state
                // Saturate on overflow (hold at max value)
                if (state[i] == ACTIVE) begin
                    if (active_cycles[i] != '1) begin  // Not saturated
                        active_cycles[i] <= active_cycles[i] + 1'b1;
                    end
                end

                if (state[i] == IDLE) begin
                    if (idle_cycles[i] != '1) begin    // Not saturated
                        idle_cycles[i] <= idle_cycles[i] + 1'b1;
                    end
                end

                // Sleep-entry detection: count transitions into SLEEP
                // Increment when previous state was not SLEEP and current is SLEEP
                if ((prev_state[i] != SLEEP) && (state[i] == SLEEP)) begin
                    if (sleep_count[i] != '1) begin    // Not saturated
                        sleep_count[i] <= sleep_count[i] + 1'b1;
                    end
                end

                // Update state history for next cycle
                prev_state[i] <= state[i];
            end
        end
    end

endmodule
