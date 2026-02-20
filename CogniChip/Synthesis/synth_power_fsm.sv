// =============================================================================
// Module: power_fsm
// Description: Low-power SoC peripheral controller with per-peripheral FSMs
// =============================================================================
// Implements a 3-state power management FSM (ACTIVE → IDLE → SLEEP) for each
// peripheral. Transitions are controlled by sleep eligibility, wake events,
// and peripheral enable signals with strict priority rules.
// =============================================================================

module power_fsm #(
    parameter int N = 4  // Number of peripherals
) (
    // Clock and reset
    input  logic             clk,
    input  logic             rst_n,        // Active-low synchronous reset
    
    // Control inputs per peripheral
    input  logic [N-1:0]     sleep_eligible,  // Request to enter lower power
    input  logic [N-1:0]     wake_evt,        // Wake event (highest priority)
    input  logic [N-1:0]     periph_en,       // Peripheral enable
    
    // Status outputs per peripheral
    output logic [N-1:0][1:0] state,         // Current power state
    output logic [N-1:0]      clk_req        // Clock request (1=ON, 0=gate)
);

    // =========================================================================
    // State Encoding
    // =========================================================================
    
    localparam logic [1:0] ACTIVE = 2'b00;   // Fully powered, processing
    localparam logic [1:0] IDLE   = 2'b01;   // Light sleep, quick wake
    localparam logic [1:0] SLEEP  = 2'b10;   // Deep sleep, clock gated
    
    // =========================================================================
    // Internal Signals
    // =========================================================================
    
    logic [N-1:0][1:0] state_next;           // Next state for each peripheral
    
    // =========================================================================
    // Per-Peripheral FSM Logic
    // =========================================================================
    // Generate N independent FSMs with identical transition logic
    // Priority: wake_evt > periph_en > normal transitions
    // =========================================================================
    
    generate
        for (genvar i = 0; i < N; i++) begin : g_fsm
            
            // -----------------------------------------------------------------
            // Next-State Logic (Combinational)
            // -----------------------------------------------------------------
            // Priority hierarchy:
            //   1. periph_en=0 → force SLEEP
            //   2. wake_evt=1 (when enabled) → force ACTIVE
            //   3. Normal state transitions based on sleep_eligible
            // -----------------------------------------------------------------
            
            always_comb begin
                // Default: hold current state
                state_next[i] = state[i];
                
                // Priority 1: Peripheral disabled → force SLEEP
                if (!periph_en[i]) begin
                    state_next[i] = SLEEP;
                end
                
                // Priority 2: Wake event when enabled → force ACTIVE
                else if (wake_evt[i]) begin
                    state_next[i] = ACTIVE;
                end
                
                // Priority 3: Normal state transitions when enabled and no wake
                else begin
                    case (state[i])
                        
                        ACTIVE: begin
                            // Transition to IDLE if sleep is eligible
                            if (sleep_eligible[i]) begin
                                state_next[i] = IDLE;
                            end
                            // Otherwise stay ACTIVE
                        end
                        
                        IDLE: begin
                            // Transition to SLEEP if still sleep eligible
                            if (sleep_eligible[i]) begin
                                state_next[i] = SLEEP;
                            end
                            // Return to ACTIVE if no longer eligible
                            else begin
                                state_next[i] = ACTIVE;
                            end
                        end
                        
                        SLEEP: begin
                            // Stay in SLEEP until wake event
                            // (wake_evt priority already checked above)
                            state_next[i] = SLEEP;
                        end
                        
                        default: begin
                            // Recover from illegal states
                            state_next[i] = ACTIVE;
                        end
                        
                    endcase
                end
            end
            
            // -----------------------------------------------------------------
            // State Register (Sequential)
            // -----------------------------------------------------------------
            // Synchronous reset to ACTIVE state
            // -----------------------------------------------------------------
            
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    state[i] <= ACTIVE;
                end else begin
                    state[i] <= state_next[i];
                end
            end
            
            // -----------------------------------------------------------------
            // Clock Request Logic (Combinational)
            // -----------------------------------------------------------------
            // Clock needed in ACTIVE and IDLE states, gated in SLEEP
            // Overridden to 0 when peripheral is disabled
            // -----------------------------------------------------------------
            
            always_comb begin
                if (!periph_en[i]) begin
                    // Disabled peripheral doesn't need clock
                    clk_req[i] = 1'b0;
                end else begin
                    // Clock request based on current state
                    clk_req[i] = (state[i] == ACTIVE) || (state[i] == IDLE);
                end
            end
            
        end : g_fsm
    endgenerate

endmodule : power_fsm
