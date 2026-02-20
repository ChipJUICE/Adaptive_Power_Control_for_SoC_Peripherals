// =============================================================================
// Module: idle_predictor
// Description: Low-power SoC peripheral idle prediction with adaptive thresholds
// =============================================================================
// Determines when peripherals can safely enter SLEEP state based on idle 
// duration and recent activity patterns. Uses lightweight arithmetic-based
// adaptive thresholding with guaranteed latch-free synthesis.
// =============================================================================

module idle_predictor #(
    parameter int N = 4,   // Number of peripherals
    parameter int W = 16   // Counter/threshold width (bits)
) (
    // Clock and reset
    input  logic                clk,
    input  logic                rst_n,
    
    // Idle monitoring inputs
    input  logic [N-1:0][W-1:0] idle_count,       // Per-peripheral idle cycle count
    input  logic [N-1:0][W-1:0] idle_base_th,     // Base idle threshold (programmable)
    input  logic [N-1:0]        recent_activity,  // Recent activity indicator
    
    // Adaptation control
    input  logic [3:0]          alpha,            // Adaptation strength tuning
    
    // Sleep eligibility output
    output logic [N-1:0]        sleep_eligible    // Asserted when sleep is safe
);

    // =========================================================================
    // Internal signals - declared outside generate for proper scoping
    // =========================================================================
    
    logic [N-1:0][W-1:0] adaptive_threshold;      // Computed adaptive threshold
    logic [N-1:0]        sleep_eligible_comb;     // Combinational eligibility
    
    // =========================================================================
    // Per-Peripheral Adaptive Threshold and Sleep Eligibility Logic
    // =========================================================================
    // Generate loop for each peripheral to compute:
    //   adaptive_th = base_th + (recent_activity ? (base_th >> alpha) : 0)
    //   sleep_eligible = (idle_count >= adaptive_th)
    // =========================================================================
    
    generate
        for (genvar i = 0; i < N; i++) begin : g_peripheral
            
            // Local signals for each peripheral (no latches)
            logic [W-1:0]   adjustment;           // Threshold adjustment value
            logic [W:0]     sum_extended;         // Extended for overflow detection
            logic           overflow;             // Overflow flag
            
            // -----------------------------------------------------------------
            // Adaptive Threshold Computation (Combinational)
            // -----------------------------------------------------------------
            // Formula: adaptive_th = base_th + (recent_activity ? adjustment : 0)
            // With saturation at maximum value on overflow
            // -----------------------------------------------------------------
            
            always_comb begin
                // Default values to prevent latches
                adjustment = '0;
                sum_extended = '0;
                overflow = 1'b0;
                adaptive_threshold[i] = '0;
                
                // Compute adjustment based on recent activity
                if (recent_activity[i]) begin
                    adjustment = idle_base_th[i] >> alpha;
                end else begin
                    adjustment = '0;
                end
                
                // Perform addition with overflow detection
                sum_extended = {1'b0, idle_base_th[i]} + {1'b0, adjustment};
                overflow = sum_extended[W];  // MSB indicates overflow
                
                // Apply saturation on overflow
                if (overflow) begin
                    adaptive_threshold[i] = {W{1'b1}};  // Saturate to max value
                end else begin
                    adaptive_threshold[i] = sum_extended[W-1:0];
                end
            end
            
            // -----------------------------------------------------------------
            // Sleep Eligibility Comparison (Combinational)
            // -----------------------------------------------------------------
            // Assert when idle_count meets or exceeds adaptive threshold
            // -----------------------------------------------------------------
            
            always_comb begin
                // Default to prevent latches
                sleep_eligible_comb[i] = 1'b0;
                
                // Compare idle count against adaptive threshold
                sleep_eligible_comb[i] = (idle_count[i] >= adaptive_threshold[i]);
            end
            
        end : g_peripheral
    endgenerate
    
    // =========================================================================
    // Output Registration (Sequential)
    // =========================================================================
    // Register outputs with synchronous reset for clean timing
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sleep_eligible <= '0;
        end else begin
            sleep_eligible <= sleep_eligible_comb;
        end
    end

endmodule : idle_predictor
