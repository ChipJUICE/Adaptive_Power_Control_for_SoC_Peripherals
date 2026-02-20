//==============================================================================
// Module: activity_counter
// Description: Activity Monitoring and Idle Cycle Counter
//              Tracks peripheral activity and counts consecutive idle cycles.
//              Resets counter on activity pulse, saturates at maximum count.
//==============================================================================

module activity_counter #(
    parameter int N = 4,          // Number of peripherals
    parameter int W = 16,         // Width of idle counter (bits)
    parameter int ACTIVITY_WINDOW = 8  // Cycles to track recent activity
) (
    // Clock and Reset
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Activity Input from Peripherals
    input  logic [N-1:0]            activity_pulse,
    
    // Enable Control from cfg_regs
    input  logic [N-1:0]            periph_en,
    
    // Outputs
    output logic [N-1:0][W-1:0]     idle_count,
    output logic [N-1:0]            recent_activity
);

    //==========================================================================
    // Internal Registers
    //==========================================================================
    logic [N-1:0][W-1:0]    idle_counter;
    logic [N-1:0][$clog2(ACTIVITY_WINDOW):0] activity_timer;
    
    // Maximum counter value (all 1's)
    localparam logic [W-1:0] MAX_COUNT = {W{1'b1}};

    //==========================================================================
    // Idle Counter Logic (Per Peripheral)
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idle_counter <= '0;
        end else begin
            for (int i = 0; i < N; i++) begin
                if (!periph_en[i]) begin
                    // Disabled peripheral: hold counter at 0
                    idle_counter[i] <= '0;
                    
                end else if (activity_pulse[i]) begin
                    // Activity detected: reset counter
                    idle_counter[i] <= '0;
                    
                end else if (idle_counter[i] < MAX_COUNT) begin
                    // No activity and not saturated: increment
                    idle_counter[i] <= idle_counter[i] + 1'b1;
                    
                end else begin
                    // Saturated at maximum: hold value
                    idle_counter[i] <= MAX_COUNT;
                end
            end
        end
    end

    //==========================================================================
    // Recent Activity Flag Logic
    // Tracks if there was activity within the last ACTIVITY_WINDOW cycles
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            activity_timer <= '0;
        end else begin
            for (int i = 0; i < N; i++) begin
                if (!periph_en[i]) begin
                    // Disabled peripheral: clear timer
                    activity_timer[i] <= '0;
                    
                end else if (activity_pulse[i]) begin
                    // Activity detected: load timer with window duration + 1
                    // This ensures recent_activity stays high for exactly ACTIVITY_WINDOW cycles
                    activity_timer[i] <= ACTIVITY_WINDOW + 1;
                    
                end else if (activity_timer[i] > 0) begin
                    // Decrement timer until it reaches zero
                    activity_timer[i] <= activity_timer[i] - 1'b1;
                    
                end else begin
                    // Timer already at zero: hold
                    activity_timer[i] <= '0;
                end
            end
        end
    end
    
    // Recent activity flag is high when timer is non-zero
    always_comb begin
        for (int i = 0; i < N; i++) begin
            recent_activity[i] = (activity_timer[i] > 0) && periph_en[i];
        end
    end

    //==========================================================================
    // Output Assignments
    //==========================================================================
    assign idle_count = idle_counter;

    //==========================================================================
    // Assertions for Parameter Validation
    //==========================================================================
    initial begin
        assert (N > 0 && N <= 32) 
            else $error("N must be between 1 and 32");
        assert (W > 1 && W <= 32) 
            else $error("W must be between 2 and 32");
        assert (ACTIVITY_WINDOW > 0 && ACTIVITY_WINDOW <= 256) 
            else $error("ACTIVITY_WINDOW must be between 1 and 256");
    end

    //==========================================================================
    // Coverage and Debug Assertions
    //==========================================================================
    // Check for counter saturation (useful for design validation)
    generate
        for (genvar i = 0; i < N; i++) begin : gen_saturation_check
            `ifndef SYNTHESIS
            always_ff @(posedge clk) begin
                if (rst_n && periph_en[i]) begin
                    if (idle_counter[i] == MAX_COUNT) begin
                        // Counter saturated - may want to increase W or adjust thresholds
                        // This is informational, not an error
                    end
                end
            end
            `endif
        end
    endgenerate

endmodule
