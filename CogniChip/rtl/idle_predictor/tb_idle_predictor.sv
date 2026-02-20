// =============================================================================
// Testbench: tb_idle_predictor
// Description: Self-checking testbench for idle_predictor RTL module
// =============================================================================
// Verifies adaptive idle threshold computation and sleep eligibility decisions
// for low-power SoC peripheral controller.
// =============================================================================

`timescale 1ns/1ps

module tb_idle_predictor;

    // =========================================================================
    // Parameters (matching DUT)
    // =========================================================================
    
    localparam int N = 4;   // Number of peripherals
    localparam int W = 16;  // Counter/threshold width
    
    // =========================================================================
    // Clock and Reset
    // =========================================================================
    
    logic clock;
    logic reset;
    
    // =========================================================================
    // DUT Interface Signals
    // =========================================================================
    
    logic [N-1:0][W-1:0] idle_count;
    logic [N-1:0][W-1:0] idle_base_th;
    logic [N-1:0]        recent_activity;
    logic [3:0]          alpha;
    logic [N-1:0]        sleep_eligible;
    
    // =========================================================================
    // Testbench Variables
    // =========================================================================
    
    int test_count;
    int pass_count;
    int fail_count;
    
    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    
    idle_predictor #(
        .N(N),
        .W(W)
    ) dut (
        .clk(clock),
        .rst_n(reset),
        .idle_count(idle_count),
        .idle_base_th(idle_base_th),
        .recent_activity(recent_activity),
        .alpha(alpha),
        .sleep_eligible(sleep_eligible)
    );
    
    // =========================================================================
    // Clock Generation (100MHz = 10ns period)
    // =========================================================================
    
    initial begin
        clock = 0;
        forever #5 clock = ~clock;
    end
    
    // =========================================================================
    // Golden Reference Model
    // =========================================================================
    // Mirrors the RTL adaptive threshold computation exactly
    // Formula: adaptive_th = base_th + (recent_activity ? (base_th >> alpha) : 0)
    // =========================================================================
    
    function automatic logic [N-1:0] golden_model(
        input logic [N-1:0][W-1:0] idle_cnt,
        input logic [N-1:0][W-1:0] base_th,
        input logic [N-1:0]        recent_act,
        input logic [3:0]          alph
    );
        logic [N-1:0][W-1:0] adaptive_th;
        logic [N-1:0]        expected_eligible;
        logic [W-1:0]        adjustment;
        logic [W:0]          sum_extended;
        logic                overflow;
        
        for (int i = 0; i < N; i++) begin
            // Compute adjustment based on recent activity
            // Formula: adaptive_th = base_th + (recent_activity ? (base_th >> alpha) : 0)
            if (recent_act[i]) begin
                adjustment = base_th[i] >> alph;
            end else begin
                adjustment = '0;
            end
            
            // Perform addition with overflow detection
            sum_extended = {1'b0, base_th[i]} + {1'b0, adjustment};
            overflow = sum_extended[W];
            
            // Apply saturation on overflow
            if (overflow) begin
                adaptive_th[i] = {W{1'b1}};  // Saturate to max value
            end else begin
                adaptive_th[i] = sum_extended[W-1:0];
            end
            
            // Check eligibility
            expected_eligible[i] = (idle_cnt[i] >= adaptive_th[i]);
        end
        
        return expected_eligible;
    endfunction
    
    // =========================================================================
    // Checker Task
    // =========================================================================
    // Compares DUT output against golden model and reports results
    // =========================================================================
    
    task automatic check_output(
        input string test_name,
        input int peripheral_id
    );
        logic [N-1:0] expected;
        logic match;
        
        // Wait one cycle for registered output
        @(posedge clock);
        #1; // Small delta for signal stability
        
        expected = golden_model(idle_count, idle_base_th, recent_activity, alpha);
        match = (sleep_eligible == expected);
        
        test_count++;
        
        if (match) begin
            pass_count++;
            $display("LOG: %0t : INFO : tb_idle_predictor : dut.sleep_eligible[%0d] : expected_value: %b actual_value: %b - PASS: %s",
                     $time, peripheral_id, expected[peripheral_id], sleep_eligible[peripheral_id], test_name);
        end else begin
            fail_count++;
            $display("LOG: %0t : ERROR : tb_idle_predictor : dut.sleep_eligible : expected_value: %b actual_value: %b",
                     $time, expected, sleep_eligible);
            $display("ERROR: Test '%s' FAILED", test_name);
            $display("  idle_count      = %p", idle_count);
            $display("  idle_base_th    = %p", idle_base_th);
            $display("  recent_activity = %b", recent_activity);
            $display("  alpha           = %0d", alpha);
            $display("  expected        = %b", expected);
            $display("  actual          = %b", sleep_eligible);
            $fatal(1, "Mismatch detected - terminating simulation");
        end
    endtask
    
    // =========================================================================
    // Initialize Inputs Task
    // =========================================================================
    
    task init_inputs();
        idle_count = '0;
        idle_base_th = '0;
        recent_activity = '0;
        alpha = '0;
    endtask
    
    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    
    initial begin
        $display("TEST START");
        $display("=========================================================================");
        $display("Testbench: tb_idle_predictor");
        $display("DUT: idle_predictor (N=%0d, W=%0d)", N, W);
        $display("=========================================================================");
        
        test_count = 0;
        pass_count = 0;
        fail_count = 0;
        
        // =====================================================================
        // Reset Sequence - Robust Reset Test
        // =====================================================================
        $display("\n[PHASE 1] Reset Test");
        $display("---------------------------------------------------------------------");
        $display("Applying reset with safe input values...");
        
        // Drive all inputs to safe, known values during reset
        // This ensures we don't assume uninitialized state behavior
        for (int i = 0; i < N; i++) begin
            idle_count[i] = '0;           // No idle time accumulated
            idle_base_th[i] = '1;         // Maximum threshold value
        end
        recent_activity = '0;             // No recent activity
        alpha = 4'h0;                     // Alpha = 0
        
        // Assert reset for 3 clock cycles
        reset = 0;
        repeat(3) @(posedge clock);
        
        // Deassert reset
        reset = 1;
        $display("Reset deasserted at time %0t", $time);
        
        // Wait for registered output to update (1-2 clock cycles)
        @(posedge clock);
        @(posedge clock);
        #1; // Small delta for signal stability
        
        // Check that all outputs are cleared after reset
        // With idle_count=0 and idle_base_th=max, sleep_eligible should be 0
        $display("Checking reset behavior:");
        $display("  idle_count      = %p (all zeros)", idle_count);
        $display("  idle_base_th    = %p (all max)", idle_base_th);
        $display("  recent_activity = %b (all zeros)", recent_activity);
        $display("  alpha           = %0d", alpha);
        $display("  sleep_eligible  = %b (checking for all zeros)", sleep_eligible);
        
        if (sleep_eligible == '0) begin
            $display("LOG: %0t : INFO : tb_idle_predictor : dut.sleep_eligible : expected_value: 0 actual_value: %b - PASS: Reset clears outputs",
                     $time, sleep_eligible);
            pass_count++;
            test_count++;
        end else begin
            $display("LOG: %0t : ERROR : tb_idle_predictor : dut.sleep_eligible : expected_value: 0 actual_value: %b",
                     $time, sleep_eligible);
            fail_count++;
            test_count++;
            $fatal(1, "Reset test failed - outputs not cleared");
        end
        
        // =====================================================================
        // Directed Test Cases
        // =====================================================================
        $display("\n[PHASE 2] Directed Test Cases");
        $display("---------------------------------------------------------------------");
        
        // ---------------------------------------------------------------------
        // Case A: Below Threshold
        // ---------------------------------------------------------------------
        $display("\n--- Test Case A: Below Threshold ---");
        init_inputs();
        idle_base_th[0] = 20;
        idle_count[0] = 19;
        recent_activity[0] = 0;
        alpha = 2;
        check_output("Case A: Below Threshold", 0);
        
        // ---------------------------------------------------------------------
        // Case B: At Threshold
        // ---------------------------------------------------------------------
        $display("\n--- Test Case B: At Threshold ---");
        init_inputs();
        idle_base_th[0] = 20;
        idle_count[0] = 20;
        recent_activity[0] = 0;
        alpha = 2;
        check_output("Case B: At Threshold", 0);
        
        // ---------------------------------------------------------------------
        // Case C: Recent Activity Increases Threshold
        // ---------------------------------------------------------------------
        $display("\n--- Test Case C: Recent Activity Increases Threshold ---");
        init_inputs();
        idle_base_th[0] = 20;
        idle_count[0] = 24;  // Would pass base threshold
        recent_activity[0] = 1;  // But recent activity increases threshold
        alpha = 2;  // adaptive_th = 20 + (20>>2) = 20 + 5 = 25
        // idle_count=24 < adaptive_th=25, so should NOT be eligible
        check_output("Case C: Recent Activity Blocks Sleep", 0);
        
        // ---------------------------------------------------------------------
        // Case D: No Recent Activity (Threshold = Base)
        // ---------------------------------------------------------------------
        $display("\n--- Test Case D: No Recent Activity (Threshold = Base) ---");
        init_inputs();
        idle_base_th[0] = 20;
        idle_count[0] = 20;  // Equal to base threshold
        recent_activity[0] = 0;  // No activity: adaptive_th = base_th + 0 = 20
        alpha = 2;  // adaptive_th = 20 + 0 = 20
        // idle_count=20 >= adaptive_th=20, so SHOULD be eligible
        check_output("Case D: No Activity Keeps Base Threshold", 0);
        
        // ---------------------------------------------------------------------
        // Multi-Peripheral Test
        // ---------------------------------------------------------------------
        $display("\n--- Test Case: Multi-Peripheral ---");
        init_inputs();
        // Peripheral 0: eligible
        idle_base_th[0] = 10;
        idle_count[0] = 15;
        recent_activity[0] = 0;
        
        // Peripheral 1: not eligible (below threshold)
        idle_base_th[1] = 30;
        idle_count[1] = 20;
        recent_activity[1] = 0;
        
        // Peripheral 2: eligible (at threshold)
        idle_base_th[2] = 25;
        idle_count[2] = 25;
        recent_activity[2] = 0;
        
        // Peripheral 3: not eligible (recent activity)
        idle_base_th[3] = 10;
        idle_count[3] = 12;
        recent_activity[3] = 1;  // adaptive_th will be > 12
        
        alpha = 1;
        check_output("Multi-Peripheral Test", 0);
        
        // =====================================================================
        // Random Testing
        // =====================================================================
        $display("\n[PHASE 3] Random Testing (200+ trials)");
        $display("---------------------------------------------------------------------");
        
        for (int trial = 0; trial < 200; trial++) begin
            init_inputs();
            
            // Randomize inputs
            for (int i = 0; i < N; i++) begin
                idle_count[i] = $urandom_range(0, (1 << W) - 1);
                idle_base_th[i] = $urandom_range(1, (1 << (W-1)) - 1);
                recent_activity[i] = $urandom_range(0, 1);
            end
            alpha = $urandom_range(0, 15);
            
            check_output($sformatf("Random Trial %0d", trial), 0);
            
            if ((trial + 1) % 50 == 0) begin
                $display("  Completed %0d/%0d random trials...", trial + 1, 200);
            end
        end
        
        // =====================================================================
        // Edge Cases
        // =====================================================================
        $display("\n[PHASE 4] Edge Cases");
        $display("---------------------------------------------------------------------");
        
        // Edge case: alpha = 0 (maximum adaptation)
        $display("\n--- Edge Case: Alpha = 0 (Max Adaptation) ---");
        init_inputs();
        idle_base_th[0] = 100;
        idle_count[0] = 150;
        recent_activity[0] = 1;
        alpha = 0;  // adaptive_th = 100 + (100>>0) = 200
        check_output("Alpha=0 Max Adaptation", 0);
        
        // Edge case: alpha = 15 (minimum adaptation)
        $display("\n--- Edge Case: Alpha = 15 (Min Adaptation) ---");
        init_inputs();
        idle_base_th[0] = 100;
        idle_count[0] = 100;
        recent_activity[0] = 1;
        alpha = 15;  // adaptive_th = 100 + (100>>15) = ~100
        check_output("Alpha=15 Min Adaptation", 0);
        
        // Edge case: zero base threshold
        $display("\n--- Edge Case: Zero Base Threshold ---");
        init_inputs();
        idle_base_th[0] = 0;
        idle_count[0] = 0;
        recent_activity[0] = 0;
        alpha = 2;
        check_output("Zero Base Threshold", 0);
        
        // Edge case: maximum values
        $display("\n--- Edge Case: Maximum Values ---");
        init_inputs();
        idle_base_th[0] = (1 << W) - 1;
        idle_count[0] = (1 << W) - 1;
        recent_activity[0] = 1;
        alpha = 1;
        check_output("Maximum Values", 0);
        
        // =====================================================================
        // Test Summary
        // =====================================================================
        $display("\n=========================================================================");
        $display("TEST SUMMARY");
        $display("=========================================================================");
        $display("Total Tests:  %0d", test_count);
        $display("Passed:       %0d", pass_count);
        $display("Failed:       %0d", fail_count);
        $display("=========================================================================");
        
        if (fail_count == 0) begin
            $display("TEST PASSED");
        end else begin
            $display("TEST FAILED");
            $error("Simulation completed with %0d failures", fail_count);
        end
        
        $finish;
    end
    
    // =========================================================================
    // Timeout Watchdog
    // =========================================================================
    
    initial begin
        #100000; // 100us timeout
        $display("ERROR: Simulation timeout - test did not complete");
        $fatal(1, "Timeout watchdog triggered");
    end
    
    // =========================================================================
    // Waveform Dump
    // =========================================================================
    
    initial begin
        $dumpfile("dumpfile.fst");
        $dumpvars(0);
    end

endmodule : tb_idle_predictor
