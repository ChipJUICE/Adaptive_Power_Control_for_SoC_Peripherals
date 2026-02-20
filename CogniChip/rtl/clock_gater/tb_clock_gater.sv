`timescale 1ns/1ps

// =============================================================================
// Testbench: tb_clock_gater
// Description: Comprehensive testbench for clock_gater module with explicit
//              verification of critical clock gating corner cases:
//              1. Baseline operation
//              2. Glitch-free gating (disable at posedge)
//              3. Wake-up test (enable during high phase)
//              4. Scan mode override
// =============================================================================

module tb_clock_gater;

    // =========================================================================
    // Parameters and Constants
    // =========================================================================
    parameter int N = 1;                    // Test single peripheral
    parameter real CLK_PERIOD = 10.0;       // 100MHz clock (10ns period)
    parameter real CLK_HIGH_TIME = 5.0;     // 5ns high phase
    parameter real CLK_LOW_TIME = 5.0;      // 5ns low phase
    parameter real GLITCH_THRESHOLD = 2.0;  // Min valid pulse width (2ns)
    
    // =========================================================================
    // DUT Signals
    // =========================================================================
    logic           clk_in;
    logic           rst_n;
    logic           scan_en;
    logic [N-1:0]   clk_req;
    logic [N-1:0]   gclk_out;
    
    // =========================================================================
    // Testbench Variables
    // =========================================================================
    int test_passed;
    int test_failed;
    realtime last_gclk_edge;
    realtime pulse_width;
    
    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    clock_gater #(
        .N(N)
    ) dut (
        .clk_in(clk_in),
        .rst_n(rst_n),
        .scan_en(scan_en),
        .clk_req(clk_req),
        .gclk_out(gclk_out)
    );
    
    // =========================================================================
    // Clock Generator (100MHz)
    // =========================================================================
    initial begin
        clk_in = 0;
        forever #(CLK_PERIOD/2) clk_in = ~clk_in;
    end
    
    // =========================================================================
    // Glitch Detection Monitor
    // =========================================================================
    // Monitors gclk_out for glitches (pulses shorter than threshold)
    always @(gclk_out[0]) begin
        realtime current_time;
        current_time = $realtime;
        
        if (gclk_out[0] == 1'b1) begin
            // Rising edge - record time
            last_gclk_edge = current_time;
        end else if (gclk_out[0] == 1'b0 && last_gclk_edge > 0) begin
            // Falling edge - check pulse width
            pulse_width = current_time - last_gclk_edge;
            if (pulse_width < GLITCH_THRESHOLD && pulse_width > 0.1) begin
                $display("LOG: %0t : ERROR : tb_clock_gater : dut.gclk_out[0] : expected_value: no_glitch actual_value: glitch_detected (pulse_width=%.3fns)", 
                         $time, pulse_width);
                test_failed++;
            end
        end
    end
    
    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        // Declare automatic variables at the beginning
        automatic int match_count = 0;
        automatic realtime test2_start = 0;
        automatic int glitch_detected_test2 = 0;
        automatic int stayed_low = 1;
        automatic int scan_toggle_count = 0;
        
        // Initialize
        $display("TEST START");
        $display("=============================================================================");
        $display("Clock Gating Testbench - Comprehensive Corner Case Verification");
        $display("=============================================================================");
        test_passed = 0;
        test_failed = 0;
        last_gclk_edge = 0;
        
        // Initialize signals
        rst_n = 0;
        scan_en = 0;
        clk_req = '0;
        
        // Apply reset
        $display("\n[%0t] Applying Reset...", $time);
        repeat(3) @(posedge clk_in);
        rst_n = 1;
        @(posedge clk_in);
        $display("[%0t] Reset Released", $time);
        
        // =====================================================================
        // TEST CASE 1: Baseline Operation
        // =====================================================================
        $display("\n=============================================================================");
        $display("TEST 1: Baseline Operation - Verify gclk_out matches clk_in when enabled");
        $display("=============================================================================");
        
        @(negedge clk_in);  // Align to falling edge
        clk_req[0] = 1;
        $display("[%0t] Setting clk_req[0] = 1", $time);
        
        // Wait for enable to propagate through latch (should happen in next cycle)
        @(posedge clk_in);
        @(posedge clk_in);
        
        // Check that gclk_out toggles with clk_in for several cycles
        match_count = 0;
        for (int i = 0; i < 5; i++) begin
            @(posedge clk_in);
            #0.1;  // Small delta delay
            if (gclk_out[0] == 1'b1) begin
                match_count++;
                $display("LOG: %0t : INFO : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b1 actual_value: 1'b1", $time);
            end else begin
                $display("LOG: %0t : ERROR : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b1 actual_value: 1'b0", $time);
                test_failed++;
            end
            @(negedge clk_in);
            #0.1;
            if (gclk_out[0] == 1'b0) begin
                match_count++;
            end
        end
        
        if (match_count == 10) begin
            $display("[%0t] TEST 1 PASSED: gclk_out correctly follows clk_in", $time);
            test_passed++;
        end else begin
            $display("[%0t] TEST 1 FAILED: gclk_out did not match clk_in", $time);
            test_failed++;
        end
        
        // =====================================================================
        // TEST CASE 2: Glitch-Free Gating
        // =====================================================================
        $display("\n=============================================================================");
        $display("TEST 2: Glitch-Free Gating - Toggle clk_req from 1 to 0 at posedge");
        $display("       Verify gclk_out completes current pulse without glitches");
        $display("=============================================================================");
        
        // Ensure we start with clock enabled and wait for stable operation
        clk_req[0] = 1;
        repeat(2) @(posedge clk_in);
        
        // Record the test start time
        test2_start = $realtime;
        glitch_detected_test2 = 0;
        
        // Toggle clk_req from 1 to 0 exactly at posedge
        @(posedge clk_in);
        $display("[%0t] Disabling clk_req[0] at posedge clk_in", $time);
        clk_req[0] = 0;
        
        // The gated clock should:
        // 1. Complete the current high pulse (it's high right now)
        // 2. Go low at the natural falling edge
        // 3. Stay low thereafter
        
        #0.1;  // Small delay to check current state
        if (gclk_out[0] !== 1'b1) begin
            $display("LOG: %0t : ERROR : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b1 (completing pulse) actual_value: 1'b%0b", $time, gclk_out[0]);
            test_failed++;
            glitch_detected_test2 = 1;
        end else begin
            $display("LOG: %0t : INFO : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b1 (completing pulse) actual_value: 1'b1", $time);
        end
        
        // Wait for falling edge
        @(negedge clk_in);
        #0.1;
        if (gclk_out[0] !== 1'b0) begin
            $display("LOG: %0t : ERROR : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b0 (after disable) actual_value: 1'b%0b", $time, gclk_out[0]);
            test_failed++;
        end else begin
            $display("LOG: %0t : INFO : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b0 (after disable) actual_value: 1'b0", $time);
        end
        
        // Verify gclk_out stays low for several cycles
        stayed_low = 1;
        for (int i = 0; i < 4; i++) begin
            @(posedge clk_in);
            #0.1;
            if (gclk_out[0] !== 1'b0) begin
                stayed_low = 0;
                $display("LOG: %0t : ERROR : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b0 (gated) actual_value: 1'b%0b", $time, gclk_out[0]);
                test_failed++;
            end
        end
        
        if (stayed_low && !glitch_detected_test2) begin
            $display("[%0t] TEST 2 PASSED: Clock gated cleanly without glitches", $time);
            test_passed++;
        end else begin
            $display("[%0t] TEST 2 FAILED: Glitch or incorrect gating behavior detected", $time);
            test_failed++;
        end
        
        // =====================================================================
        // TEST CASE 3: Wake-up Test
        // =====================================================================
        $display("\n=============================================================================");
        $display("TEST 3: Wake-up Test - Enable clk_req during high phase of clk_in");
        $display("       Verify no partial pulse (gclk_out waits for next full cycle)");
        $display("=============================================================================");
        
        // Ensure clock is disabled
        clk_req[0] = 0;
        repeat(2) @(posedge clk_in);
        
        // Wait for middle of high phase
        @(posedge clk_in);
        #2.5;  // 2.5ns into the 5ns high phase
        $display("[%0t] Enabling clk_req[0] during HIGH phase of clk_in", $time);
        clk_req[0] = 1;
        
        // At this point, gclk_out should remain LOW (no partial pulse)
        #2.0;  // Check for remaining high phase
        if (gclk_out[0] !== 1'b0) begin
            $display("LOG: %0t : ERROR : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b0 (no partial pulse) actual_value: 1'b%0b", $time, gclk_out[0]);
            test_failed++;
        end else begin
            $display("LOG: %0t : INFO : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b0 (no partial pulse) actual_value: 1'b0", $time);
        end
        
        // Wait for falling edge and low phase
        wait (clk_in == 0);
        #(CLK_LOW_TIME - 0.1);
        
        // Now wait for next rising edge - this is when gclk_out should start
        @(posedge clk_in);
        #0.1;
        if (gclk_out[0] !== 1'b1) begin
            $display("LOG: %0t : ERROR : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b1 (full cycle start) actual_value: 1'b%0b", $time, gclk_out[0]);
            test_failed++;
        end else begin
            $display("LOG: %0t : INFO : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b1 (full cycle start) actual_value: 1'b1", $time);
            $display("[%0t] TEST 3 PASSED: No partial pulse, clock started on next full cycle", $time);
            test_passed++;
        end
        
        // =====================================================================
        // TEST CASE 4: Scan Mode Override
        // =====================================================================
        $display("\n=============================================================================");
        $display("TEST 4: Scan Mode Override - Verify scan_en bypasses clock gating");
        $display("=============================================================================");
        
        // Disable clock request but enable scan mode
        @(negedge clk_in);
        clk_req[0] = 0;
        scan_en = 1;
        $display("[%0t] Setting clk_req[0]=0, scan_en=1", $time);
        
        // Wait for scan enable to propagate
        @(posedge clk_in);
        @(posedge clk_in);
        
        // Verify gclk_out toggles despite clk_req being 0
        scan_toggle_count = 0;
        for (int i = 0; i < 5; i++) begin
            @(posedge clk_in);
            #0.1;
            if (gclk_out[0] == 1'b1) begin
                scan_toggle_count++;
                $display("LOG: %0t : INFO : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b1 (scan bypass) actual_value: 1'b1", $time);
            end else begin
                $display("LOG: %0t : ERROR : tb_clock_gater : dut.gclk_out[0] : expected_value: 1'b1 (scan bypass) actual_value: 1'b0", $time);
                test_failed++;
            end
            @(negedge clk_in);
            #0.1;
            if (gclk_out[0] == 1'b0) begin
                scan_toggle_count++;
            end
        end
        
        if (scan_toggle_count == 10) begin
            $display("[%0t] TEST 4 PASSED: Scan mode successfully bypasses clock gating", $time);
            test_passed++;
        end else begin
            $display("[%0t] TEST 4 FAILED: Scan mode did not bypass clock gating correctly", $time);
            test_failed++;
        end
        
        // =====================================================================
        // Test Summary
        // =====================================================================
        scan_en = 0;
        clk_req = '0;
        repeat(2) @(posedge clk_in);
        
        $display("\n=============================================================================");
        $display("Test Summary");
        $display("=============================================================================");
        $display("Tests Passed: %0d", test_passed);
        $display("Tests Failed: %0d", test_failed);
        $display("=============================================================================");
        
        if (test_failed == 0) begin
            $display("TEST PASSED");
        end else begin
            $display("TEST FAILED");
            $error("One or more test cases failed");
        end
        
        #100;
        $finish;
    end
    
    // =========================================================================
    // Timeout Watchdog
    // =========================================================================
    initial begin
        #50000;  // 50us timeout
        $display("\n=============================================================================");
        $display("ERROR: Simulation timeout!");
        $display("=============================================================================");
        $display("TEST FAILED");
        $fatal(1, "Testbench timeout - simulation ran too long");
    end
    
    // =========================================================================
    // Waveform Dump
    // =========================================================================
    initial begin
        $dumpfile("dumpfile.fst");
        $dumpvars(0);
    end

endmodule : tb_clock_gater
