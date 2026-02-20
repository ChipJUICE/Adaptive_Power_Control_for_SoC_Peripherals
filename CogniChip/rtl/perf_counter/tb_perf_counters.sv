// ==============================================================================
// tb_perf_counters.sv
// Self-checking testbench for perf_counters module
// ==============================================================================

module tb_perf_counters;

    // Parameters
    localparam int N  = 4;
    localparam int CW = 32;
    
    // State encoding
    localparam logic [1:0] ACTIVE = 2'b00;
    localparam logic [1:0] IDLE   = 2'b01;
    localparam logic [1:0] SLEEP  = 2'b10;
    
    // Clock period: 100MHz = 10ns
    localparam time CLK_PERIOD = 10ns;
    
    // DUT signals
    logic                    clk;
    logic                    rst_n;
    logic [N-1:0][1:0]       state;
    logic [N-1:0][CW-1:0]    sleep_count;
    logic [N-1:0][CW-1:0]    active_cycles;
    logic [N-1:0][CW-1:0]    idle_cycles;
    
    // Golden reference model
    logic [CW-1:0] exp_sleep_count[N];
    logic [CW-1:0] exp_active_cycles[N];
    logic [CW-1:0] exp_idle_cycles[N];
    logic [1:0]    prev_state_ref[N];
    
    // Baseline counters for relative checking
    logic [CW-1:0] base_sleep_count[N];
    logic [CW-1:0] base_active_cycles[N];
    logic [CW-1:0] base_idle_cycles[N];
    
    // Test control
    int cycle_count;
    int error_count;
    
    // DUT instantiation
    perf_counters #(
        .N  (N),
        .CW (CW)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .state        (state),
        .sleep_count  (sleep_count),
        .active_cycles(active_cycles),
        .idle_cycles  (idle_cycles)
    );
    
    // Clock generation: 100MHz (10ns period)
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    // Golden reference model - mirrors DUT behavior
    always @(posedge clk) begin
        if (!rst_n) begin
            // Reset reference model
            for (int i = 0; i < N; i++) begin
                exp_sleep_count[i]   = '0;
                exp_active_cycles[i] = '0;
                exp_idle_cycles[i]   = '0;
                prev_state_ref[i]    = ACTIVE;
            end
        end else begin
            // Update reference counters
            for (int i = 0; i < N; i++) begin
                // Cycle counting
                if (state[i] == ACTIVE && exp_active_cycles[i] != '1) begin
                    exp_active_cycles[i] = exp_active_cycles[i] + 1;
                end
                
                if (state[i] == IDLE && exp_idle_cycles[i] != '1) begin
                    exp_idle_cycles[i] = exp_idle_cycles[i] + 1;
                end
                
                // Sleep entry detection
                if ((prev_state_ref[i] != SLEEP) && (state[i] == SLEEP) && (exp_sleep_count[i] != '1)) begin
                    exp_sleep_count[i] = exp_sleep_count[i] + 1;
                end
                
                // Update state history
                prev_state_ref[i] = state[i];
            end
        end
    end
    
    // Checker: Compare DUT outputs vs expected after every rising edge
    always @(posedge clk) begin
        if (rst_n) begin  // Only check when not in reset
            #1;  // Small delay after clock edge to allow outputs to settle
            for (int i = 0; i < N; i++) begin
                if (sleep_count[i] !== exp_sleep_count[i] ||
                    active_cycles[i] !== exp_active_cycles[i] ||
                    idle_cycles[i] !== exp_idle_cycles[i]) begin
                    
                    $display("LOG: %0t : ERROR : tb_perf_counters : dut.perf_counters[%0d] : expected_value: sleep=%0d,active=%0d,idle=%0d actual_value: sleep=%0d,active=%0d,idle=%0d",
                             $time, i, 
                             exp_sleep_count[i], exp_active_cycles[i], exp_idle_cycles[i],
                             sleep_count[i], active_cycles[i], idle_cycles[i]);
                    $display("ERROR: Cycle %0d, Peripheral %0d, State=%b", cycle_count, i, state[i]);
                    $display("  Expected: sleep_count=%0d, active_cycles=%0d, idle_cycles=%0d",
                             exp_sleep_count[i], exp_active_cycles[i], exp_idle_cycles[i]);
                    $display("  Actual:   sleep_count=%0d, active_cycles=%0d, idle_cycles=%0d",
                             sleep_count[i], active_cycles[i], idle_cycles[i]);
                    error_count++;
                    $fatal(1, "Counter mismatch detected!");
                end
            end
        end
    end
    
    // Cycle counter
    always @(posedge clk) begin
        if (!rst_n) begin
            cycle_count <= 0;
        end else begin
            cycle_count <= cycle_count + 1;
        end
    end
    
    // Task: Apply synchronous active-low reset
    task do_reset();
        begin
            $display("[%0t] Applying reset...", $time);
            state = '0;  // Set all peripherals to ACTIVE
            rst_n = 0;   // Assert reset
            repeat(3) @(posedge clk);
            rst_n = 1;   // Deassert reset
            $display("[%0t] Reset released", $time);
            // Wait 1 rising edge to let design stabilize
            @(posedge clk);
            $display("[%0t] Post-reset: ready for testing", $time);
        end
    endtask
    
    // Task: Step exactly K clock cycles
    task step_cycles(input int K);
        begin
            repeat(K) @(posedge clk);
        end
    endtask
    
    // Task: Capture baseline counters
    task capture_baseline();
        begin
            #1;  // Small delay to sample after clock edge
            for (int i = 0; i < N; i++) begin
                base_sleep_count[i]   = sleep_count[i];
                base_active_cycles[i] = active_cycles[i];
                base_idle_cycles[i]   = idle_cycles[i];
            end
            $display("[%0t] Baseline captured", $time);
        end
    endtask
    
    // Main test sequence
    initial begin
        $display("TEST START");
        $display("==============================================================================");
        $display("Testbench: tb_perf_counters");
        $display("DUT: perf_counters (N=%0d, CW=%0d)", N, CW);
        $display("==============================================================================");
        
        // Initialize
        error_count = 0;
        rst_n = 1;
        state = '0;  // All ACTIVE
        
        // Initial reset sequence with checks
        $display("[%0t] Initial reset sequence with validation...", $time);
        rst_n = 0;
        repeat(3) @(posedge clk);
        
        // Check all counters are zero during reset
        #1;
        for (int i = 0; i < N; i++) begin
            if (sleep_count[i] !== 0 || active_cycles[i] !== 0 || idle_cycles[i] !== 0) begin
                $display("ERROR: Counters not zero during reset for peripheral %0d", i);
                $fatal(1, "Reset check failed!");
            end
        end
        $display("[%0t] Reset check during reset: PASS - All counters zero", $time);
        
        // Release reset
        rst_n = 1;
        $display("[%0t] Reset released", $time);
        @(posedge clk);
        $display("[%0t] Initial reset validation complete", $time);
        
        // ======================================================================
        // Case A: ACTIVE counting
        // ======================================================================
        $display("\n[%0t] ===== Case A: ACTIVE counting =====", $time);
        do_reset();
        capture_baseline();
        
        // Apply stimulus: hold peripheral 0 in ACTIVE for 5 cycles
        state = '0;  // All ACTIVE
        step_cycles(5);
        #1;
        
        // Check: active_cycles[0] should have increased by 5
        $display("[%0t] Peripheral 0: active_cycles=%0d, baseline=%0d, delta=%0d (expected +5)",
                 $time, active_cycles[0], base_active_cycles[0], 
                 active_cycles[0] - base_active_cycles[0]);
        if ((active_cycles[0] - base_active_cycles[0]) !== 5) begin
            $fatal(1, "Case A failed!");
        end
        $display("[%0t] Case A: PASS", $time);
        
        // ======================================================================
        // Case B: IDLE counting
        // ======================================================================
        $display("\n[%0t] ===== Case B: IDLE counting =====", $time);
        do_reset();
        capture_baseline();
        
        // Apply stimulus: hold peripheral 1 in IDLE for 7 cycles
        state = '0;  // All ACTIVE initially
        state[1] = IDLE;
        step_cycles(7);
        #1;
        
        // Check: idle_cycles[1] should have increased by 7
        $display("[%0t] Peripheral 1: idle_cycles=%0d, baseline=%0d, delta=%0d (expected +7)",
                 $time, idle_cycles[1], base_idle_cycles[1],
                 idle_cycles[1] - base_idle_cycles[1]);
        if ((idle_cycles[1] - base_idle_cycles[1]) !== 7) begin
            $fatal(1, "Case B failed!");
        end
        $display("[%0t] Case B: PASS", $time);
        
        // ======================================================================
        // Case C: Sleep entry counting
        // ======================================================================
        $display("\n[%0t] ===== Case C: Sleep entry counting =====", $time);
        do_reset();
        capture_baseline();
        
        // Apply stimulus: Peripheral 2 goes ACTIVE → IDLE → SLEEP
        state = '0;  // All ACTIVE
        state[2] = ACTIVE;
        step_cycles(2);
        
        state[2] = IDLE;
        step_cycles(1);
        
        // Transition to SLEEP - should increment sleep_count by 1
        state[2] = SLEEP;
        step_cycles(1);
        #1;
        $display("[%0t] After entering SLEEP: sleep_count[2]=%0d, baseline=%0d, delta=%0d (expected +1)",
                 $time, sleep_count[2], base_sleep_count[2],
                 sleep_count[2] - base_sleep_count[2]);
        if ((sleep_count[2] - base_sleep_count[2]) !== 1) begin
            $fatal(1, "Case C failed - sleep_count should increment by 1 on entry!");
        end
        
        // Stay in SLEEP for 2 more cycles - sleep_count should NOT increment
        step_cycles(2);
        #1;
        $display("[%0t] After 3 total SLEEP cycles: sleep_count[2]=%0d, baseline=%0d, delta=%0d (should still be +1)",
                 $time, sleep_count[2], base_sleep_count[2],
                 sleep_count[2] - base_sleep_count[2]);
        if ((sleep_count[2] - base_sleep_count[2]) !== 1) begin
            $fatal(1, "Case C failed - sleep_count should not increment during SLEEP!");
        end
        $display("[%0t] Case C: PASS", $time);
        
        // ======================================================================
        // Case D: Multiple sleep entries
        // ======================================================================
        $display("\n[%0t] ===== Case D: Multiple sleep entries =====", $time);
        do_reset();
        capture_baseline();
        
        // Ensure state transitions are aligned to clock edges for clean sampling
        // Phase 1: Hold ACTIVE for 2 cycles
        state = '0;  // All ACTIVE
        state[2] = ACTIVE;
        step_cycles(2);
        $display("[%0t] Held ACTIVE for 2 cycles", $time);
        
        // Phase 2: First entry to SLEEP, hold for 2 cycles
        state[2] = SLEEP;
        step_cycles(2);
        #1;
        $display("[%0t] First entry to SLEEP: sleep_count[2]=%0d, baseline=%0d, delta=%0d (expected +1)",
                 $time, sleep_count[2], base_sleep_count[2],
                 sleep_count[2] - base_sleep_count[2]);
        if ((sleep_count[2] - base_sleep_count[2]) !== 1) begin
            $fatal(1, "Case D failed - sleep_count should increment by 1 after first entry!");
        end
        
        // Phase 3: Transition back to ACTIVE, hold for 2 cycles (ensures prev_state updates)
        state[2] = ACTIVE;
        step_cycles(2);
        $display("[%0t] Transitioned to ACTIVE, held for 2 cycles", $time);
        
        // Phase 4: Second entry to SLEEP, hold for 2 cycles
        state[2] = SLEEP;
        step_cycles(2);
        #1;
        $display("[%0t] Second entry to SLEEP: sleep_count[2]=%0d, baseline=%0d, delta=%0d (expected +2)",
                 $time, sleep_count[2], base_sleep_count[2],
                 sleep_count[2] - base_sleep_count[2]);
        if ((sleep_count[2] - base_sleep_count[2]) !== 2) begin
            $fatal(1, "Case D failed - sleep_count should increment by 2 after second entry!");
        end
        $display("[%0t] Case D: PASS", $time);
        
        // ======================================================================
        // Case E: Multi-peripheral independence
        // ======================================================================
        $display("\n[%0t] ===== Case E: Multi-peripheral independence =====", $time);
        do_reset();
        capture_baseline();
        
        // Drive different states simultaneously for 3 cycles
        state[0] = ACTIVE;
        state[1] = IDLE;
        state[2] = SLEEP;
        state[3] = ACTIVE;
        step_cycles(3);
        #1;
        
        $display("[%0t] Multi-peripheral test results:", $time);
        $display("  Peripheral 0 (ACTIVE): active_cycles delta=%0d (expected +3)", 
                 active_cycles[0] - base_active_cycles[0]);
        $display("  Peripheral 1 (IDLE):   idle_cycles delta=%0d (expected +3)", 
                 idle_cycles[1] - base_idle_cycles[1]);
        $display("  Peripheral 2 (SLEEP):  sleep_count delta=%0d (expected +1)", 
                 sleep_count[2] - base_sleep_count[2]);
        $display("  Peripheral 3 (ACTIVE): active_cycles delta=%0d (expected +3)", 
                 active_cycles[3] - base_active_cycles[3]);
        
        // Verify independent operation with relative checks
        if ((active_cycles[0] - base_active_cycles[0]) !== 3 || 
            (idle_cycles[1] - base_idle_cycles[1]) !== 3 || 
            (sleep_count[2] - base_sleep_count[2]) !== 1 || 
            (active_cycles[3] - base_active_cycles[3]) !== 3) begin
            $fatal(1, "Case E failed - counter deltas don't match expected values!");
        end
        $display("[%0t] Case E: PASS - Independent operation verified", $time);
        
        // ======================================================================
        // Random Testing: 200+ cycles
        // ======================================================================
        $display("\n[%0t] ===== Random Testing (200 cycles) =====", $time);
        for (int cycle = 0; cycle < 200; cycle++) begin
            // Randomize state for each peripheral
            for (int i = 0; i < N; i++) begin
                automatic int rand_state = $urandom_range(0, 2);
                case (rand_state)
                    0: state[i] = ACTIVE;
                    1: state[i] = IDLE;
                    2: state[i] = SLEEP;
                endcase
            end
            @(posedge clk);
            // Checker runs automatically via always block
        end
        #1;
        $display("[%0t] Random testing complete: 200 cycles executed", $time);
        $display("[%0t] Random Testing: PASS", $time);
        
        // ======================================================================
        // Test Complete
        // ======================================================================
        $display("\n==============================================================================");
        $display("All tests completed successfully!");
        $display("Total cycles executed: %0d", cycle_count);
        $display("Errors detected: %0d", error_count);
        $display("==============================================================================");
        $display("TEST PASSED");
        
        #20;
        $finish;
    end
    
    // Waveform dumping
    initial begin
        $dumpfile("dumpfile.fst");
        $dumpvars(0);
    end
    
    // Timeout watchdog
    initial begin
        #100us;
        $display("ERROR: Simulation timeout!");
        $fatal(1, "Watchdog timeout - simulation ran too long");
    end

endmodule
