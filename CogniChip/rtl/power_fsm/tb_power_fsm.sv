// =============================================================================
// Testbench: tb_power_fsm
// Description: Self-checking testbench for power_fsm module
// =============================================================================
// Verifies per-peripheral power state machine transitions with:
//   - Golden reference model for expected behavior
//   - Cycle-accurate checking with automatic error detection
//   - Directed tests for all state transitions and priority rules
//   - Random stress testing with 200+ cycles
// =============================================================================

module tb_power_fsm;

    // =========================================================================
    // Parameters and State Encoding
    // =========================================================================
    
    parameter int N = 4;
    parameter int CLK_PERIOD = 10;  // 100MHz clock (10ns period)
    
    // State encoding (must match DUT)
    localparam logic [1:0] ACTIVE = 2'b00;
    localparam logic [1:0] IDLE   = 2'b01;
    localparam logic [1:0] SLEEP  = 2'b10;
    
    // =========================================================================
    // DUT Interface Signals
    // =========================================================================
    
    logic                 clk;
    logic                 rst_n;
    logic [N-1:0]         sleep_eligible;
    logic [N-1:0]         wake_evt;
    logic [N-1:0]         periph_en;
    logic [N-1:0][1:0]    state;
    logic [N-1:0]         clk_req;
    
    // =========================================================================
    // Golden Reference Model Signals
    // =========================================================================
    
    logic [N-1:0][1:0]    expected_state;
    logic [N-1:0]         expected_clk_req;
    logic [N-1:0][1:0]    golden_state_current;
    logic [N-1:0][1:0]    golden_state_next;
    
    // =========================================================================
    // Testbench Control
    // =========================================================================
    
    int error_count = 0;
    int cycle_count = 0;
    
    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    
    power_fsm #(
        .N(N)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .sleep_eligible (sleep_eligible),
        .wake_evt       (wake_evt),
        .periph_en      (periph_en),
        .state          (state),
        .clk_req        (clk_req)
    );
    
    // =========================================================================
    // Clock Generation
    // =========================================================================
    // 100MHz clock (10ns period, 5ns half-period)
    // =========================================================================
    
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    // =========================================================================
    // Golden Reference Model
    // =========================================================================
    // Mirrors DUT FSM logic to compute expected outputs
    // =========================================================================
    
    // Update golden state on clock edge
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < N; i++) begin
                golden_state_current[i] <= ACTIVE;
            end
        end else begin
            golden_state_current <= golden_state_next;
        end
    end
    
    // Compute next state (combinational) - mirrors DUT logic exactly
    always_comb begin
        for (int i = 0; i < N; i++) begin
            // Default: hold current state
            golden_state_next[i] = golden_state_current[i];
            
            // Priority 1: Peripheral disabled → force SLEEP
            if (!periph_en[i]) begin
                golden_state_next[i] = SLEEP;
            end
            
            // Priority 2: Wake event when enabled → force ACTIVE
            else if (wake_evt[i]) begin
                golden_state_next[i] = ACTIVE;
            end
            
            // Priority 3: Normal state transitions
            else begin
                case (golden_state_current[i])
                    
                    ACTIVE: begin
                        if (sleep_eligible[i]) begin
                            golden_state_next[i] = IDLE;
                        end
                    end
                    
                    IDLE: begin
                        if (sleep_eligible[i]) begin
                            golden_state_next[i] = SLEEP;
                        end else begin
                            golden_state_next[i] = ACTIVE;
                        end
                    end
                    
                    SLEEP: begin
                        golden_state_next[i] = SLEEP;
                    end
                    
                    default: begin
                        golden_state_next[i] = ACTIVE;
                    end
                    
                endcase
            end
        end
    end
    
    // Compute expected outputs
    always_comb begin
        for (int i = 0; i < N; i++) begin
            // Expected state is the registered golden state
            expected_state[i] = golden_state_current[i];
            
            // Expected clock request
            if (!periph_en[i]) begin
                expected_clk_req[i] = 1'b0;
            end else begin
                expected_clk_req[i] = (golden_state_current[i] == ACTIVE) || 
                                      (golden_state_current[i] == IDLE);
            end
        end
    end
    
    // =========================================================================
    // Automatic Output Checker
    // =========================================================================
    // Compare DUT outputs vs golden reference after each clock edge
    // =========================================================================
    
    always @(posedge clk) begin
        if (rst_n) begin  // Only check when not in reset
            for (int i = 0; i < N; i++) begin
                // Check state
                if (state[i] !== expected_state[i]) begin
                    $display("LOG: %0t : ERROR : tb_power_fsm : dut.state[%0d] : expected_value: %b actual_value: %b", 
                             $time, i, expected_state[i], state[i]);
                    $display("ERROR: State mismatch at cycle %0d, peripheral %0d", cycle_count, i);
                    $display("  Inputs: periph_en=%b, wake_evt=%b, sleep_eligible=%b", 
                             periph_en[i], wake_evt[i], sleep_eligible[i]);
                    $display("  Expected state=%b, Actual state=%b", expected_state[i], state[i]);
                    $fatal(1, "State verification failed");
                end
                
                // Check clk_req
                if (clk_req[i] !== expected_clk_req[i]) begin
                    $display("LOG: %0t : ERROR : tb_power_fsm : dut.clk_req[%0d] : expected_value: %b actual_value: %b", 
                             $time, i, expected_clk_req[i], clk_req[i]);
                    $display("ERROR: Clock request mismatch at cycle %0d, peripheral %0d", cycle_count, i);
                    $display("  Inputs: periph_en=%b, state=%b", periph_en[i], state[i]);
                    $display("  Expected clk_req=%b, Actual clk_req=%b", expected_clk_req[i], clk_req[i]);
                    $fatal(1, "Clock request verification failed");
                end
            end
        end
    end
    
    // =========================================================================
    // Helper Tasks
    // =========================================================================
    
    // Wait for one clock cycle
    task wait_cycle(input int num_cycles = 1);
        repeat(num_cycles) @(posedge clk);
    endtask
    
    // Apply inputs and wait for next clock edge
    task apply_inputs(input logic [N-1:0] s_elig, input logic [N-1:0] w_evt, input logic [N-1:0] p_en);
        sleep_eligible = s_elig;
        wake_evt = w_evt;
        periph_en = p_en;
        @(posedge clk);
        cycle_count++;
    endtask
    
    // Check specific peripheral state
    task check_peripheral_state(input int periph_idx, input logic [1:0] exp_state, input logic exp_clk_req);
        if (state[periph_idx] !== exp_state) begin
            $display("ERROR: Peripheral %0d state mismatch: expected=%b, actual=%b", 
                     periph_idx, exp_state, state[periph_idx]);
            $fatal(1, "Manual state check failed");
        end
        if (clk_req[periph_idx] !== exp_clk_req) begin
            $display("ERROR: Peripheral %0d clk_req mismatch: expected=%b, actual=%b", 
                     periph_idx, exp_clk_req, clk_req[periph_idx]);
            $fatal(1, "Manual clk_req check failed");
        end
    endtask
    
    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    
    initial begin
        $display("TEST START");
        $display("========================================");
        $display("Power FSM Testbench");
        $display("Parameters: N=%0d peripherals", N);
        $display("========================================");
        
        // =====================================================================
        // Reset Sequence
        // =====================================================================
        
        $display("\n[%0t] Phase 1: Reset Sequence", $time);
        
        // Initialize inputs
        rst_n = 0;
        sleep_eligible = '0;
        wake_evt = '0;
        periph_en = '1;  // All peripherals enabled
        
        // Hold reset for 3 clock cycles
        repeat(3) @(posedge clk);
        
        // Release reset
        @(posedge clk);
        rst_n = 1;
        
        // Check reset state on next rising edge
        @(posedge clk);
        cycle_count++;
        
        $display("  Checking reset state...");
        for (int i = 0; i < N; i++) begin
            if (state[i] !== ACTIVE) begin
                $display("ERROR: After reset, peripheral %0d state=%b (expected ACTIVE)", i, state[i]);
                $fatal(1, "Reset check failed");
            end
            if (clk_req[i] !== 1'b1) begin
                $display("ERROR: After reset, peripheral %0d clk_req=%b (expected 1)", i, clk_req[i]);
                $fatal(1, "Reset check failed");
            end
        end
        $display("  ✓ All peripherals in ACTIVE with clk_req=1");
        
        // =====================================================================
        // Case A: Disable Override Test
        // =====================================================================
        
        $display("\n[%0t] Phase 2: Case A - Disable Override", $time);
        $display("  Testing peripheral 1 with periph_en=0");
        
        // Disable peripheral 1
        apply_inputs(4'b0000, 4'b0000, 4'b1101);  // periph_en[1]=0
        check_peripheral_state(1, SLEEP, 1'b0);
        $display("  ✓ Peripheral 1 forced to SLEEP, clk_req=0");
        
        // Toggle sleep_eligible and wake_evt for peripheral 1 (should stay SLEEP)
        for (int i = 0; i < 5; i++) begin
            apply_inputs(4'b0010, 4'b0010, 4'b1101);  // Try to wake peripheral 1
            check_peripheral_state(1, SLEEP, 1'b0);
        end
        $display("  ✓ Peripheral 1 stays SLEEP despite wake/sleep toggles");
        
        // Re-enable all peripherals
        apply_inputs(4'b0000, 4'b0000, 4'b1111);
        
        // =====================================================================
        // Case B: Wake Highest Priority
        // =====================================================================
        
        $display("\n[%0t] Phase 3: Case B - Wake Priority", $time);
        $display("  Putting peripheral 0 into SLEEP...");
        
        // Transition peripheral 0: ACTIVE → IDLE → SLEEP
        apply_inputs(4'b0001, 4'b0000, 4'b1111);  // sleep_eligible[0]=1
        check_peripheral_state(0, IDLE, 1'b1);
        $display("  ✓ Peripheral 0 transitioned to IDLE");
        
        apply_inputs(4'b0001, 4'b0000, 4'b1111);  // Keep sleep_eligible[0]=1
        check_peripheral_state(0, SLEEP, 1'b0);
        $display("  ✓ Peripheral 0 transitioned to SLEEP");
        
        // Assert wake event for peripheral 0
        apply_inputs(4'b0001, 4'b0001, 4'b1111);  // wake_evt[0]=1
        check_peripheral_state(0, ACTIVE, 1'b1);
        $display("  ✓ Wake event forced peripheral 0 back to ACTIVE");
        
        // Clear wake event
        apply_inputs(4'b0000, 4'b0000, 4'b1111);
        
        // =====================================================================
        // Case C: ACTIVE → IDLE Transition
        // =====================================================================
        
        $display("\n[%0t] Phase 4: Case C - ACTIVE→IDLE Transition", $time);
        
        // Ensure peripheral 2 is in ACTIVE (should be from reset)
        apply_inputs(4'b0000, 4'b0000, 4'b1111);
        check_peripheral_state(2, ACTIVE, 1'b1);
        
        // Assert sleep_eligible for peripheral 2
        apply_inputs(4'b0100, 4'b0000, 4'b1111);  // sleep_eligible[2]=1
        check_peripheral_state(2, IDLE, 1'b1);
        $display("  ✓ Peripheral 2 transitioned ACTIVE→IDLE");
        
        // =====================================================================
        // Case D: IDLE → SLEEP Transition
        // =====================================================================
        
        $display("\n[%0t] Phase 5: Case D - IDLE→SLEEP Transition", $time);
        
        // Keep sleep_eligible asserted (peripheral 2 should be in IDLE)
        apply_inputs(4'b0100, 4'b0000, 4'b1111);  // sleep_eligible[2]=1
        check_peripheral_state(2, SLEEP, 1'b0);
        $display("  ✓ Peripheral 2 transitioned IDLE→SLEEP");
        
        // =====================================================================
        // Case E: IDLE → ACTIVE When Not Eligible
        // =====================================================================
        
        $display("\n[%0t] Phase 6: Case E - IDLE→ACTIVE Transition", $time);
        
        // Put peripheral 3 into IDLE
        apply_inputs(4'b1000, 4'b0000, 4'b1111);  // sleep_eligible[3]=1
        check_peripheral_state(3, IDLE, 1'b1);
        $display("  ✓ Peripheral 3 in IDLE");
        
        // Deassert sleep_eligible for peripheral 3
        apply_inputs(4'b0000, 4'b0000, 4'b1111);  // sleep_eligible[3]=0
        check_peripheral_state(3, ACTIVE, 1'b1);
        $display("  ✓ Peripheral 3 transitioned IDLE→ACTIVE");
        
        // =====================================================================
        // Case F: Multi-Peripheral Independence
        // =====================================================================
        
        $display("\n[%0t] Phase 7: Case F - Multi-Peripheral Independence", $time);
        
        // Set different patterns for each peripheral simultaneously
        // Periph 0: disabled → SLEEP
        // Periph 1: wake event → ACTIVE
        // Periph 2: already in SLEEP, stay SLEEP
        // Periph 3: sleep_eligible → transition from current state
        
        apply_inputs(4'b1000, 4'b0010, 4'b1110);
        // periph_en: 1110 (0 disabled)
        // wake_evt:  0010 (1 has wake)
        // sleep_elig: 1000 (3 eligible)
        
        check_peripheral_state(0, SLEEP, 1'b0);   // Disabled
        check_peripheral_state(1, ACTIVE, 1'b1);  // Wake event
        check_peripheral_state(2, SLEEP, 1'b0);   // Still in SLEEP
        check_peripheral_state(3, IDLE, 1'b1);    // Transitioned to IDLE
        
        $display("  ✓ All peripherals behave independently");
        
        // Reset to known state
        apply_inputs(4'b0000, 4'b0000, 4'b1111);
        apply_inputs(4'b0000, 4'b0000, 4'b1111);
        
        // =====================================================================
        // Random Testing Phase
        // =====================================================================
        
        $display("\n[%0t] Phase 8: Random Stress Testing (200 cycles)", $time);
        
        for (int cyc = 0; cyc < 200; cyc++) begin
            // Randomize inputs
            sleep_eligible = $random;
            wake_evt = $random;
            periph_en = $random;
            
            @(posedge clk);
            cycle_count++;
            
            // Golden model and checker automatically validate outputs
            
            if (cyc % 50 == 0) begin
                $display("  Progress: %0d/200 cycles completed", cyc);
            end
        end
        
        $display("  ✓ Random testing completed: %0d cycles", cycle_count);
        
        // =====================================================================
        // Test Completion
        // =====================================================================
        
        $display("\n========================================");
        $display("All tests completed successfully!");
        $display("Total cycles simulated: %0d", cycle_count);
        $display("========================================");
        $display("TEST PASSED");
        
        $finish;
    end
    
    // =========================================================================
    // Timeout Watchdog
    // =========================================================================
    
    initial begin
        #100000;  // 100us timeout
        $display("ERROR: Simulation timeout!");
        $fatal(1, "Watchdog timeout");
    end
    
    // =========================================================================
    // Waveform Dump
    // =========================================================================
    
    initial begin
        $dumpfile("dumpfile.fst");
        $dumpvars(0);
    end

endmodule : tb_power_fsm
