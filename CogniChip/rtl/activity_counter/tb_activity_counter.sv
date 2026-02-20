//==============================================================================
// Testbench: tb_activity_counter
// Description: Comprehensive testbench for activity_counter module
//              Tests idle counting, activity detection, recent activity tracking,
//              enable control, and counter saturation
//==============================================================================

module tb_activity_counter;

    //==========================================================================
    // Parameters
    //==========================================================================
    localparam int N = 4;
    localparam int W = 16;
    localparam int ACTIVITY_WINDOW = 8;
    localparam int CLK_PERIOD = 10;  // 10ns = 100MHz

    //==========================================================================
    // DUT Signals
    //==========================================================================
    logic                    clock;
    logic                    reset;
    logic [N-1:0]            activity_pulse;
    logic [N-1:0]            periph_en;
    logic [N-1:0][W-1:0]     idle_count;
    logic [N-1:0]            recent_activity;

    //==========================================================================
    // Test Control
    //==========================================================================
    int error_count = 0;
    int test_count = 0;

    //==========================================================================
    // DUT Instantiation
    //==========================================================================
    activity_counter #(
        .N(N),
        .W(W),
        .ACTIVITY_WINDOW(ACTIVITY_WINDOW)
    ) dut (
        .clk(clock),
        .rst_n(reset),
        .activity_pulse(activity_pulse),
        .periph_en(periph_en),
        .idle_count(idle_count),
        .recent_activity(recent_activity)
    );

    //==========================================================================
    // Clock Generation
    //==========================================================================
    initial begin
        clock = 0;
        forever #(CLK_PERIOD/2) clock = ~clock;
    end

    //==========================================================================
    // Checker Tasks
    //==========================================================================
    task automatic check_idle_count(
        input int periph_id,
        input logic [W-1:0] expected,
        input string test_name
    );
        test_count++;
        if (idle_count[periph_id] !== expected) begin
            error_count++;
            $display("LOG: %0t : ERROR : tb_activity_counter : dut.idle_count[%0d] : expected_value: %0d actual_value: %0d",
                     $time, periph_id, expected, idle_count[periph_id]);
        end else begin
            $display("LOG: %0t : INFO : tb_activity_counter : dut.idle_count[%0d] : expected_value: %0d actual_value: %0d",
                     $time, periph_id, expected, idle_count[periph_id]);
        end
    endtask

    task automatic check_recent_activity(
        input int periph_id,
        input logic expected,
        input string test_name
    );
        test_count++;
        if (recent_activity[periph_id] !== expected) begin
            error_count++;
            $display("LOG: %0t : ERROR : tb_activity_counter : dut.recent_activity[%0d] : expected_value: %0b actual_value: %0b",
                     $time, periph_id, expected, recent_activity[periph_id]);
        end else begin
            $display("LOG: %0t : INFO : tb_activity_counter : dut.recent_activity[%0d] : expected_value: %0b actual_value: %0b",
                     $time, periph_id, expected, recent_activity[periph_id]);
        end
    endtask

    //==========================================================================
    // Main Test Sequence
    //==========================================================================
    initial begin
        // Local test variables
        automatic logic [W-1:0] MAX_COUNT = {W{1'b1}};
        
        $display("TEST START");
        $display("================================================================================");
        $display("Activity Counter Testbench");
        $display("Parameters: N=%0d, W=%0d, ACTIVITY_WINDOW=%0d", N, W, ACTIVITY_WINDOW);
        $display("================================================================================");

        // Initialize signals
        reset = 0;
        activity_pulse = '0;
        periph_en = '0;

        // Wait for reset
        repeat(5) @(posedge clock);
        reset = 1;
        @(posedge clock);

        //======================================================================
        // TEST 1: Reset Behavior
        //======================================================================
        $display("\n[TEST 1] Reset Behavior - All counters should be 0");
        @(posedge clock);
        for (int i = 0; i < N; i++) begin
            check_idle_count(i, 0, "Reset Check");
            check_recent_activity(i, 0, "Reset Recent Activity");
        end

        //======================================================================
        // TEST 2: Idle Counting (Peripheral 0)
        //======================================================================
        $display("\n[TEST 2] Idle Counting - Counter increments when enabled and idle");
        periph_en[0] = 1;
        
        for (int cycle = 1; cycle <= 10; cycle++) begin
            @(posedge clock);
            check_idle_count(0, cycle, "Idle Count");
        end

        //======================================================================
        // TEST 3: Activity Pulse Reset
        //======================================================================
        $display("\n[TEST 3] Activity Pulse Reset - Counter resets to 0 on activity");
        @(posedge clock);
        activity_pulse[0] = 1;
        @(posedge clock);
        activity_pulse[0] = 0;
        check_idle_count(0, 0, "Activity Reset");
        check_recent_activity(0, 1, "Recent Activity After Pulse");

        //======================================================================
        // TEST 4: Recent Activity Window Tracking
        //======================================================================
        $display("\n[TEST 4] Recent Activity Window - Flag stays high for ACTIVITY_WINDOW cycles");
        for (int cycle = 1; cycle <= ACTIVITY_WINDOW; cycle++) begin
            @(posedge clock);
            check_recent_activity(0, 1, "Recent Activity Window");
        end
        
        // After window expires, should go low
        @(posedge clock);
        check_recent_activity(0, 0, "Recent Activity Expired");

        //======================================================================
        // TEST 5: Disable Behavior
        //======================================================================
        $display("\n[TEST 5] Disable Behavior - Counter holds at 0 when disabled");
        periph_en[0] = 0;
        @(posedge clock);
        check_idle_count(0, 0, "Disabled - Count Stays 0");
        repeat(5) @(posedge clock);
        check_idle_count(0, 0, "Disabled - Still 0 After 5 Cycles");

        //======================================================================
        // TEST 6: Re-enable and Continue Counting
        //======================================================================
        $display("\n[TEST 6] Re-enable - Counter resumes from 0");
        periph_en[0] = 1;
        @(posedge clock);
        check_idle_count(0, 1, "Re-enabled");
        @(posedge clock);
        check_idle_count(0, 2, "Re-enabled");

        //======================================================================
        // TEST 7: Multiple Activity Pulses
        //======================================================================
        $display("\n[TEST 7] Multiple Activity Pulses - Each pulse resets counter");
        activity_pulse[0] = 1;
        @(posedge clock);
        activity_pulse[0] = 0;
        check_idle_count(0, 0, "First Pulse Reset");
        
        repeat(5) @(posedge clock);
        check_idle_count(0, 5, "Count After 5 Cycles");
        
        activity_pulse[0] = 1;
        @(posedge clock);
        activity_pulse[0] = 0;
        check_idle_count(0, 0, "Second Pulse Reset");

        //======================================================================
        // TEST 8: Multiple Peripherals Independent Operation
        //======================================================================
        $display("\n[TEST 8] Multiple Peripherals - Independent counting");
        periph_en = 4'b1111;  // Enable all peripherals
        
        // Peripheral 0: activity pulse
        activity_pulse[0] = 1;
        @(posedge clock);
        activity_pulse[0] = 0;
        
        // Wait 3 cycles
        repeat(3) @(posedge clock);
        
        check_idle_count(0, 3, "Periph 0 Count");
        check_idle_count(1, 4, "Periph 1 Count");
        check_idle_count(2, 4, "Periph 2 Count");
        check_idle_count(3, 4, "Periph 3 Count");
        
        // Peripheral 2: activity pulse
        activity_pulse[2] = 1;
        @(posedge clock);
        activity_pulse[2] = 0;
        @(posedge clock);
        
        check_idle_count(0, 5, "Periph 0 Still Counting");
        check_idle_count(2, 1, "Periph 2 Reset and Counting");

        //======================================================================
        // TEST 9: Counter Saturation
        //======================================================================
        $display("\n[TEST 9] Counter Saturation - Counter saturates at MAX_COUNT");
        
        // Force counter to near-max
        force dut.idle_counter[1] = 16'hFFFA;
        @(posedge clock);
        release dut.idle_counter[1];
        
        // Count up to saturation
        for (int i = 0; i < 10; i++) begin
            @(posedge clock);
            if (idle_count[1] == MAX_COUNT) begin
                $display("LOG: %0t : INFO : tb_activity_counter : dut.idle_count[1] : Saturated at MAX_COUNT", $time);
                break;
            end
        end
        
        // Verify it stays at MAX
        @(posedge clock);
        check_idle_count(1, MAX_COUNT, "Saturation");
        @(posedge clock);
        check_idle_count(1, MAX_COUNT, "Saturation");

        //======================================================================
        // TEST 10: Activity Window Edge Cases
        //======================================================================
        $display("\n[TEST 10] Activity Window Edge Cases");
        
        periph_en[3] = 1;
        activity_pulse[3] = 1;
        @(posedge clock);
        activity_pulse[3] = 0;
        check_recent_activity(3, 1, "Activity Window Start");
        
        // Pulse again before window expires
        repeat(3) @(posedge clock);
        activity_pulse[3] = 1;
        @(posedge clock);
        activity_pulse[3] = 0;
        check_recent_activity(3, 1, "Activity Window Reloaded");
        
        // Wait for full window
        repeat(ACTIVITY_WINDOW) @(posedge clock);
        @(posedge clock);
        check_recent_activity(3, 0, "Activity Window Expired");

        //======================================================================
        // TEST 11: Simultaneous Activity on All Peripherals
        //======================================================================
        $display("\n[TEST 11] Simultaneous Activity - All peripherals pulse together");
        periph_en = 4'b1111;
        activity_pulse = 4'b1111;
        @(posedge clock);
        activity_pulse = 4'b0000;
        
        for (int i = 0; i < N; i++) begin
            check_idle_count(i, 0, "Simultaneous Pulse");
            check_recent_activity(i, 1, "Simultaneous Recent Activity");
        end
        
        repeat(3) @(posedge clock);
        for (int i = 0; i < N; i++) begin
            check_idle_count(i, 3, "All Counting Together");
        end

        //======================================================================
        // TEST 12: Disable During Active Counting
        //======================================================================
        $display("\n[TEST 12] Disable During Counting - Counter freezes then resets");
        
        // Reset peripheral 0 counter first
        periph_en[0] = 1;
        activity_pulse[0] = 1;
        @(posedge clock);
        activity_pulse[0] = 0;
        
        // Count for 5 cycles
        repeat(5) @(posedge clock);
        check_idle_count(0, 5, "Before Disable");
        
        periph_en[0] = 0;
        @(posedge clock);
        check_idle_count(0, 0, "After Disable");
        check_recent_activity(0, 0, "Recent Activity Cleared");

        //======================================================================
        // Test Summary
        //======================================================================
        $display("\n================================================================================");
        $display("Test Summary");
        $display("================================================================================");
        $display("Total Tests: %0d", test_count);
        $display("Errors: %0d", error_count);
        
        if (error_count == 0) begin
            $display("TEST PASSED");
        end else begin
            $display("TEST FAILED");
            $error("Test failed with %0d errors", error_count);
        end
        
        $display("================================================================================");
        
        // End simulation
        repeat(10) @(posedge clock);
        $finish(0);
    end

    //==========================================================================
    // Timeout Watchdog
    //==========================================================================
    initial begin
        #1000000;  // 1ms timeout
        $display("ERROR");
        $fatal(1, "Simulation timeout!");
    end

    //==========================================================================
    // Waveform Dump
    //==========================================================================
    initial begin
        $dumpfile("dumpfile.fst");
        $dumpvars(0);
    end

endmodule
