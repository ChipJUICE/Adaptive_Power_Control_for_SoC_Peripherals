//==============================================================================
// Testbench: tb_cfg_regs
// Description: Comprehensive testbench for cfg_regs module
//              Tests register read/write operations, address decoding,
//              interrupt generation, and edge cases
//==============================================================================

module tb_cfg_regs;

    //==========================================================================
    // Parameters
    //==========================================================================
    localparam int N = 4;
    localparam int W = 16;
    localparam int DATA_W = 32;
    localparam int ADDR_W = 8;
    localparam int CLK_PERIOD = 10;  // 10ns = 100MHz

    //==========================================================================
    // Register Addresses
    //==========================================================================
    localparam logic [ADDR_W-1:0] ADDR_PERIPH_EN    = 8'h00;
    localparam logic [ADDR_W-1:0] ADDR_ALPHA        = 8'h04;
    localparam logic [ADDR_W-1:0] ADDR_WAKE_MASK    = 8'h08;
    localparam logic [ADDR_W-1:0] ADDR_IRQ_MASK     = 8'h0C;
    localparam logic [ADDR_W-1:0] ADDR_IDLE_TH_BASE = 8'h10;
    localparam logic [ADDR_W-1:0] ADDR_STATE_BASE   = 8'h80;
    localparam logic [ADDR_W-1:0] ADDR_IRQ_STATUS   = 8'hC0;

    //==========================================================================
    // DUT Signals
    //==========================================================================
    logic                    clock;
    logic                    reset;
    logic [ADDR_W-1:0]       cfg_addr;
    logic [DATA_W-1:0]       cfg_wdata;
    logic                    cfg_we;
    logic                    cfg_re;
    logic [DATA_W-1:0]       cfg_rdata;
    logic [N-1:0][1:0]       state;
    logic [N-1:0]            periph_en;
    logic [N-1:0][W-1:0]     idle_base_th;
    logic [3:0]              alpha;
    logic [N-1:0]            wake_mask;
    logic                    irq;

    //==========================================================================
    // Test Control
    //==========================================================================
    int error_count = 0;
    int test_count = 0;

    //==========================================================================
    // DUT Instantiation
    //==========================================================================
    cfg_regs #(
        .N(N),
        .W(W),
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W)
    ) dut (
        .clk(clock),
        .rst_n(reset),
        .cfg_addr(cfg_addr),
        .cfg_wdata(cfg_wdata),
        .cfg_we(cfg_we),
        .cfg_re(cfg_re),
        .cfg_rdata(cfg_rdata),
        .state(state),
        .periph_en(periph_en),
        .idle_base_th(idle_base_th),
        .alpha(alpha),
        .wake_mask(wake_mask),
        .irq(irq)
    );

    //==========================================================================
    // Clock Generation
    //==========================================================================
    initial begin
        clock = 0;
        forever #(CLK_PERIOD/2) clock = ~clock;
    end

    //==========================================================================
    // Helper Tasks
    //==========================================================================
    task automatic write_reg(
        input logic [ADDR_W-1:0] addr,
        input logic [DATA_W-1:0] data
    );
        @(posedge clock);
        cfg_addr = addr;
        cfg_wdata = data;
        cfg_we = 1;
        cfg_re = 0;
        @(posedge clock);
        cfg_we = 0;
        @(posedge clock);
    endtask

    task automatic read_reg(
        input logic [ADDR_W-1:0] addr,
        output logic [DATA_W-1:0] data
    );
        @(posedge clock);
        cfg_addr = addr;
        cfg_we = 0;
        cfg_re = 1;
        @(posedge clock);
        cfg_re = 0;
        data = cfg_rdata;
        @(posedge clock);
    endtask

    task automatic check_read(
        input logic [ADDR_W-1:0] addr,
        input logic [DATA_W-1:0] expected,
        input string test_name
    );
        logic [DATA_W-1:0] read_data;
        test_count++;
        read_reg(addr, read_data);
        if (read_data !== expected) begin
            error_count++;
            $display("LOG: %0t : ERROR : tb_cfg_regs : dut.cfg_rdata : expected_value: 0x%08h actual_value: 0x%08h",
                     $time, expected, read_data);
        end else begin
            $display("LOG: %0t : INFO : tb_cfg_regs : dut.cfg_rdata : expected_value: 0x%08h actual_value: 0x%08h",
                     $time, expected, read_data);
        end
    endtask

    task automatic check_output(
        input string signal_name,
        input logic [DATA_W-1:0] actual,
        input logic [DATA_W-1:0] expected,
        input string test_name
    );
        test_count++;
        if (actual !== expected) begin
            error_count++;
            $display("LOG: %0t : ERROR : tb_cfg_regs : dut.%s : expected_value: 0x%08h actual_value: 0x%08h",
                     $time, signal_name, expected, actual);
        end else begin
            $display("LOG: %0t : INFO : tb_cfg_regs : dut.%s : expected_value: 0x%08h actual_value: 0x%08h",
                     $time, signal_name, expected, actual);
        end
    endtask

    task automatic check_irq(
        input logic expected,
        input string test_name
    );
        test_count++;
        if (irq !== expected) begin
            error_count++;
            $display("LOG: %0t : ERROR : tb_cfg_regs : dut.irq : expected_value: %0b actual_value: %0b",
                     $time, expected, irq);
        end else begin
            $display("LOG: %0t : INFO : tb_cfg_regs : dut.irq : expected_value: %0b actual_value: %0b",
                     $time, expected, irq);
        end
    endtask

    //==========================================================================
    // Main Test Sequence
    //==========================================================================
    initial begin
        logic [DATA_W-1:0] read_data;
        
        $display("TEST START");
        $display("================================================================================");
        $display("Configuration Registers Testbench");
        $display("Parameters: N=%0d, W=%0d, DATA_W=%0d, ADDR_W=%0d", N, W, DATA_W, ADDR_W);
        $display("================================================================================");

        // Initialize signals
        reset = 0;
        cfg_addr = '0;
        cfg_wdata = '0;
        cfg_we = 0;
        cfg_re = 0;
        state = '0;

        // Wait for reset
        repeat(5) @(posedge clock);
        reset = 1;
        @(posedge clock);

        //======================================================================
        // TEST 1: Reset Default Values
        //======================================================================
        $display("\n[TEST 1] Reset Default Values");
        @(posedge clock);
        check_output("periph_en", periph_en, 32'h00000000, "Reset periph_en");
        check_output("alpha", alpha, 4'h8, "Reset alpha");
        check_output("wake_mask", wake_mask, 32'h0000000F, "Reset wake_mask");
        check_irq(0, "Reset IRQ");
        
        // Check default idle thresholds
        for (int i = 0; i < N; i++) begin
            check_output("idle_base_th", idle_base_th[i], 16'd1000, "Reset idle_th");
        end

        //======================================================================
        // TEST 2: Write and Read PERIPH_EN Register
        //======================================================================
        $display("\n[TEST 2] Write and Read PERIPH_EN Register");
        write_reg(ADDR_PERIPH_EN, 32'h0000000F);
        check_output("periph_en", periph_en, 32'h0000000F, "PERIPH_EN output");
        check_read(ADDR_PERIPH_EN, 32'h0000000F, "PERIPH_EN readback");

        //======================================================================
        // TEST 3: Write and Read ALPHA Register
        //======================================================================
        $display("\n[TEST 3] Write and Read ALPHA Register");
        write_reg(ADDR_ALPHA, 32'h00000005);
        check_output("alpha", alpha, 4'h5, "ALPHA output");
        check_read(ADDR_ALPHA, 32'h00000005, "ALPHA readback");

        //======================================================================
        // TEST 4: Write and Read WAKE_MASK Register
        //======================================================================
        $display("\n[TEST 4] Write and Read WAKE_MASK Register");
        write_reg(ADDR_WAKE_MASK, 32'h00000003);
        check_output("wake_mask", wake_mask, 32'h00000003, "WAKE_MASK output");
        check_read(ADDR_WAKE_MASK, 32'h00000003, "WAKE_MASK readback");

        //======================================================================
        // TEST 5: Write and Read IRQ_MASK Register
        //======================================================================
        $display("\n[TEST 5] Write and Read IRQ_MASK Register");
        write_reg(ADDR_IRQ_MASK, 32'h0000000C);
        check_read(ADDR_IRQ_MASK, 32'h0000000C, "IRQ_MASK readback");

        //======================================================================
        // TEST 6: Write and Read IDLE_TH Registers
        //======================================================================
        $display("\n[TEST 6] Write and Read IDLE_TH Registers");
        for (int i = 0; i < N; i++) begin
            automatic logic [ADDR_W-1:0] addr = ADDR_IDLE_TH_BASE + (i * 4);
            automatic logic [15:0] threshold = 16'd2000 + (i * 100);
            write_reg(addr, {16'h0000, threshold});
            check_output("idle_base_th", idle_base_th[i], threshold, "IDLE_TH output");
            check_read(addr, {16'h0000, threshold}, "IDLE_TH readback");
        end

        //======================================================================
        // TEST 7: Read-Only STATE Registers
        //======================================================================
        $display("\n[TEST 7] Read-Only STATE Registers");
        state[0] = 2'b01;
        state[1] = 2'b10;
        state[2] = 2'b11;
        state[3] = 2'b00;
        @(posedge clock);
        @(posedge clock);
        
        for (int i = 0; i < N; i++) begin
            automatic logic [ADDR_W-1:0] addr = ADDR_STATE_BASE + (i * 4);
            check_read(addr, {30'h0, state[i]}, "STATE readback");
        end
        
        // Clear any IRQ status bits that were set by state changes in TEST 7
        write_reg(ADDR_IRQ_STATUS, 32'h0000000F);

        //======================================================================
        // TEST 8: Attempt Write to Read-Only STATE (Should be Ignored)
        //======================================================================
        $display("\n[TEST 8] Attempt Write to Read-Only STATE");
        write_reg(ADDR_STATE_BASE, 32'hFFFFFFFF);
        check_read(ADDR_STATE_BASE, {30'h0, state[0]}, "STATE unchanged");

        //======================================================================
        // TEST 9: Interrupt Generation on State Change
        //======================================================================
        $display("\n[TEST 9] Interrupt Generation on State Change");
        
        // Enable IRQ for peripherals 0 and 1
        write_reg(ADDR_IRQ_MASK, 32'h00000003);
        
        // Change state[0]
        state[0] = 2'b10;
        @(posedge clock);
        @(posedge clock);
        
        // Check IRQ status and IRQ output
        check_read(ADDR_IRQ_STATUS, 32'h00000001, "IRQ_STATUS after state change");
        check_irq(1, "IRQ asserted");

        //======================================================================
        // TEST 10: Clear IRQ Status (Write-1-to-Clear)
        //======================================================================
        $display("\n[TEST 10] Clear IRQ Status (Write-1-to-Clear)");
        write_reg(ADDR_IRQ_STATUS, 32'h00000001);
        check_read(ADDR_IRQ_STATUS, 32'h00000000, "IRQ_STATUS cleared");
        check_irq(0, "IRQ deasserted");

        //======================================================================
        // TEST 11: Multiple State Changes
        //======================================================================
        $display("\n[TEST 11] Multiple State Changes");
        state[0] = 2'b11;
        state[1] = 2'b01;
        @(posedge clock);
        @(posedge clock);
        
        check_read(ADDR_IRQ_STATUS, 32'h00000003, "Multiple IRQ bits set");
        check_irq(1, "IRQ asserted for multiple");

        //======================================================================
        // TEST 12: IRQ Masking
        //======================================================================
        $display("\n[TEST 12] IRQ Masking");
        
        // Mask out peripheral 1, only peripheral 0 should generate IRQ
        write_reg(ADDR_IRQ_MASK, 32'h00000001);
        check_irq(1, "IRQ still set for periph 0");
        
        // Clear peripheral 0 IRQ
        write_reg(ADDR_IRQ_STATUS, 32'h00000001);
        check_irq(0, "IRQ cleared (periph 1 masked)");

        //======================================================================
        // TEST 13: Partial Write to Wide Registers
        //======================================================================
        $display("\n[TEST 13] Partial Write to Wide Registers");
        write_reg(ADDR_PERIPH_EN, 32'h000000AA);
        check_output("periph_en", periph_en & 32'h0000000F, 32'h0000000A, "PERIPH_EN masked");

        //======================================================================
        // TEST 14: Address Decode Boundaries
        //======================================================================
        $display("\n[TEST 14] Address Decode Boundaries");
        
        // Test highest IDLE_TH address
        write_reg(ADDR_IDLE_TH_BASE + ((N-1) * 4), 32'h00005555);
        check_read(ADDR_IDLE_TH_BASE + ((N-1) * 4), 32'h00005555, "Last IDLE_TH");
        
        // Test highest STATE address
        check_read(ADDR_STATE_BASE + ((N-1) * 4), {30'h0, state[N-1]}, "Last STATE");

        //======================================================================
        // TEST 15: Invalid Address Read (Should Return 0)
        //======================================================================
        $display("\n[TEST 15] Invalid Address Read");
        check_read(8'hFF, 32'h00000000, "Invalid address returns 0");

        //======================================================================
        // TEST 16: Simultaneous State Changes
        //======================================================================
        $display("\n[TEST 16] Simultaneous State Changes");
        
        // Clear all pending IRQs first
        write_reg(ADDR_IRQ_STATUS, 32'h0000000F);
        
        // Enable all IRQ masks
        write_reg(ADDR_IRQ_MASK, 32'h0000000F);
        
        // Change all states simultaneously
        state[0] = 2'b00;
        state[1] = 2'b00;
        state[2] = 2'b01;
        state[3] = 2'b01;
        @(posedge clock);
        @(posedge clock);
        
        check_read(ADDR_IRQ_STATUS, 32'h0000000F, "All IRQ bits set");
        check_irq(1, "IRQ for all peripherals");

        //======================================================================
        // TEST 17: Selective IRQ Clear
        //======================================================================
        $display("\n[TEST 17] Selective IRQ Clear");
        
        // Clear only peripherals 0 and 2
        write_reg(ADDR_IRQ_STATUS, 32'h00000005);
        check_read(ADDR_IRQ_STATUS, 32'h0000000A, "Selective clear");
        check_irq(1, "IRQ still active");
        
        // Clear remaining
        write_reg(ADDR_IRQ_STATUS, 32'h0000000A);
        check_read(ADDR_IRQ_STATUS, 32'h00000000, "All cleared");
        check_irq(0, "IRQ inactive");

        //======================================================================
        // TEST 18: Continuous State Changes
        //======================================================================
        $display("\n[TEST 18] Continuous State Changes");
        for (int i = 0; i < 4; i++) begin
            state[0] = state[0] + 1;
            @(posedge clock);
            @(posedge clock);
        end
        check_read(ADDR_IRQ_STATUS, 32'h00000001, "State toggled multiple times");

        //======================================================================
        // TEST 19: Zero Write Tests
        //======================================================================
        $display("\n[TEST 19] Zero Write Tests");
        write_reg(ADDR_PERIPH_EN, 32'h00000000);
        check_output("periph_en", periph_en, 32'h00000000, "Zero PERIPH_EN");
        
        write_reg(ADDR_ALPHA, 32'h00000000);
        check_output("alpha", alpha, 4'h0, "Zero ALPHA");

        //======================================================================
        // TEST 20: Maximum Value Writes
        //======================================================================
        $display("\n[TEST 20] Maximum Value Writes");
        write_reg(ADDR_PERIPH_EN, 32'hFFFFFFFF);
        check_output("periph_en", periph_en & 32'h0000000F, 32'h0000000F, "Max PERIPH_EN");
        
        write_reg(ADDR_ALPHA, 32'hFFFFFFFF);
        check_output("alpha", alpha, 4'hF, "Max ALPHA");
        
        write_reg(ADDR_IDLE_TH_BASE, 32'hFFFFFFFF);
        check_output("idle_base_th", idle_base_th[0], 16'hFFFF, "Max IDLE_TH");

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
