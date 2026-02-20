//==============================================================================
// Module: cfg_regs
// Description: Configuration & Control Registers for Power-Aware Peripheral Controller
//              Provides programmable interface to control power management behavior
//              at runtime (idle thresholds, enables, wake masks, predictor tuning)
//==============================================================================

module cfg_regs #(
    parameter int N = 4,          // Number of peripherals
    parameter int W = 16,         // Width of idle threshold counters
    parameter int DATA_W = 32,    // Data bus width
    parameter int ADDR_W = 8      // Address bus width
) (
    // Clock and Reset
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Configuration Interface
    input  logic [ADDR_W-1:0]       cfg_addr,
    input  logic [DATA_W-1:0]       cfg_wdata,
    input  logic                    cfg_we,
    input  logic                    cfg_re,
    output logic [DATA_W-1:0]       cfg_rdata,
    
    // Status Input (from FSM)
    input  logic [N-1:0][1:0]       state,
    
    // Configuration Outputs
    output logic [N-1:0]            periph_en,
    output logic [N-1:0][W-1:0]     idle_base_th,
    output logic [3:0]              alpha,
    output logic [N-1:0]            wake_mask,
    
    // Interrupt Output
    output logic                    irq
);

    //==========================================================================
    // Register Map Definition
    //==========================================================================
    // 0x00: PERIPH_EN      - Peripheral enable bits [N-1:0]
    // 0x04: ALPHA          - Predictor tuning parameter [3:0]
    // 0x08: WAKE_MASK      - Wake source enable mask [N-1:0]
    // 0x0C: IRQ_MASK       - Interrupt mask [N-1:0] (1=enable IRQ for state change)
    // 0x10: IDLE_TH_0      - Base idle threshold for peripheral 0
    // 0x14: IDLE_TH_1      - Base idle threshold for peripheral 1
    // ...
    // 0x10+4*(N-1): IDLE_TH_N-1
    // 0x80: STATE_0        - Current state for peripheral 0 (read-only)
    // 0x84: STATE_1        - Current state for peripheral 1 (read-only)
    // ...
    // 0x80+4*(N-1): STATE_N-1
    // 0xC0: IRQ_STATUS     - Interrupt status [N-1:0] (read-only, write-1-to-clear)
    
    localparam logic [ADDR_W-1:0] ADDR_PERIPH_EN  = 8'h00;
    localparam logic [ADDR_W-1:0] ADDR_ALPHA      = 8'h04;
    localparam logic [ADDR_W-1:0] ADDR_WAKE_MASK  = 8'h08;
    localparam logic [ADDR_W-1:0] ADDR_IRQ_MASK   = 8'h0C;
    localparam logic [ADDR_W-1:0] ADDR_IDLE_TH_BASE = 8'h10;
    localparam logic [ADDR_W-1:0] ADDR_STATE_BASE   = 8'h80;
    localparam logic [ADDR_W-1:0] ADDR_IRQ_STATUS   = 8'hC0;

    //==========================================================================
    // Internal Registers
    //==========================================================================
    logic [N-1:0]            periph_en_reg;
    logic [N-1:0][W-1:0]     idle_base_th_reg;
    logic [3:0]              alpha_reg;
    logic [N-1:0]            wake_mask_reg;
    logic [N-1:0]            irq_mask_reg;
    logic [N-1:0]            irq_status_reg;
    
    // State change detection
    logic [N-1:0][1:0]       state_prev;
    logic [N-1:0]            state_changed;

    //==========================================================================
    // Write Logic
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset to safe defaults
            periph_en_reg     <= '0;              // All peripherals disabled
            alpha_reg         <= 4'h8;            // Mid-range predictor tuning
            wake_mask_reg     <= '1;              // All wake sources enabled
            irq_mask_reg      <= '0;              // All interrupts disabled by default
            irq_status_reg    <= '0;              // No pending interrupts
            state_prev        <= '0;              // Clear previous state
            for (int i = 0; i < N; i++) begin
                idle_base_th_reg[i] <= W'(1000);  // Default 1000 cycle threshold
            end
        end else if (cfg_we) begin
            // Decode address and write to appropriate register
            case (cfg_addr)
                ADDR_PERIPH_EN: begin
                    periph_en_reg <= cfg_wdata[N-1:0];
                end
                
                ADDR_ALPHA: begin
                    alpha_reg <= cfg_wdata[3:0];
                end
                
                ADDR_WAKE_MASK: begin
                    wake_mask_reg <= cfg_wdata[N-1:0];
                end
                
                ADDR_IRQ_MASK: begin
                    irq_mask_reg <= cfg_wdata[N-1:0];
                end
                
                ADDR_IRQ_STATUS: begin
                    // Write-1-to-clear: Clear interrupt status bits
                    irq_status_reg <= irq_status_reg & ~cfg_wdata[N-1:0];
                end
                
                default: begin
                    // Check if address is in IDLE_TH range
                    if ((cfg_addr >= ADDR_IDLE_TH_BASE) && 
                        (cfg_addr < ADDR_IDLE_TH_BASE + (N * 4))) begin
                        automatic int idx = (cfg_addr - ADDR_IDLE_TH_BASE) >> 2;
                        if (idx < N) begin
                            idle_base_th_reg[idx] <= cfg_wdata[W-1:0];
                        end
                    end
                    // STATE registers are read-only, ignore writes
                end
            endcase
        end
    end

    //==========================================================================
    // Read Logic
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_rdata <= '0;
        end else if (cfg_re) begin
            // Default read value
            cfg_rdata <= '0;
            
            case (cfg_addr)
                ADDR_PERIPH_EN: begin
                    cfg_rdata[N-1:0] <= periph_en_reg;
                end
                
                ADDR_ALPHA: begin
                    cfg_rdata[3:0] <= alpha_reg;
                end
                
                ADDR_WAKE_MASK: begin
                    cfg_rdata[N-1:0] <= wake_mask_reg;
                end
                
                ADDR_IRQ_MASK: begin
                    cfg_rdata[N-1:0] <= irq_mask_reg;
                end
                
                ADDR_IRQ_STATUS: begin
                    cfg_rdata[N-1:0] <= irq_status_reg;
                end
                
                default: begin
                    // Check if address is in IDLE_TH range
                    if ((cfg_addr >= ADDR_IDLE_TH_BASE) && 
                        (cfg_addr < ADDR_IDLE_TH_BASE + (N * 4))) begin
                        automatic int idx = (cfg_addr - ADDR_IDLE_TH_BASE) >> 2;
                        if (idx < N) begin
                            cfg_rdata[W-1:0] <= idle_base_th_reg[idx];
                        end
                    end
                    // Check if address is in STATE range (read-only status)
                    else if ((cfg_addr >= ADDR_STATE_BASE) && 
                             (cfg_addr < ADDR_STATE_BASE + (N * 4))) begin
                        automatic int idx = (cfg_addr - ADDR_STATE_BASE) >> 2;
                        if (idx < N) begin
                            cfg_rdata[1:0] <= state[idx];
                        end
                    end
                end
            endcase
        end
    end

    //==========================================================================
    // State Change Detection & Interrupt Generation
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_prev <= '0;
        end else begin
            state_prev <= state;
        end
    end
    
    // Detect state changes for each peripheral
    always_comb begin
        for (int i = 0; i < N; i++) begin
            state_changed[i] = (state[i] != state_prev[i]);
        end
    end
    
    // Set interrupt status bits on state change (sticky until cleared)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            irq_status_reg <= '0;
        end else begin
            // Set bits on state change, cleared by software write
            for (int i = 0; i < N; i++) begin
                if (state_changed[i]) begin
                    irq_status_reg[i] <= 1'b1;
                end else if (cfg_we && (cfg_addr == ADDR_IRQ_STATUS) && cfg_wdata[i]) begin
                    irq_status_reg[i] <= 1'b0;  // Write-1-to-clear
                end
            end
        end
    end
    
    // Generate combined interrupt output
    assign irq = |(irq_status_reg & irq_mask_reg);

    //==========================================================================
    // Output Assignments
    //==========================================================================
    assign periph_en    = periph_en_reg;
    assign idle_base_th = idle_base_th_reg;
    assign alpha        = alpha_reg;
    assign wake_mask    = wake_mask_reg;

    //==========================================================================
    // Assertions for Parameter Validation
    //==========================================================================
    initial begin
        assert (N > 0 && N <= 32) 
            else $error("N must be between 1 and 32");
        assert (W > 0 && W <= DATA_W) 
            else $error("W must be between 1 and DATA_W");
        assert (DATA_W == 32) 
            else $warning("DATA_W != 32 may require register map adjustment");
    end

endmodule
