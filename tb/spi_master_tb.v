`timescale 1ns/1ps
// ============================================================
// Self-checking testbench for spi_master -- covers all 4 SPI modes
// (CPOL x CPHA = {00,01,10,11}) and both bit orders (MSB/LSB first).
//
// Instantiates one DUT per mode/order combination using generate,
// and runs every DUT through identical test sequences.
// ============================================================
module spi_master_tb;

// ---- Common parameters ----
parameter W           = 8;
parameter HALF_PERIOD = 25;
parameter CLK_PERIOD  = 20;   // 50 MHz

// ---- Global error / pass counters ----
integer total_errors = 0;
integer total_checks = 0;

// ============================================================
// Clock
// ============================================================
reg clk = 1'b0;
always #(CLK_PERIOD/2) clk = ~clk;

// ============================================================
// Task: run one complete test suite on a single DUT instance.
//
// Parameters passed in:
//   cpol, cpha, lsb_first -- mode configuration
//   dut_busy, dut_cs, dut_sclk, dut_mosi, dut_rx -- DUT outputs
//   dut_start, dut_tx, dut_miso -- DUT inputs (driven by task)
// ============================================================

// ============================================================
// Mode 0 (CPOL=0, CPHA=0, MSB first)
// ============================================================
reg          m0_rst_n = 0, m0_start = 0;
reg  [W-1:0] m0_tx    = 0;
wire         m0_sclk, m0_cs, m0_mosi, m0_busy;
wire [W-1:0] m0_rx;
wire         m0_miso;

spi_master #(.CPOL(0),.CPHA(0),.LSB_FIRST(0),.SPI_WORD_LENGTH(W),.HALF_PERIOD(HALF_PERIOD))
u_m0 (.clk_50MHz(clk),.rst_n(m0_rst_n),.start(m0_start),.tx_data(m0_tx),
      .miso_input(m0_miso),.sclk_out(m0_sclk),.cs_out(m0_cs),
      .mosi_output(m0_mosi),.rx_data(m0_rx),.busy(m0_busy));

// ============================================================
// Mode 1 (CPOL=0, CPHA=1, MSB first)
// ============================================================
reg          m1_rst_n = 0, m1_start = 0;
reg  [W-1:0] m1_tx    = 0;
wire         m1_sclk, m1_cs, m1_mosi, m1_busy;
wire [W-1:0] m1_rx;
wire         m1_miso;

spi_master #(.CPOL(0),.CPHA(1),.LSB_FIRST(0),.SPI_WORD_LENGTH(W),.HALF_PERIOD(HALF_PERIOD))
u_m1 (.clk_50MHz(clk),.rst_n(m1_rst_n),.start(m1_start),.tx_data(m1_tx),
      .miso_input(m1_miso),.sclk_out(m1_sclk),.cs_out(m1_cs),
      .mosi_output(m1_mosi),.rx_data(m1_rx),.busy(m1_busy));

// ============================================================
// Mode 2 (CPOL=1, CPHA=0, MSB first)
// ============================================================
reg          m2_rst_n = 0, m2_start = 0;
reg  [W-1:0] m2_tx    = 0;
wire         m2_sclk, m2_cs, m2_mosi, m2_busy;
wire [W-1:0] m2_rx;
wire         m2_miso;

spi_master #(.CPOL(1),.CPHA(0),.LSB_FIRST(0),.SPI_WORD_LENGTH(W),.HALF_PERIOD(HALF_PERIOD))
u_m2 (.clk_50MHz(clk),.rst_n(m2_rst_n),.start(m2_start),.tx_data(m2_tx),
      .miso_input(m2_miso),.sclk_out(m2_sclk),.cs_out(m2_cs),
      .mosi_output(m2_mosi),.rx_data(m2_rx),.busy(m2_busy));

// ============================================================
// Mode 3 (CPOL=1, CPHA=1, MSB first)
// ============================================================
reg          m3_rst_n = 0, m3_start = 0;
reg  [W-1:0] m3_tx    = 0;
wire         m3_sclk, m3_cs, m3_mosi, m3_busy;
wire [W-1:0] m3_rx;
wire         m3_miso;

spi_master #(.CPOL(1),.CPHA(1),.LSB_FIRST(0),.SPI_WORD_LENGTH(W),.HALF_PERIOD(HALF_PERIOD))
u_m3 (.clk_50MHz(clk),.rst_n(m3_rst_n),.start(m3_start),.tx_data(m3_tx),
      .miso_input(m3_miso),.sclk_out(m3_sclk),.cs_out(m3_cs),
      .mosi_output(m3_mosi),.rx_data(m3_rx),.busy(m3_busy));

// ============================================================
// Mode 0 LSB-first (CPOL=0, CPHA=0, LSB_FIRST=1)
// ============================================================
reg          m0l_rst_n = 0, m0l_start = 0;
reg  [W-1:0] m0l_tx    = 0;
wire         m0l_sclk, m0l_cs, m0l_mosi, m0l_busy;
wire [W-1:0] m0l_rx;
wire         m0l_miso;

spi_master #(.CPOL(0),.CPHA(0),.LSB_FIRST(1),.SPI_WORD_LENGTH(W),.HALF_PERIOD(HALF_PERIOD))
u_m0l (.clk_50MHz(clk),.rst_n(m0l_rst_n),.start(m0l_start),.tx_data(m0l_tx),
       .miso_input(m0l_miso),.sclk_out(m0l_sclk),.cs_out(m0l_cs),
       .mosi_output(m0l_mosi),.rx_data(m0l_rx),.busy(m0l_busy));

// ============================================================
// Behavioural SPI slave model (shared; one per DUT)
// ============================================================

// ---- Slave for Mode 0 ----
reg [W-1:0] s0_data = 0, s0_sr = 0, s0_rx = 0;
assign m0_miso = s0_sr[W-1];
always @(negedge m0_cs)              begin s0_sr = s0_data; s0_rx = 0; end
always @(posedge m0_sclk) begin s0_rx = {s0_rx[W-2:0], m0_mosi}; end
always @(negedge m0_sclk) if(!m0_cs) begin s0_sr = {s0_sr[W-2:0], 1'b0}; end

// ---- Slave for Mode 1 (sample on SCLK fall, shift on sample edge) ----
reg [W-1:0] s1_data = 0, s1_sr = 0, s1_rx = 0;
assign m1_miso = s1_sr[W-1];
always @(negedge m1_cs)   begin s1_sr = s1_data; s1_rx = 0; end
always @(negedge m1_sclk) begin
    s1_rx <= {s1_rx[W-2:0], m1_mosi};
    s1_sr <= {s1_sr[W-2:0], 1'b0};
end

// ---- Slave for Mode 2 (sample on SCLK fall, shift on rise) ----
reg [W-1:0] s2_data = 0, s2_sr = 0, s2_rx = 0;
assign m2_miso = s2_sr[W-1];
always @(negedge m2_cs)              begin s2_sr = s2_data; s2_rx = 0; end
always @(negedge m2_sclk) begin s2_rx = {s2_rx[W-2:0], m2_mosi}; end
always @(posedge m2_sclk) if(!m2_cs) begin s2_sr = {s2_sr[W-2:0], 1'b0}; end

// ---- Slave for Mode 3 (sample on SCLK rise, shift on sample edge) ----
reg [W-1:0] s3_data = 0, s3_sr = 0, s3_rx = 0;
assign m3_miso = s3_sr[W-1];
always @(negedge m3_cs)   begin s3_sr = s3_data; s3_rx = 0; end
always @(posedge m3_sclk) begin
    s3_rx <= {s3_rx[W-2:0], m3_mosi};
    s3_sr <= {s3_sr[W-2:0], 1'b0};
end

// ---- Slave for Mode 0 LSB-first (same edges as Mode 0, but RX is reversed) ----
reg [W-1:0] s0l_data = 0, s0l_sr = 0, s0l_rx = 0;
// Slave sends LSB of its word first (right-shift)
assign m0l_miso = s0l_sr[0];
always @(negedge m0l_cs)               begin s0l_sr = s0l_data; s0l_rx = 0; end
always @(posedge m0l_sclk) begin s0l_rx = {m0l_mosi, s0l_rx[W-1:1]}; end
always @(negedge m0l_sclk) if(!m0l_cs) begin s0l_sr = {1'b0, s0l_sr[W-1:1]}; end

// ============================================================
// Protocol monitors -- checked every negedge clk
// ============================================================
always @(negedge clk) begin
    if (m0_rst_n && (m0_cs !== ~m0_busy)) begin
        total_errors = total_errors + 1;
        $display("[%0t] ERR M0: cs(%b) != ~busy(%b)", $time, m0_cs, m0_busy);
    end
    if (m1_rst_n && (m1_cs !== ~m1_busy)) begin
        total_errors = total_errors + 1;
        $display("[%0t] ERR M1: cs(%b) != ~busy(%b)", $time, m1_cs, m1_busy);
    end
    if (m2_rst_n && (m2_cs !== ~m2_busy)) begin
        total_errors = total_errors + 1;
        $display("[%0t] ERR M2: cs(%b) != ~busy(%b)", $time, m2_cs, m2_busy);
    end
    if (m3_rst_n && (m3_cs !== ~m3_busy)) begin
        total_errors = total_errors + 1;
        $display("[%0t] ERR M3: cs(%b) != ~busy(%b)", $time, m3_cs, m3_busy);
    end
    // Idle SCLK polarity checks
    if (m0_rst_n && m0_cs && (m0_sclk !== 1'b0)) begin
        total_errors = total_errors + 1;
        $display("[%0t] ERR M0: idle SCLK=%b, expected 0 (CPOL=0)", $time, m0_sclk);
    end
    if (m1_rst_n && m1_cs && (m1_sclk !== 1'b0)) begin
        total_errors = total_errors + 1;
        $display("[%0t] ERR M1: idle SCLK=%b, expected 0 (CPOL=0)", $time, m1_sclk);
    end
    if (m2_rst_n && m2_cs && (m2_sclk !== 1'b1)) begin
        total_errors = total_errors + 1;
        $display("[%0t] ERR M2: idle SCLK=%b, expected 1 (CPOL=1)", $time, m2_sclk);
    end
    if (m3_rst_n && m3_cs && (m3_sclk !== 1'b1)) begin
        total_errors = total_errors + 1;
        $display("[%0t] ERR M3: idle SCLK=%b, expected 1 (CPOL=1)", $time, m3_sclk);
    end
end

// ============================================================
// Tasks
// ============================================================

// ---- do_transfer_m0: drive Mode-0 DUT ----
task do_transfer_m0;
    input [W-1:0] tx;
    input [W-1:0] slv;
    begin
        s0_data = slv;
        @(negedge clk); m0_tx = tx; m0_start = 1;
        @(negedge clk); m0_start = 0; m0_tx = ~tx;
        @(negedge m0_busy); #1;
        total_checks = total_checks + 1;
        if (m0_rx !== slv) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M0 rx: got %h, exp %h", $time, m0_rx, slv);
        end
        if (s0_rx !== tx) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M0 slave_rx: got %h, exp %h", $time, s0_rx, tx);
        end
        if (total_errors == 0)
            $display("[%0t] PASS M0 tx=%h rx=%h", $time, tx, slv);
    end
endtask

// ---- do_transfer_m1: drive Mode-1 DUT ----
task do_transfer_m1;
    input [W-1:0] tx;
    input [W-1:0] slv;
    begin
        s1_data = slv;
        @(negedge clk); m1_tx = tx; m1_start = 1;
        @(negedge clk); m1_start = 0; m1_tx = ~tx;
        @(negedge m1_busy); #1;
        total_checks = total_checks + 1;
        if (m1_rx !== slv) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M1 rx: got %h, exp %h", $time, m1_rx, slv);
        end
        if (s1_rx !== tx) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M1 slave_rx: got %h, exp %h", $time, s1_rx, tx);
        end
        if (total_errors == 0)
            $display("[%0t] PASS M1 tx=%h rx=%h", $time, tx, slv);
    end
endtask

// ---- do_transfer_m2: drive Mode-2 DUT ----
task do_transfer_m2;
    input [W-1:0] tx;
    input [W-1:0] slv;
    begin
        s2_data = slv;
        @(negedge clk); m2_tx = tx; m2_start = 1;
        @(negedge clk); m2_start = 0; m2_tx = ~tx;
        @(negedge m2_busy); #1;
        total_checks = total_checks + 1;
        if (m2_rx !== slv) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M2 rx: got %h, exp %h", $time, m2_rx, slv);
        end
        if (s2_rx !== tx) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M2 slave_rx: got %h, exp %h", $time, s2_rx, tx);
        end
        if (total_errors == 0)
            $display("[%0t] PASS M2 tx=%h rx=%h", $time, tx, slv);
    end
endtask

// ---- do_transfer_m3: drive Mode-3 DUT ----
task do_transfer_m3;
    input [W-1:0] tx;
    input [W-1:0] slv;
    begin
        s3_data = slv;
        @(negedge clk); m3_tx = tx; m3_start = 1;
        @(negedge clk); m3_start = 0; m3_tx = ~tx;
        @(negedge m3_busy); #1;
        total_checks = total_checks + 1;
        if (m3_rx !== slv) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M3 rx: got %h, exp %h", $time, m3_rx, slv);
        end
        if (s3_rx !== tx) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M3 slave_rx: got %h, exp %h", $time, s3_rx, tx);
        end
        if (total_errors == 0)
            $display("[%0t] PASS M3 tx=%h rx=%h", $time, tx, slv);
    end
endtask

// ---- do_transfer_m0l: drive Mode-0 LSB-first DUT ----
task do_transfer_m0l;
    input [W-1:0] tx;
    input [W-1:0] slv;
    begin
        s0l_data = slv;
        @(negedge clk); m0l_tx = tx; m0l_start = 1;
        @(negedge clk); m0l_start = 0; m0l_tx = ~tx;
        @(negedge m0l_busy); #1;
        total_checks = total_checks + 1;
        if (m0l_rx !== slv) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M0L rx: got %h, exp %h", $time, m0l_rx, slv);
        end
        if (s0l_rx !== tx) begin
            total_errors = total_errors + 1;
            $display("[%0t] ERR M0L slave_rx: got %h, exp %h", $time, s0l_rx, tx);
        end
        if (total_errors == 0)
            $display("[%0t] PASS M0L tx=%h rx=%h", $time, tx, slv);
    end
endtask

// ============================================================
// Test sequence
// ============================================================
integer i;
reg [W-1:0] rtx, rslv;

initial begin
    $dumpfile("spi_master_tb.vcd");
    $dumpvars(0, spi_master_tb);

    // ---- Reset all DUTs ----
    m0_rst_n = 0; m1_rst_n = 0; m2_rst_n = 0; m3_rst_n = 0; m0l_rst_n = 0;
    #(CLK_PERIOD*4);
    m0_rst_n = 1; m1_rst_n = 1; m2_rst_n = 1; m3_rst_n = 1; m0l_rst_n = 1;
    repeat(3) @(negedge clk);

    // ----------------------------------------------------------------
    // Test: directed corner vectors on every mode
    // ----------------------------------------------------------------
    $display("\n=== Directed vectors -- all modes ===");
    do_transfer_m0 (8'hA5, 8'h3C);
    do_transfer_m0 (8'h00, 8'hFF);
    do_transfer_m0 (8'hFF, 8'h00);
    do_transfer_m0 (8'h55, 8'hAA);

    do_transfer_m1 (8'hA5, 8'h3C);
    do_transfer_m1 (8'h00, 8'hFF);
    do_transfer_m1 (8'hFF, 8'h00);
    do_transfer_m1 (8'h55, 8'hAA);

    do_transfer_m2 (8'hA5, 8'h3C);
    do_transfer_m2 (8'h00, 8'hFF);
    do_transfer_m2 (8'hFF, 8'h00);
    do_transfer_m2 (8'h55, 8'hAA);

    do_transfer_m3 (8'hA5, 8'h3C);
    do_transfer_m3 (8'h00, 8'hFF);
    do_transfer_m3 (8'hFF, 8'h00);
    do_transfer_m3 (8'h55, 8'hAA);

    do_transfer_m0l(8'hA5, 8'h3C);
    do_transfer_m0l(8'h00, 8'hFF);
    do_transfer_m0l(8'hFF, 8'h00);
    do_transfer_m0l(8'h55, 8'hAA);

    // ----------------------------------------------------------------
    // Test: random vectors (20 per mode)
    // ----------------------------------------------------------------
    $display("\n=== Random vectors -- all modes ===");
    for (i = 0; i < 20; i = i + 1) begin
        rtx = $random; rslv = $random;
        do_transfer_m0 (rtx, rslv);
        do_transfer_m1 (rtx, rslv);
        do_transfer_m2 (rtx, rslv);
        do_transfer_m3 (rtx, rslv);
        do_transfer_m0l(rtx, rslv);
    end

    // ----------------------------------------------------------------
    // Test: reset in the middle of a Mode-0 transfer
    // ----------------------------------------------------------------
    $display("\n=== Mid-transfer reset (Mode 0) ===");
    s0_data = 8'hF0;
    @(negedge clk); m0_tx = 8'h0F; m0_start = 1;
    @(negedge clk); m0_start = 0;
    repeat(60) @(negedge clk);
    m0_rst_n = 0; #2;
    if (m0_busy !== 0 || m0_cs !== 1 || m0_sclk !== 0) begin
        total_errors = total_errors + 1;
        $display("ERR M0: reset did not abort (busy=%b cs=%b sclk=%b)", m0_busy, m0_cs, m0_sclk);
    end else
        $display("PASS M0: reset aborted transfer correctly");
    repeat(3) @(negedge clk); m0_rst_n = 1;
    repeat(3) @(negedge clk);
    do_transfer_m0(8'hDE, 8'hAD);

    // ----------------------------------------------------------------
    // Test: reset in the middle of a Mode-2 transfer (SCLK idle=1)
    // ----------------------------------------------------------------
    $display("\n=== Mid-transfer reset (Mode 2, CPOL=1) ===");
    s2_data = 8'hC3;
    @(negedge clk); m2_tx = 8'h3C; m2_start = 1;
    @(negedge clk); m2_start = 0;
    repeat(60) @(negedge clk);
    m2_rst_n = 0; #2;
    if (m2_busy !== 0 || m2_cs !== 1 || m2_sclk !== 1) begin
        total_errors = total_errors + 1;
        $display("ERR M2: reset wrong (busy=%b cs=%b sclk=%b, expect 0,1,1)", m2_busy, m2_cs, m2_sclk);
    end else
        $display("PASS M2: reset aborted, SCLK returned to CPOL=1");
    repeat(3) @(negedge clk); m2_rst_n = 1;
    repeat(3) @(negedge clk);
    do_transfer_m2(8'hBE, 8'hEF);

    // ----------------------------------------------------------------
    // Summary
    // ----------------------------------------------------------------
    #(CLK_PERIOD*10);
    $display("\n==================================================");
    if (total_errors == 0)
        $display("ALL TESTS PASSED (%0d transfers checked, all modes)", total_checks);
    else
        $display("TESTS FAILED: %0d error(s) across %0d checks", total_errors, total_checks);
    $display("==================================================\n");
    $finish;
end

// Watchdog
initial begin
    #(20_000_000);
    $display("ERROR: simulation timeout");
    $finish;
end

endmodule
