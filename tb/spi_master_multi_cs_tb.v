`timescale 1ns/1ps
// ============================================================
// Behavioural SPI slave (Mode 0, MSB first)
// Sends tx_word, records what it received, counts how many times
// it was selected. Drives MISO low while deselected.
// ============================================================
module spi_slave_model #(parameter W = 8)
(
    input  wire         sclk,
    input  wire         cs_n,
    input  wire         mosi,
    output wire         miso,
    input  wire [W-1:0] tx_word,
    output reg  [W-1:0] rx_word,
    output reg  [31:0]  xfer_count
);
    reg [W-1:0] sr;

    initial begin
        sr         = {W{1'b0}};
        rx_word    = {W{1'b0}};
        xfer_count = 32'd0;
    end

    assign miso = cs_n ? 1'b0 : sr[W-1];

    always @(negedge cs_n) begin
        sr         = tx_word;
        rx_word    = {W{1'b0}};
        xfer_count = xfer_count + 32'd1;
    end

    always @(posedge sclk) begin
        if (!cs_n) rx_word = {rx_word[W-2:0], mosi};
    end

    always @(negedge sclk) begin
        if (!cs_n) sr = {sr[W-2:0], 1'b0};
    end
endmodule

// ============================================================
// Self-checking testbench for spi_master_multi_cs (Mode 0)
// ============================================================
module spi_master_multi_cs_tb;

parameter W           = 8;
parameter N           = 4;      // number of slaves on the main bus
parameter HALF_PERIOD = 25;
parameter CLK_PERIOD  = 20;     // 50 MHz
localparam SEL_W      = 2;      // = ceil(log2(N)) for N = 4

integer errors = 0;
integer checks = 0;

reg clk   = 1'b0;
reg rst_n = 1'b0;
always #(CLK_PERIOD/2) clk = ~clk;

// ---------------- Main DUT: 4 slaves ----------------
reg                start     = 1'b0;
reg  [SEL_W-1:0]   slave_sel = {SEL_W{1'b0}};
reg  [W-1:0]       tx_data   = {W{1'b0}};
wire [N-1:0]       miso_bus;
wire [N-1:0]       cs_bus;
wire               sclk, mosi, busy;
wire [W-1:0]       rx_data;

spi_master_multi_cs #(
    .CPOL(1'b0), .CPHA(1'b0), .LSB_FIRST(1'b0),
    .SPI_WORD_LENGTH(W), .HALF_PERIOD(HALF_PERIOD), .NUM_SLAVES(N)
) dut (
    .clk_50MHz  (clk),
    .rst_n      (rst_n),
    .start      (start),
    .slave_sel  (slave_sel),
    .tx_data    (tx_data),
    .miso_input (miso_bus),
    .sclk_out   (sclk),
    .cs_out     (cs_bus),
    .mosi_output(mosi),
    .rx_data    (rx_data),
    .busy       (busy)
);

// Each slave returns a different word.
reg  [N*W-1:0]  slave_tx_flat;
wire [N*W-1:0]  slave_rx_flat;
wire [N*32-1:0] slave_cnt_flat;

genvar g;
generate
    for (g = 0; g < N; g = g + 1) begin : slv
        spi_slave_model #(.W(W)) u_slave (
            .sclk      (sclk),
            .cs_n      (cs_bus[g]),
            .mosi      (mosi),
            .miso      (miso_bus[g]),
            .tx_word   (slave_tx_flat[g*W +: W]),
            .rx_word   (slave_rx_flat[g*W +: W]),
            .xfer_count(slave_cnt_flat[g*32 +: 32])
        );
    end
endgenerate

// ---------------- Second DUT: 3 slaves (invalid-select test) ----------------
reg          start2 = 1'b0;
reg  [1:0]   sel2   = 2'd0;
reg  [W-1:0] tx2    = {W{1'b0}};
wire [2:0]   cs2;
wire         sclk2, mosi2, busy2;
wire [W-1:0] rx2;

spi_master_multi_cs #(
    .CPOL(1'b0), .CPHA(1'b0), .LSB_FIRST(1'b0),
    .SPI_WORD_LENGTH(W), .HALF_PERIOD(HALF_PERIOD), .NUM_SLAVES(3)
) dut2 (
    .clk_50MHz  (clk),
    .rst_n      (rst_n),
    .start      (start2),
    .slave_sel  (sel2),
    .tx_data    (tx2),
    .miso_input (3'b000),
    .sclk_out   (sclk2),
    .cs_out     (cs2),
    .mosi_output(mosi2),
    .rx_data    (rx2),
    .busy       (busy2)
);

// ---------------- Helpers ----------------
reg [N*32-1:0] cnt_before;
reg [N-1:0]    exp_cs;
integer        j;
integer        e0;

task do_xfer;
    input [SEL_W-1:0] sel;
    input [W-1:0]     tx;
    begin
        e0         = errors;
        cnt_before = slave_cnt_flat;

        @(negedge clk);
        slave_sel = sel;
        tx_data   = tx;
        start     = 1'b1;
        @(negedge clk);
        start     = 1'b0;
        tx_data   = ~tx;          // change inputs after start to prove they were latched
        slave_sel = ~sel;

        // Mid-transfer: only the selected CS may be low.
        repeat (40) @(negedge clk);
        exp_cs = {N{1'b1}};
        exp_cs[sel] = 1'b0;
        if (cs_bus !== exp_cs) begin
            errors = errors + 1;
            $display("[%0t] ERROR: cs_bus = %b, expected %b (sel=%0d)", $time, cs_bus, exp_cs, sel);
        end

        @(negedge busy);
        #1;
        checks = checks + 1;

        if (rx_data !== slave_tx_flat[sel*W +: W]) begin
            errors = errors + 1;
            $display("[%0t] ERROR: rx_data = %h, expected %h (slave %0d)",
                     $time, rx_data, slave_tx_flat[sel*W +: W], sel);
        end
        if (slave_rx_flat[sel*W +: W] !== tx) begin
            errors = errors + 1;
            $display("[%0t] ERROR: slave %0d received %h, expected %h",
                     $time, sel, slave_rx_flat[sel*W +: W], tx);
        end
        for (j = 0; j < N; j = j + 1) begin
            if (j == sel) begin
                if (slave_cnt_flat[j*32 +: 32] !== cnt_before[j*32 +: 32] + 1) begin
                    errors = errors + 1;
                    $display("[%0t] ERROR: slave %0d was not selected exactly once", $time, j);
                end
            end else begin
                if (slave_cnt_flat[j*32 +: 32] !== cnt_before[j*32 +: 32]) begin
                    errors = errors + 1;
                    $display("[%0t] ERROR: slave %0d was selected but should not be", $time, j);
                end
            end
        end
        if (cs_bus !== {N{1'b1}} || busy !== 1'b0) begin
            errors = errors + 1;
            $display("[%0t] ERROR: bad idle state after transfer (cs=%b busy=%b)", $time, cs_bus, busy);
        end

        if (errors == e0)
            $display("[%0t] PASS: slave %0d tx=%h rx=%h", $time, sel, tx, rx_data);
    end
endtask

integer     i;
reg [SEL_W-1:0] rsel;
reg [W-1:0]     rtx;

// ---------------- Test sequence ----------------
initial begin
    $dumpfile("spi_master_multi_cs_tb.vcd");
    $dumpvars(0, spi_master_multi_cs_tb);

    slave_tx_flat = {8'hD4, 8'hC3, 8'hB2, 8'hA1};   // slave0=A1 ... slave3=D4

    rst_n = 1'b0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (3) @(negedge clk);

    $display("--- each slave in turn, two rounds");
    for (i = 0; i < 8; i = i + 1)
        do_xfer(i % N, 8'h30 + i);

    $display("--- random slave and data");
    for (i = 0; i < 40; i = i + 1) begin
        rsel = {$random} % N;
        rtx  = $random;
        do_xfer(rsel, rtx);
    end

    // ---- Invalid slave number on a 3-slave instance ----
    $display("--- invalid slave select (3-slave instance, sel=3)");
    @(negedge clk);
    sel2 = 2'd3; tx2 = 8'h5A; start2 = 1'b1;
    @(negedge clk);
    start2 = 1'b0;
    repeat (20) @(negedge clk);
    if (busy2 !== 1'b0 || cs2 !== 3'b111) begin
        errors = errors + 1;
        $display("ERROR: invalid select was not ignored (busy2=%b cs2=%b)", busy2, cs2);
    end else
        $display("PASS: invalid select ignored");

    $display("--- valid select on the 3-slave instance (sel=2)");
    @(negedge clk);
    sel2 = 2'd2; tx2 = 8'h5A; start2 = 1'b1;
    @(negedge clk);
    start2 = 1'b0;
    repeat (40) @(negedge clk);
    if (cs2 !== 3'b011) begin
        errors = errors + 1;
        $display("ERROR: cs2 = %b, expected 011", cs2);
    end
    @(negedge busy2);
    #1;
    if (cs2 !== 3'b111) begin
        errors = errors + 1;
        $display("ERROR: cs2 = %b after transfer, expected 111", cs2);
    end else
        $display("PASS: valid select on 3-slave instance");

    #(CLK_PERIOD*10);
    $display("==================================================");
    if (errors == 0) $display("ALL TESTS PASSED (%0d transfers checked)", checks);
    else             $display("TESTS FAILED: %0d error(s)", errors);
    $display("==================================================");
    $finish;
end

// Watchdog
initial begin
    #(5_000_000);
    $display("ERROR: simulation timeout");
    $finish;
end

endmodule
