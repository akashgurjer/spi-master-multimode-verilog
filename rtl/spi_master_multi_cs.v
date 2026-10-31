`timescale 1ns/1ps
// ============================================================
// spi_master_multi_cs
//
// Multi-slave wrapper around the verified spi_master core.
// The core is NOT modified: this module only
//   1. latches the requested slave number when a transfer starts,
//   2. decodes the core's single chip select into NUM_SLAVES
//      active-low chip selects (only the selected one goes low),
//   3. muxes the selected slave's MISO line back to the core.
//
// SCLK, MOSI and rx_data are shared by all slaves (standard
// SPI multi-drop). Each slave must ignore SCLK/MOSI while its
// own chip select is high.
//
// A start request with slave_sel >= NUM_SLAVES is ignored.
// ============================================================
module spi_master_multi_cs
#(
    parameter CPOL            = 1'b0,
    parameter CPHA            = 1'b0,
    parameter LSB_FIRST       = 1'b0,
    parameter SPI_WORD_LENGTH = 8,
    parameter HALF_PERIOD     = 25,
    parameter NUM_SLAVES      = 4
)
(
    clk_50MHz, rst_n, start, slave_sel, tx_data, miso_input,
    sclk_out, cs_out, mosi_output, rx_data, busy
);

// clog2: elaboration-time ceiling of log base 2.
function integer clog2;
    input integer value;
    integer v;
    begin
        v     = value - 1;
        clog2 = 0;
        while (v > 0) begin
            clog2 = clog2 + 1;
            v     = v >> 1;
        end
    end
endfunction

// SEL_W: width of the slave-select input (minimum 1 bit).
localparam SEL_W = (clog2(NUM_SLAVES) < 1) ? 1 : clog2(NUM_SLAVES);

input  wire                        clk_50MHz;
input  wire                        rst_n;
input  wire                        start;
input  wire [SEL_W-1:0]            slave_sel;     // which slave to talk to
input  wire [SPI_WORD_LENGTH-1:0]  tx_data;
input  wire [NUM_SLAVES-1:0]       miso_input;    // one MISO line per slave
output wire                        sclk_out;
output reg  [NUM_SLAVES-1:0]       cs_out;        // one active-low CS per slave
output wire                        mosi_output;
output wire [SPI_WORD_LENGTH-1:0]  rx_data;
output wire                        busy;

// sel_ok: requested slave number exists.
wire sel_ok = (slave_sel < NUM_SLAVES);

// core_start: start request passed to the core (only for valid slaves).
wire core_start = start & sel_ok;

// sel_q: slave number latched on the same edge the core accepts the start,
// so changing slave_sel during a transfer has no effect.
reg [SEL_W-1:0] sel_q;

always @(posedge clk_50MHz or negedge rst_n) begin
    if (!rst_n)
        sel_q <= {SEL_W{1'b0}};
    else if (core_start & ~busy)
        sel_q <= slave_sel;
end

// core_cs_n: the core's single chip select (low during a transfer).
wire core_cs_n;

// miso_mux: MISO of the currently selected slave.
wire miso_mux = miso_input[sel_q];

// CS decode: only the selected slave sees the core's chip select.
integer k;
always @(*) begin
    for (k = 0; k < NUM_SLAVES; k = k + 1)
        cs_out[k] = (sel_q == k) ? core_cs_n : 1'b1;
end

// Unmodified single-slave core.
spi_master #(
    .CPOL           (CPOL),
    .CPHA           (CPHA),
    .LSB_FIRST      (LSB_FIRST),
    .SPI_WORD_LENGTH(SPI_WORD_LENGTH),
    .HALF_PERIOD    (HALF_PERIOD)
) u_core (
    .clk_50MHz  (clk_50MHz),
    .rst_n      (rst_n),
    .start      (core_start),
    .tx_data    (tx_data),
    .miso_input (miso_mux),
    .sclk_out   (sclk_out),
    .cs_out     (core_cs_n),
    .mosi_output(mosi_output),
    .rx_data    (rx_data),
    .busy       (busy)
);

endmodule
