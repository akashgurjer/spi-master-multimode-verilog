`timescale 1ns/1ps
module spi_master
#(
    parameter CPOL             = 1'b0,
    parameter CPHA             = 1'b0,
    parameter LSB_FIRST        = 1'b0,
    parameter SPI_WORD_LENGTH  = 8,
    parameter HALF_PERIOD      = 25
)
(
    input wire clk_50MHz,
    input wire rst_n,
    input wire start,
    input wire [SPI_WORD_LENGTH-1:0] tx_data,
    input wire miso_input,
    output reg sclk_out,
    output reg cs_out,
    output reg mosi_output,
    output reg [SPI_WORD_LENGTH-1:0] rx_data,
    output reg busy
);

//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

// ============================================================
// clog2: Elaboration-time ceiling-log-2 helper.
// ============================================================
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

// ============================================================
// Derived width localparams
// ============================================================
// CNT_W: Width of the half-period counter.
localparam CNT_W = clog2(HALF_PERIOD + 1);

// BIT_W: Width of the bit counter, minimum 1.
localparam BIT_W = (clog2(SPI_WORD_LENGTH) < 1) ? 1 : clog2(SPI_WORD_LENGTH);

// HALF_CNT: Counter value that triggers an SCLK transition.
localparam [CNT_W-1:0] HALF_CNT   = HALF_PERIOD;

// CNT_RELOAD: Counter value reloaded after each SCLK transition.
localparam [CNT_W-1:0] CNT_RELOAD = 1;

// LAST_BIT: Index of the final bit in a transfer.
localparam [BIT_W-1:0] LAST_BIT   = SPI_WORD_LENGTH - 1;

// ============================================================
// SPI Mode encoding -- resolved entirely at elaboration time.
// ============================================================
//
//  Mode | CPOL | CPHA | SCLK idle | Sample edge | Shift edge | Last edge
//  -----+------+------+-----------+-------------+------------+----------
//    0  |  0   |  0   |   LOW     |    Rise     |    Fall    |   Fall
//    1  |  0   |  1   |   LOW     |    Fall     |    Rise    |   Fall
//    2  |  1   |  0   |   HIGH    |    Fall     |    Rise    |   Rise
//    3  |  1   |  1   |   HIGH    |    Rise     |    Fall    |   Rise
//
//  Unified rule:
//    CPOL == CPHA  =>  sample on rise (Modes 0 and 3)
//    CPOL != CPHA  =>  sample on fall (Modes 1 and 2)
//    Terminal/last edge: fall when CPOL=0, rise when CPOL=1.
//
// SAMPLE_RISE = 1: MISO is sampled on rising SCLK  (Modes 0 and 3).
// SAMPLE_RISE = 0: MISO is sampled on falling SCLK (Modes 1 and 2).
localparam SAMPLE_RISE   = (CPOL == CPHA) ? 1 : 0;

// COUNT_ON_RISE = 1: bit_cnt and xfer_done track rising  SCLK (CPOL=1).
// COUNT_ON_RISE = 0: bit_cnt and xfer_done track falling SCLK (CPOL=0).
localparam COUNT_ON_RISE = CPOL ? 1 : 0;

// ============================================================
// Internal registers
// ============================================================
// cnt: Counts system-clock cycles during each SCLK half-period.
reg [CNT_W-1:0] cnt;

// bit_cnt: Counts terminal SCLK edges during the current transfer.
reg [BIT_W-1:0] bit_cnt;

// shift_reg: Holds TX data pre-transfer; accumulates received MISO bits.
reg [SPI_WORD_LENGTH-1:0] shift_reg;

// ============================================================
// Combinational control signals
// ============================================================
// xfer_start: Accepts a start request when the controller is idle.
wire xfer_start = ~busy & start;

// half_tick: Time to toggle SCLK (HALF_PERIOD system clocks elapsed).
wire half_tick = busy & (cnt == HALF_CNT);

// SCLK edge detects -- based on the CURRENT (pre-toggle) sclk_out.
wire sclk_rise = half_tick & ~sclk_out;   // 0->1 transition
wire sclk_fall = half_tick &  sclk_out;   // 1->0 transition

// sclk_sample: SCLK edge on which MISO is captured (mode-dependent).
wire sclk_sample = SAMPLE_RISE ? sclk_rise : sclk_fall;

// sclk_shift: SCLK edge on which MOSI is updated (mode-dependent).
//             Always the opposite type from sclk_sample.
wire sclk_shift  = SAMPLE_RISE ? sclk_fall  : sclk_rise;

// cnt_edge: The "terminal" (last) edge of each SCLK full period.
//   CPOL=0 => full-period sequence R,F,R,F,...  last edge = fall.
//   CPOL=1 => full-period sequence F,R,F,R,...  last edge = rise.
//   bit_cnt and xfer_done are gated on this edge.
wire cnt_edge = COUNT_ON_RISE ? sclk_rise : sclk_fall;

// last_bit: True when the current bit is the final one.
wire last_bit = (bit_cnt == LAST_BIT);

// xfer_done: Transfer completes at the final terminal cnt_edge event.
wire xfer_done = cnt_edge & last_bit;

// mosi_bit: Selects the output data bit according to bit order.
//   LSB_FIRST=0 (MSB first): drive shift_reg[MSB].
//   LSB_FIRST=1 (LSB first): drive shift_reg[0].
wire mosi_bit = LSB_FIRST ? shift_reg[0] : shift_reg[SPI_WORD_LENGTH-1];

// ============================================================
// busy -- asserted throughout an active transfer.
// ============================================================
always @(posedge clk_50MHz or negedge rst_n) begin
    /*
    Purpose: Controls the transfer status.

    Sets busy when a start request is accepted; clears busy when
    the final terminal SCLK edge completes the transfer.
    */
    if (!rst_n)
        busy <= 1'b0;
    else if (xfer_start)
        busy <= 1'b1;
    else if (xfer_done)
        busy <= 1'b0;
end

// ============================================================
// cs_out -- chip select (active-low, asserted during transfer).
// ============================================================
always @(*) begin
    /*
    Purpose: Drives chip select low during an active transfer,
    high while idle.
    */
    cs_out = ~busy;
end

// ============================================================
// cnt -- half-period counter.
// ============================================================
always @(posedge clk_50MHz or negedge rst_n) begin
    /*
    Purpose: Counts system-clock cycles to determine when SCLK
    should toggle.

    Resets while idle; reloads to CNT_RELOAD after each toggle.
    */
    if (!rst_n)
        cnt <= {CNT_W{1'b0}};
    else if (!busy)
        cnt <= {CNT_W{1'b0}};
    else if (half_tick)
        cnt <= CNT_RELOAD;
    else
        cnt <= cnt + 1'b1;
end

// ============================================================
// sclk_out -- serial clock.
// ============================================================
always @(posedge clk_50MHz or negedge rst_n) begin
    /*
    Purpose: Generates the SPI serial clock.

    Idle level is CPOL:
      CPOL=0 => idle LOW  (Modes 0 and 1)
      CPOL=1 => idle HIGH (Modes 2 and 3)

    SCLK toggles on every half_tick.  At xfer_done the final toggle
    naturally returns SCLK to CPOL (the transfer has 2*W toggles,
    an even count, so it ends on the same polarity it started from).
    The !busy hold enforces the idle level between transfers.
    */
    if (!rst_n)
        sclk_out <= CPOL;       // reset to idle polarity
    else if (!busy)
        sclk_out <= CPOL;       // hold at idle between transfers
    else if (half_tick)
        sclk_out <= ~sclk_out;
end

// ============================================================
// bit_cnt -- counts completed SCLK full periods.
// ============================================================
always @(posedge clk_50MHz or negedge rst_n) begin
    /*
    Purpose: Tracks the number of bits transferred.

    Increments on cnt_edge (fall for CPOL=0; rise for CPOL=1).
    Resets to 0 after the last bit so the controller is immediately
    ready for the next transfer.
    */
    if (!rst_n)
        bit_cnt <= {BIT_W{1'b0}};
    else if (cnt_edge) begin
        if (last_bit)
            bit_cnt <= {BIT_W{1'b0}};
        else
            bit_cnt <= bit_cnt + 1'b1;
    end
end

// ============================================================
// shift_reg -- dual-purpose TX/RX shift register.
// ============================================================
always @(posedge clk_50MHz or negedge rst_n) begin
    /*
    Purpose: Pre-loaded with tx_data while idle.

    During a transfer, shifted on every sclk_sample edge to
    accumulate received MISO bits.  Shift direction:
      LSB_FIRST=0 (MSB first): left-shift  -- MISO enters at bit 0.
      LSB_FIRST=1 (LSB first): right-shift -- MISO enters at MSB.

    The mosi_bit wire always reads from the correct end of shift_reg,
    so each shift automatically presents the next TX bit on MOSI.
    */
    if (!rst_n)
        shift_reg <= {SPI_WORD_LENGTH{1'b0}};
    else if (!busy)
        shift_reg <= tx_data;                               // pre-load while idle
    else if (sclk_sample) begin
        if (LSB_FIRST)
            shift_reg <= {miso_input, shift_reg[SPI_WORD_LENGTH-1:1]};  // right-shift
        else
            shift_reg <= {shift_reg[SPI_WORD_LENGTH-2:0], miso_input};  // left-shift
    end
end

// ============================================================
// mosi_output -- serial data to the slave.
// ============================================================
always @(posedge clk_50MHz or negedge rst_n) begin
    /*
    Purpose: Drives MOSI with the correct bit at the correct time
    for all four SPI modes and both bit orders.

    CPHA=0: The first bit must be on MOSI BEFORE the leading SCLK
            edge so the slave can capture it.  We therefore load the
            MSB (or LSB for LSB_FIRST) of tx_data at xfer_start.
            Subsequent bits are updated on each sclk_shift.
            IMPORTANT GUARD for CPHA=0: sclk_shift and cnt_edge are
            the SAME edge type.  When last_bit is true, sclk_shift
            coincides with xfer_done -- the transfer is ending, so
            we must NOT overwrite MOSI.  Condition: !last_bit.

    CPHA=1: No data is placed on MOSI at xfer_start.  The first bit
            is driven on the FIRST sclk_shift edge (half a SCLK period
            after CS goes low).  The slave captures on the following
            (opposite) edge.  Because xfer_done fires on cnt_edge
            which is the OPPOSITE type from sclk_shift for CPHA=1,
            sclk_shift is never the final event -- MOSI is updated on
            ALL sclk_shift pulses (no last_bit guard required).
    */
    if (!rst_n)
        mosi_output <= 1'b0;
    else if (xfer_start && (CPHA == 1'b0))
        // CPHA=0: put first bit on MOSI the instant CS asserts.
        mosi_output <= LSB_FIRST ? tx_data[0] : tx_data[SPI_WORD_LENGTH-1];
    else if (sclk_shift)
        // CPHA=0: guard -- skip update when sclk_shift == xfer_done.
        // CPHA=1: no guard needed -- sclk_shift != cnt_edge always.
        if (CPHA == 1'b1 || !last_bit)
            mosi_output <= mosi_bit;
end

// ============================================================
// rx_data -- captured received word.
// ============================================================
always @(posedge clk_50MHz or negedge rst_n) begin
    /*
    Purpose: Clears rx_data on reset and captures the received word
    at the end of every transfer.

    IMPORTANT: For CPHA=1 (Modes 1 and 3), sclk_sample and xfer_done
    coincide on the same clock edge.  Verilog non-blocking semantics
    evaluate all RHS values BEFORE any assignment takes effect, so
    "rx_data <= shift_reg" would capture shift_reg WITHOUT the last
    MISO bit (which is being shifted in simultaneously by the
    sclk_sample branch of the shift_reg always block).

    Fix: when xfer_done and sclk_sample occur together, directly
    compute the final receive word by inlining the last MISO shift,
    exactly mirroring the shift_reg update expression.
    */
    if (!rst_n)
        rx_data <= {SPI_WORD_LENGTH{1'b0}};
    else if (xfer_done) begin
        if (sclk_sample)
            // Last sample coincides with xfer_done (CPHA=1 modes):
            // inline the final MISO bit so rx_data is complete.
            rx_data <= LSB_FIRST
                ? {miso_input, shift_reg[SPI_WORD_LENGTH-1:1]}
                : {shift_reg[SPI_WORD_LENGTH-2:0], miso_input};
        else
            // Last sample already occurred before xfer_done (CPHA=0 modes):
            // shift_reg is fully updated; capture it directly.
            rx_data <= shift_reg;
    end
end

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule
