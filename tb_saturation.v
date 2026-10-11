`timescale 1ns/1ps

module tb_saturation;
    reg clk, rst_n;
    reg [1:0] mode_sel;
    reg spi_data_valid;
    reg signed [15:0] setpoint_att;
    reg signed [15:0] meas_att, meas_rate;
    reg signed [15:0] kp_att, ki_att, kd_att;
    reg signed [15:0] kp_rate, ki_rate, kd_rate;

    wire signed [15:0] pwm_out;
    wire saturated;

    top_controller dut (
        .clk(clk), .rst_n(rst_n), .mode_sel(mode_sel),
        .spi_data_valid(spi_data_valid),
        .setpoint_att(setpoint_att), .meas_att(meas_att), .meas_rate(meas_rate),
        .kp_att(kp_att), .ki_att(ki_att), .kd_att(kd_att),
        .kp_rate(kp_rate), .ki_rate(ki_rate), .kd_rate(kd_rate),
        .pwm_out(pwm_out), .saturated(saturated)
    );

    // Clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

        initial begin
        // Initialize
        rst_n = 0; mode_sel = 2'b00; // Acro mode (rate-only)
        spi_data_valid = 1;
        setpoint_att = 0; meas_att = 0; meas_rate = 0;

        // Very high rate gains -> force saturation
        kp_att  = 16'sh0000;
        ki_att  = 16'sh0000;
        kd_att  = 16'sh0000;
        kp_rate = 16'sh7FFF;   // Huge P gain
        ki_rate = 16'sh7FFF;   // Huge I gain
        kd_rate = 16'sh7FFF;   // Huge D gain

        #20 rst_n = 1;

        // In Acro mode, setpoint_att = rate setpoint directly
        // Command max rate (32767) with zero feedback -> huge error
        #100 setpoint_att = 16'sh7FFF;  // Max rate command (32,767)
        meas_rate = 0;                   // No feedback

        // Hold. Integral winds up. Saturation kicks in.
        #50000;

        $finish;
    end
endmodule
