`timescale 1ns/1ps

module top_controller (
    input  wire clk,
    input  wire rst_n,
    input  wire [1:0] mode_sel,
    input  wire spi_data_valid,
    input  wire signed [15:0] setpoint_att,
    input  wire signed [15:0] meas_att,
    input  wire signed [15:0] meas_rate,
    input  wire signed [15:0] kp_att, ki_att, kd_att,
    input  wire signed [15:0] kp_rate, ki_rate, kd_rate,
    output wire signed [15:0] pwm_out,
    output wire saturated
);

    wire failsafe_trigger;
    wire [1:0] active_mode;

    // Instantiate Watchdog
    watchdog #(.TIMEOUT(50000)) wd (
        .clk(clk),
        .rst_n(rst_n),
        .spi_data_valid(spi_data_valid),
        .failsafe_trigger(failsafe_trigger)
    );

    // Override mode if watchdog triggers
    assign active_mode = failsafe_trigger ? 2'b11 : mode_sel;

    // Instantiate PID Core
    pid_core #(.DATA_WIDTH(16), .FRAC_BITS(12)) pid (
        .clk(clk),
        .rst_n(rst_n),
        .mode(active_mode),
        .setpoint_attitude(setpoint_att),
        .meas_attitude(meas_att),
        .meas_rate(meas_rate),
        .kp_att(kp_att), .ki_att(ki_att), .kd_att(kd_att),
        .kp_rate(kp_rate), .ki_rate(ki_rate), .kd_rate(kd_rate),
        .pwm_out(pwm_out),
        .saturated(saturated)
    );

endmodule
