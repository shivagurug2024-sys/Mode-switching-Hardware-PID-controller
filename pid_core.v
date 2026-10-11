`timescale 1ns/1ps

module pid_core #(
    parameter DATA_WIDTH = 16,
    parameter FRAC_BITS  = 12
)(
    input  wire                   clk,
    input  wire                   rst_n,
    
    // Mode and Inputs
    input  wire [1:0]             mode,        // 0=Acro, 1=Stabilize, 2=AltHold, 3=Failsafe
    input  wire signed [DATA_WIDTH-1:0] setpoint_attitude,
    input  wire signed [DATA_WIDTH-1:0] meas_attitude,
    input  wire signed [DATA_WIDTH-1:0] meas_rate,
    
    // Gain Banks
    input  wire signed [DATA_WIDTH-1:0] kp_att, ki_att, kd_att,
    input  wire signed [DATA_WIDTH-1:0] kp_rate, ki_rate, kd_rate,
    
    // Output
    output reg  signed [DATA_WIDTH-1:0] pwm_out,
    output reg                    saturated
);

    // --- Internal Signals ---
    wire signed [DATA_WIDTH-1:0] rate_setpoint;
    wire signed [DATA_WIDTH-1:0] rate_error;
    wire signed [DATA_WIDTH-1:0] attitude_error;
    
    reg  signed [DATA_WIDTH-1:0] integral_rate;
    reg  signed [DATA_WIDTH-1:0] prev_meas_rate;
    wire signed [DATA_WIDTH-1:0] derivative_rate;
    
    // Wide accumulators for Q4.12 multiplication
    wire signed [2*DATA_WIDTH-1:0] p_term_full;
    wire signed [2*DATA_WIDTH-1:0] i_term_full;
    wire signed [2*DATA_WIDTH-1:0] d_term_full;
    wire signed [2*DATA_WIDTH-1:0] sum_full;
    wire signed [DATA_WIDTH-1:0]   sum_scaled;
    
    // Bumpless Transfer Registers
    reg signed [DATA_WIDTH-1:0] last_pwm_out;
    reg [1:0] last_mode;
    reg bumpless_calc_done;

    // --- 1. Outer Loop (Attitude) ---
    assign attitude_error = setpoint_attitude - meas_attitude;
    
    // Simple P controller for outer loop
    wire signed [2*DATA_WIDTH-1:0] att_p_term_full = kp_att * attitude_error;
    assign rate_setpoint = att_p_term_full >>> FRAC_BITS; 

    // --- 2. Inner Loop (Rate) ---
    // In Acro mode (00), setpoint comes directly from pilot. Otherwise from outer loop.
    wire signed [DATA_WIDTH-1:0] active_rate_setpoint = (mode == 2'b00) ? setpoint_attitude : rate_setpoint;
    
    assign rate_error = active_rate_setpoint - meas_rate;
    assign derivative_rate = meas_rate - prev_meas_rate;

    // --- 3. PID Terms Calculation ---
    assign p_term_full = kp_rate * rate_error;
    assign i_term_full = ki_rate * integral_rate;
    assign d_term_full = kd_rate * derivative_rate;
    assign sum_full = p_term_full + i_term_full + d_term_full;
    assign sum_scaled = sum_full >>> FRAC_BITS;

    // --- 4. Sequential Logic: Bumpless Transfer & Anti-Windup ---
    wire signed [2*DATA_WIDTH-1:0] p_d_sum = p_term_full + d_term_full;
    wire signed [2*DATA_WIDTH-1:0] p_d_scaled = p_d_sum >>> FRAC_BITS;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_out           <= 16'sd0;
            integral_rate     <= 16'sd0;
            prev_meas_rate    <= 16'sd0;
            saturated         <= 1'b0;
            last_pwm_out      <= 16'sd0;
            last_mode         <= 2'b00;
            bumpless_calc_done<= 1'b0;
        end else begin
            prev_meas_rate <= meas_rate;
            last_mode      <= mode;

            // --- BUMPLESS TRANSFER LOGIC (Simplified for Hardware) ---
            if (mode != last_mode) begin
                integral_rate <= integral_rate; 
                bumpless_calc_done <= 1'b1;
            end else begin
                bumpless_calc_done <= 1'b0;
                
                // --- ANTI-WINDUP (Clamping) ---
                if ((sum_scaled > 16'sh7FFF) || (sum_scaled < 16'sh8000)) begin
                    integral_rate <= integral_rate; // Freeze integral on saturation
                    saturated <= 1'b1;
                end else begin
                    integral_rate <= integral_rate + rate_error; // Accumulate
                    saturated <= 1'b0;
                end
            end

            // --- OUTPUT SATURATION & FAILSAFE ---
            if (mode == 2'b11) begin // Failsafe Mode
                pwm_out <= 16'sd0;
                last_pwm_out <= 16'sd0;
            end else if (sum_scaled > 16'sh7FFF) begin
                pwm_out <= 16'sh7FFF;
                last_pwm_out <= 16'sh7FFF;
            end else if (sum_scaled < 16'sh8000) begin
                pwm_out <= 16'sh8000;
                last_pwm_out <= 16'sh8000;
            end else begin
                pwm_out <= sum_scaled;
                last_pwm_out <= sum_scaled;
            end
        end
    end

endmodule
