`timescale 1ns/1ps

module tb_top_controller;
    reg clk, rst_n;
    reg [1:0] mode_sel;
    reg spi_data_valid;
    reg signed [15:0] setpoint_att;
    reg signed [15:0] meas_att, meas_rate;
    reg signed [15:0] kp_att, ki_att, kd_att;
    reg signed [15:0] kp_rate, ki_rate, kd_rate;
    
    wire signed [15:0] pwm_out;
    wire saturated;

    // Internal plant simulation registers
    reg signed [31:0] plant_attitude;
    reg signed [31:0] plant_rate;

    top_controller dut (
        .clk(clk), .rst_n(rst_n), .mode_sel(mode_sel),
        .spi_data_valid(spi_data_valid),
        .setpoint_att(setpoint_att), .meas_att(meas_att), .meas_rate(meas_rate),
        .kp_att(kp_att), .ki_att(ki_att), .kd_att(kd_att),
        .kp_rate(kp_rate), .ki_rate(ki_rate), .kd_rate(kd_rate),
        .pwm_out(pwm_out), .saturated(saturated)
    );

    initial begin
        clk = 0; forever #5 clk = ~clk; // 100MHz
    end

    // --- Drone Physics Plant ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            plant_attitude <= 0;
            plant_rate <= 0;
        end else begin
            plant_rate <= plant_rate + (pwm_out >>> 2) - (plant_rate >>> 4);
            plant_attitude <= plant_attitude + (plant_rate >>> 6);
        end
    end

    // Feed the plant state back into the controller as sensor data
    always @(posedge clk) begin
        // Scale down the plant to match Q4.12 format
        meas_att  <= plant_attitude[21:6];
        meas_rate <= plant_rate[21:6];
        spi_data_valid <= 1'b1; 
    end

    initial begin
        // 1. Initialize
        rst_n = 0; mode_sel = 2'b01; // Stabilize Mode
        setpoint_att = 0;
        kp_att  = 16'sh0400;  // 1.0
        ki_att  = 16'sh0000;
        kd_att  = 16'sh0000;
        kp_rate = 16'sh0800;  // 2.0
        ki_rate = 16'sh0020;  // 0.03125
        kd_rate = 16'sh0100;  // 0.0625
        
        #20 rst_n = 1;

        // 2. Apply Step Input
        #100 setpoint_att = 16'sh1000; // Command 1.0 (Q4.12)
        
        // 3. Let it stabilize
        #50000;

        // 4. Trigger Failsafe (Manual)
        mode_sel = 2'b11; 
        #1000;
        
        // 5. Return to Stabilize
        mode_sel = 2'b01;
        #50000;

        $finish;
    end
endmodule
