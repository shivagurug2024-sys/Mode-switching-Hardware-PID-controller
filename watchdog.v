`timescale 1ns/1ps

module watchdog #(
    parameter TIMEOUT = 50000 // Programmable timeout in clock cycles
)(
    input  wire clk,
    input  wire rst_n,
    input  wire spi_data_valid, // Pulse when new IMU data arrives
    output reg  failsafe_trigger
);

    reg [31:0] counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
            failsafe_trigger <= 1'b0;
        end else begin
            if (spi_data_valid) begin
                counter <= 0; // Reset timer on valid data
                failsafe_trigger <= 1'b0;
            end else if (counter < TIMEOUT) begin
                counter <= counter + 1;
            end else begin
                failsafe_trigger <= 1'b1; // Trigger failsafe
            end
        end
    end

endmodule
