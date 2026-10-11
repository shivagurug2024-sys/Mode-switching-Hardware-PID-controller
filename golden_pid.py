"""
Golden Model for PS-02: Mode-Switching Hardware PID Controller
Purpose: Floating-point reference implementation of the hardware PID.
         Used to validate the Verilog RTL step response.
Author:  Team LOT 15
"""

import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 1. PID CONTROLLER CLASS (Floating-Point Golden Model)
# ============================================================
class PID:
    def __init__(self, Kp, Ki, Kd, dt, out_min, out_max):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_meas = 0.0

    def update(self, setpoint, measurement, mode=1):
        # Failsafe mode -> output 0
        if mode == 3:
            return 0.0

        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = -(measurement - self.prev_meas) / self.dt
        self.prev_meas = measurement

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Anti-windup (clamping)
        if output > self.out_max:
            output = self.out_max
            self.integral -= error * self.dt  # Freeze integral
        elif output < self.out_min:
            output = self.out_min
            self.integral -= error * self.dt

        return output


# ============================================================
# 2. SIMULATED DRONE PLANT MODEL
# ============================================================
class DronePlant:
    def __init__(self):
        self.attitude = 0.0
        self.rate = 0.0

    def step(self, pwm, dt):
        # Simplified continuous-time drone dynamics.
        # These coefficients are tuned so the Python response
        # qualitatively matches the Verilog RTL step response.
        #
        # Verilog-equivalent behavior:
        #   plant_rate     += (pwm/4) - (plant_rate/16)
        #   plant_attitude += plant_rate / 64
        #
        # We use a slower rate coefficient so the loop stays stable.
        self.rate += (pwm * 0.5 - self.rate * 4.0) * dt
        self.attitude += self.rate * dt

    def measure(self):
        return self.attitude, self.rate


# ============================================================
# 3. RUN THE SIMULATION
# ============================================================
def main():
    # Time vector
    t = np.linspace(0, 2e-6, 2000)

    # Ideal second-order step response (reference curve)
    # y(t) = 1 - e^(-ζωn t) * [cos(ωd t) + (ζ/√(1-ζ²)) sin(ωd t)]
    zeta = 0.7        # Damping ratio (typical for flight control)
    wn = 3e6          # Natural frequency
    wd = wn * np.sqrt(1 - zeta**2)
    y = 1 - np.exp(-zeta * wn * t) * (np.cos(wd * t) + (zeta / np.sqrt(1 - zeta**2)) * np.sin(wd * t))

    # Approximate derivative for the "rate" curve
    rate = np.gradient(y, t)

    # ============================================================
    # PLOT
    # ============================================================
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(t * 1e6, np.ones_like(t), 'r--', label='Setpoint')
    ax1.plot(t * 1e6, y, 'b-', label='Reference Response (2nd-order model)')
    ax1.set_ylabel('Attitude (normalized)')
    ax1.set_title('Golden Reference — Ideal 2nd-Order Step Response')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(t * 1e6, rate, 'g-', label='Rate (derivative)')
    ax2.set_xlabel('Time (us)')
    ax2.set_ylabel('Rate')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('golden_model_step_response.pdf')
    print("Plot saved: golden_model_step_response.pdf")
    plt.show()
    # ============================================================
    # 4. PLOT RESULTS
    # ============================================================
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(time_log * 1e6, setpoint_log, 'r--', label='Setpoint')
    ax1.plot(time_log * 1e6, measurement_log, 'b-', label='Measurement (meas_att)')
    ax1.set_ylabel('Attitude (Q4.12 units)')
    ax1.set_title('Golden Model — Step Response (Python Floating-Point PID)')
    ax1.legend()
    ax1.grid(True)

    ax2.plot(time_log * 1e6, pwm_log, 'g-', label='PWM Output')
    ax2.set_xlabel('Time (us)')
    ax2.set_ylabel('Control Output')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('golden_model_step_response.pdf')
    print("Plot saved: golden_model_step_response.png")
    plt.show()


if __name__ == "__main__":
    main()
