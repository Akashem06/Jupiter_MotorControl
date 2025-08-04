import numpy as np
import matplotlib.pyplot as plt

class PLLVisualizer:
    def __init__(self, pll_common):
        self.pll_common = pll_common

    @staticmethod
    def wrap_phase_error(error):
        """Wrap angle to [-π, π)."""
        return (error + np.pi) % (2 * np.pi) - np.pi

    def run_sweep(self, dt, total_time=5.0, freq=0.5, noise=0.05):
        """
        Simulates the PLL tracking a continuously increasing reference input.
        """
        num_steps = int(total_time / dt)
        time = np.linspace(0, total_time, num_steps)

        omega_input = 2 * np.pi * freq  # rad/s
        theta_vals = []
        omega_vals = []
        phase_errors = []
        thetas_ref = []

        for t in time:
            # Let theta_ref increase continuously (simulate rotating shaft)
            theta_ref = omega_input * t
            theta_est = self.pll_common.state.theta

            # Compute wrapped phase error with added noise
            phase_error = self.wrap_phase_error(theta_ref - theta_est)
            noisy_error = self.wrap_phase_error(phase_error + np.random.normal(0, noise))

            # Update PLL
            theta_out, omega_out = self.pll_common.update(noisy_error, dt)

            thetas_ref.append(theta_ref)
            theta_vals.append(theta_out)
            omega_vals.append(omega_out)

            # Compute wrapped tracking error
            track_error = self.wrap_phase_error(theta_ref - theta_out)
            phase_errors.append(track_error)

        return time, np.array(omega_vals), np.array(theta_vals), np.array(phase_errors), np.array(thetas_ref)


    def plot_pll(self, time, omega_vals, theta_vals, phase_errors, thetas_ref=None):
        unwrapped_error = np.unwrap(phase_errors)
        rms_error = np.sqrt(np.mean(unwrapped_error ** 2))
        print(f"RMS Phase Tracking Error: {rms_error:.4f} rad")

        plt.figure(figsize=(10, 8))

        plt.subplot(4, 1, 1)
        plt.plot(time, theta_vals, label="PLL Output Angle (theta)")
        if thetas_ref is not None:
            plt.plot(time, thetas_ref, '--', label="Reference Angle (theta_ref)")
        plt.title("PLL Output Angle over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Theta (rad)")
        plt.grid()
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.plot(time, omega_vals, label="PLL Output Angular Velocity (omega)")
        plt.title("PLL Angular Velocity over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Omega (rad/s)")
        plt.grid()
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.plot(time, phase_errors, label="Phase Error")
        plt.title("Phase Error over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Phase Error (rad)")
        plt.grid()
        plt.legend()

        plt.subplot(4, 1, 4)
        plt.plot(time, unwrapped_error, label="Unwrapped Phase Error")
        plt.xlabel("Time (s)")
        plt.ylabel("Unwrapped Error (rad)")
        plt.grid()
        plt.legend()

        plt.tight_layout()
        plt.show()
