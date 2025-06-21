import numpy as np
import matplotlib.pyplot as plt

class ParkVisualizer:
    def __init__(self, foc_common):
        self.foc_common = foc_common

    def run_sweep(self, alpha, beta, resolution=360, noise_sigma=0.0):
        thetas = np.linspace(0, 2*np.pi, resolution)

        alpha_array = np.full(resolution, alpha)
        beta_array = np.full(resolution, beta)

        if noise_sigma > 0.0:
            alpha_array += np.random.normal(0, noise_sigma, size=resolution)
            beta_array += np.random.normal(0, noise_sigma, size=resolution)

        d_vals, q_vals = [], []
        for theta, a, b in zip(thetas, alpha_array, beta_array):
            d, q = self.foc_common.park(a, b, theta)
            d_vals.append(d)
            q_vals.append(q)

        return thetas, np.array(d_vals), np.array(q_vals)

    def plot_d_q(self, d_vals, q_vals):
        plt.figure(figsize=(6,6))
        plt.plot(d_vals, q_vals, label="Park d-q")
        plt.title("Park Transform d-q Plane")
        plt.xlabel("d-axis")
        plt.ylabel("q-axis")
        plt.axis("equal")
        plt.grid()
        plt.legend()
        plt.tight_layout()
        plt.show()
