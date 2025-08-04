import numpy as np
import matplotlib.pyplot as plt

class InverseParkVisualizer:
    def __init__(self, foc_common):
        self.foc_common = foc_common

    def run_sweep(self, d, q, resolution=360, noise_sigma=0.0):
        thetas = np.linspace(0, 2*np.pi, resolution)

        d_array = np.full(resolution, d)
        q_array = np.full(resolution, q)

        if noise_sigma > 0.0:
            d_array += np.random.normal(0, noise_sigma, size=resolution)
            q_array += np.random.normal(0, noise_sigma, size=resolution)

        alphas, betas = [], []
        for theta, d_val, q_val in zip(thetas, d_array, q_array):
            alpha, beta = self.foc_common.inverse_park(d_val, q_val, theta)
            alphas.append(alpha)
            betas.append(beta)

        return thetas, np.array(alphas), np.array(betas)

    def plot_alpha_beta(self, alphas, betas):
        plt.figure(figsize=(6,6))
        plt.plot(alphas, betas, label="Inverse Park α-β")
        plt.title("Inverse Park α-β Plane")
        plt.xlabel("Alpha (α)")
        plt.ylabel("Beta (β)")
        plt.axis("equal")
        plt.grid()
        plt.legend()
        plt.tight_layout()
        plt.show()
