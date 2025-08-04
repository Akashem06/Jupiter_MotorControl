import numpy as np
import matplotlib.pyplot as plt

class ClarkeVisualizer:
    def __init__(self, foc_common):
        self.foc_common = foc_common

    def run_3phase_sweep(self, thetas, ia, ib, ic):
        """
        Compute Clarke transform for 3-phase inputs
        """
        alphas, betas = [], []
        for ia_val, ib_val, ic_val in zip(ia, ib, ic):
            alpha, beta = self.foc_common.clarke_3phase(ia_val, ib_val, ic_val)
            alphas.append(alpha)
            betas.append(beta)
        return np.array(alphas), np.array(betas)

    def run_2phase_sweep(self, thetas, ia, ib):
        """
        Compute Clarke transform for 2-phase inputs
        """
        alphas, betas = [], []
        for ia_val, ib_val in zip(ia, ib):
            alpha, beta = self.foc_common.clarke_2phase(ia_val, ib_val)
            alphas.append(alpha)
            betas.append(beta)
        return np.array(alphas), np.array(betas)

    def plot_alpha_beta(self, alphas, betas, title="Clarke α-β Plane"):
        plt.figure(figsize=(6,6))
        plt.plot(alphas, betas, label="α-β Trajectory")
        plt.title(title)
        plt.xlabel("Alpha (α)")
        plt.ylabel("Beta (β)")
        plt.axis("equal")
        plt.grid()
        plt.legend()
        plt.tight_layout()
        plt.show()
