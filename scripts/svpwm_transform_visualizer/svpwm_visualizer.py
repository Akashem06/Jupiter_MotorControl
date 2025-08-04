import numpy as np
import matplotlib.pyplot as plt

class SVPWMVisualizer:
    def __init__(self, foc_common):
        self.foc_common = foc_common

    def run_sweep(self, vref, resolution=360):
        thetas = np.linspace(0, 2*np.pi, resolution)
        duties_A, duties_B, duties_C = [], [], []

        for theta in thetas:
            a, b, c = self.foc_common.svpwm_generate(theta, vref)
            duties_A.append(a)
            duties_B.append(b)
            duties_C.append(c)

        return thetas, np.array(duties_A), np.array(duties_B), np.array(duties_C)

    def plot_phases(self, thetas, duties_A, duties_B, duties_C):
        vbus = 1.0
        vA = (duties_A - 0.5) * vbus
        vB = (duties_B - 0.5) * vbus
        vC = (duties_C - 0.5) * vbus

        plt.figure(figsize=(10, 6))
        plt.plot(thetas, vA, label="Phase A")
        plt.plot(thetas, vB, label="Phase B")
        plt.plot(thetas, vC, label="Phase C")
        plt.title("SVPWM Phase Voltages")
        plt.xlabel("Electrical angle (rad)")
        plt.ylabel("Normalized Phase Voltage")
        plt.grid()
        plt.legend()
        plt.tight_layout()
        plt.show()
