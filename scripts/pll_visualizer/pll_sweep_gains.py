import argparse
import numpy as np
import matplotlib.pyplot as plt

from pll_interface              import PLLInterface, DefaultPLL
from pll_visualizer             import PLLVisualizer

def sweep_pll_gains_c_wrapper(kp_vals, ki_vals, dt=0.01, total_time=5.0, freq=0.5, noise=0.05, lib_path=None):
    rms_errors = np.zeros((len(kp_vals), len(ki_vals)))

    for i, kp in enumerate(kp_vals):
        for j, ki in enumerate(ki_vals):
            pll = DefaultPLL(lib_path=lib_path)
            pll.config.kp = kp
            pll.config.ki = ki
            pll.config.max_omega = 100.0
            pll.config.enable_filtering = True
            pll.config.filter_alpha = 0.5
            pll.interface.init_pll(pll.state, pll.config)

            vis = PLLVisualizer(pll)
            t, omega, theta, phase_err, theta_ref = vis.run_sweep(dt, total_time, freq, noise)

            unwrapped_error = np.unwrap(phase_err)
            rms_error = np.sqrt(np.mean(unwrapped_error**2))
            rms_errors[i, j] = rms_error

    return rms_errors

def calculate_optimal_pll_gains():
    # Define ranges
    kp_range = np.linspace(0.5, 5.0, 10)
    ki_range = np.linspace(0.2, 3.0, 10)

    # Run sweep
    rms_matrix = sweep_pll_gains_c_wrapper(kp_range, ki_range)

    # Plot heatmap
    plt.figure(figsize=(8, 6))
    plt.imshow(rms_matrix, origin='lower', aspect='auto',
            extent=[ki_range[0], ki_range[-1], kp_range[0], kp_range[-1]],
            cmap='viridis')
    plt.colorbar(label="RMS Phase Error (rad)")
    plt.xlabel("ki")
    plt.ylabel("kp")
    plt.title("PLL Gain Sweep - RMS Phase Error")
    plt.tight_layout()
    plt.show()

    # Report best result
    min_idx = np.unravel_index(np.argmin(rms_matrix), rms_matrix.shape)
    best_kp = kp_range[min_idx[0]]
    best_ki = ki_range[min_idx[1]]
    best_rms = rms_matrix[min_idx]

    print(f"\nBest Gains: kp = {best_kp:.3f}, ki = {best_ki:.3f} → RMS Error = {best_rms:.4f} rad")

    return {best_kp, best_ki}
