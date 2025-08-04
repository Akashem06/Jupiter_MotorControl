import argparse
import numpy as np

from pll_interface              import PLLInterface, DefaultPLL
from pll_visualizer             import PLLVisualizer
from pll_sweep_gains            import calculate_optimal_pll_gains

def main():
    parser = argparse.ArgumentParser(description="PLL Visualization Framework")
    parser.add_argument("--dt", type=float, default=0.001)
    parser.add_argument("--lib", type=str, default=None)
    args = parser.parse_args()

    [optimal_ki, optimal_kp] = calculate_optimal_pll_gains()

    pll = DefaultPLL(lib_path=args.lib)

    pll.config.kp = optimal_kp
    pll.config.ki = optimal_ki
    pll.config.max_omega = 100.0
    pll.config.enable_filtering = True
    pll.config.filter_alpha = 0.5
    pll.interface.init_pll(pll.state, pll.config)

    pll_viz = PLLVisualizer(pll)

    time, omega_vals, theta_vals, phase_errors, thetas_ref = pll_viz.run_sweep(
        dt=args.dt, total_time=5.0, freq=0.5, noise=0.02
    )

    pll_viz.plot_pll(time, omega_vals, theta_vals, phase_errors, thetas_ref)


if __name__ == "__main__":
    main()
