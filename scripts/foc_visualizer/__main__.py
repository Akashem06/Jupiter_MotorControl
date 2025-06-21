import argparse
import numpy as np

from foc_common_interface       import FOCCommon
from svpwm_visualizer           import SVPWMVisualizer
from clarke_phase_visualizer    import ClarkeVisualizer
from park_visualizer            import ParkVisualizer
from inverse_park_visualizer    import InverseParkVisualizer
from noise_generator            import generate_noisy_phases

def main():
    parser = argparse.ArgumentParser(description="FOC Visualization Framework")
    parser.add_argument("--vref", type=float, default=0.8)
    parser.add_argument("--resolution", type=int, default=360)
    parser.add_argument("--noise", type=float, default=0.05)
    parser.add_argument("--lib", type=str, default=None)
    args = parser.parse_args()

    foc = FOCCommon(lib_path=args.lib)

    # SVPWM
    svpwm_viz = SVPWMVisualizer(foc)
    thetas, dA, dB, dC = svpwm_viz.run_sweep(args.vref, args.resolution)
    svpwm_viz.plot_phases(thetas, dA, dB, dC)

    thetas = np.linspace(0, 2*np.pi, args.resolution)
    ia, ib, ic = generate_noisy_phases(thetas, sigma=args.noise)

    # Clarke 3-phase
    clarke_viz = ClarkeVisualizer(foc)
    alphas, betas = clarke_viz.run_3phase_sweep(thetas, ia, ib, ic)
    clarke_viz.plot_alpha_beta(alphas, betas, "Clarke Transform 3-phase")

    # Clarke 2-phase
    alphas, betas = clarke_viz.run_2phase_sweep(thetas, ia, ib)
    clarke_viz.plot_alpha_beta(alphas, betas, "Clarke Transform 2-phase")

    # Park
    park_viz = ParkVisualizer(foc)
    thetas, d_vals, q_vals = park_viz.run_sweep(alpha=1.0, beta=0.0, resolution=args.resolution, noise_sigma=args.noise)
    park_viz.plot_d_q(d_vals, q_vals)

    # Inverse Park
    inv_park_viz = InverseParkVisualizer(foc)
    thetas, alphas, betas = inv_park_viz.run_sweep(d=1.0, q=0.0, resolution=args.resolution, noise_sigma=args.noise)
    inv_park_viz.plot_alpha_beta(alphas, betas)

if __name__ == "__main__":
    main()
