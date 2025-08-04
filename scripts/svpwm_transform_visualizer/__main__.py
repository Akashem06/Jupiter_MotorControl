import argparse
import numpy as np

from svpwm_transform_interface  import SvpwmTransformInterface
from svpwm_visualizer           import SVPWMVisualizer
from clarke_phase_visualizer    import ClarkeVisualizer
from park_visualizer            import ParkVisualizer
from inverse_park_visualizer    import InverseParkVisualizer
from noise_generator            import generate_noisy_phases

def main():
    parser = argparse.ArgumentParser(description="SVPWM and Parke/Clarke Transform Visualization Framework")
    parser.add_argument("--vref", type=float, default=0.8)
    parser.add_argument("--resolution", type=int, default=360)
    parser.add_argument("--noise", type=float, default=0.05)
    parser.add_argument("--lib", type=str, default=None)
    args = parser.parse_args()

    svpwm_transform_interface = SvpwmTransformInterface(lib_path=args.lib)

    # SVPWM
    svpwm_viz = SVPWMVisualizer(svpwm_transform_interface)
    thetas, dA, dB, dC = svpwm_viz.run_sweep(args.vref, args.resolution)
    svpwm_viz.plot_phases(thetas, dA, dB, dC)

    thetas = np.linspace(0, 2*np.pi, args.resolution)
    ia, ib, ic = generate_noisy_phases(thetas, sigma=args.noise)

    # Clarke 3-phase
    clarke_viz = ClarkeVisualizer(svpwm_transform_interface)
    alphas, betas = clarke_viz.run_3phase_sweep(thetas, ia, ib, ic)
    clarke_viz.plot_alpha_beta(alphas, betas, "Clarke Transform 3-phase")

    # Clarke 2-phase
    alphas, betas = clarke_viz.run_2phase_sweep(thetas, ia, ib)
    clarke_viz.plot_alpha_beta(alphas, betas, "Clarke Transform 2-phase")

    # Park
    park_viz = ParkVisualizer(svpwm_transform_interface)
    thetas, d_vals, q_vals = park_viz.run_sweep(alpha=1.0, beta=0.0, resolution=args.resolution, noise_sigma=args.noise)
    park_viz.plot_d_q(d_vals, q_vals)

    # Inverse Park
    inv_park_viz = InverseParkVisualizer(svpwm_transform_interface)
    thetas, alphas, betas = inv_park_viz.run_sweep(d=1.0, q=0.0, resolution=args.resolution, noise_sigma=args.noise)
    inv_park_viz.plot_alpha_beta(alphas, betas)

if __name__ == "__main__":
    main()
