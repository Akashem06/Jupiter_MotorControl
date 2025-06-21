import numpy as np

def generate_noisy_phases(thetas, sigma=0.05):
    ia = np.sin(thetas) + np.random.normal(0, sigma, size=thetas.shape)
    ib = np.sin(thetas - 2*np.pi/3) + np.random.normal(0, sigma, size=thetas.shape)
    ic = np.sin(thetas + 2*np.pi/3) + np.random.normal(0, sigma, size=thetas.shape)
    return ia, ib, ic
