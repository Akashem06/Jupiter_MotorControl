import ctypes
from pathlib import Path

class PLLConfig(ctypes.Structure):
    _fields_ = [
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("max_omega", ctypes.c_float),
        ("enable_filtering", ctypes.c_bool),
        ("filter_alpha", ctypes.c_float),
    ]


class PLLState(ctypes.Structure):
    _fields_ = [
        ("theta", ctypes.c_float),
        ("omega", ctypes.c_float),
        ("integrator", ctypes.c_float),
        ("max_error", ctypes.c_float),
        ("is_converged", ctypes.c_bool),
        ("prev_error", ctypes.c_float),
        ("cfg", ctypes.POINTER(PLLConfig))
    ]


class PLLInterface:
    def __init__(self, lib_path=None):
        if lib_path is None:
            lib_path = Path(__file__).parent.parent.parent / "build" / "libpll_python_visualizer.so"
        lib_path = lib_path.resolve()

        self.lib = ctypes.CDLL(str(lib_path))

        self.lib.pll_init.argtypes = [ctypes.POINTER(PLLState), ctypes.POINTER(PLLConfig)]
        self.lib.pll_init.restype = ctypes.c_int

        self.lib.pll_update.argtypes = [
            ctypes.POINTER(PLLState),
            ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float)
        ]
        self.lib.pll_update.restype = ctypes.c_int

        self.UTILS_OK = 0
        self.UTILS_INVALID_ARGS = 1

    def init_pll(self, pll_state: PLLState, pll_config: PLLConfig):
        result = self.lib.pll_init(ctypes.byref(pll_state), ctypes.byref(pll_config))
        if result != self.UTILS_OK:
            raise RuntimeError(f"pll_init failed with error code {result}")

    def update_pll(self, pll_state: PLLState, phase_error: float, dt: float):
        theta_out = ctypes.c_float()
        omega_out = ctypes.c_float()
        result = self.lib.pll_update(
            ctypes.byref(pll_state),
            ctypes.c_float(phase_error),
            ctypes.c_float(dt),
            ctypes.byref(theta_out),
            ctypes.byref(omega_out)
        )
        if result != self.UTILS_OK:
            raise RuntimeError(f"pll_update failed with error code {result}")
        return theta_out.value, omega_out.value

class DefaultPLL:
    def __init__(self, lib_path=None):
        self.interface = PLLInterface(lib_path)

        self.config = PLLConfig(
            kp=1.0,
            ki=0.8,
            max_omega=100.0,
            enable_filtering=True,
            filter_alpha=0.5
        )

        self.state = PLLState()
        self.interface.init_pll(self.state, self.config)

    def update(self, phase_error: float, dt: float):
        """
        Run one update step of the PLL.
        Returns:
            (theta, omega): tuple of float values representing the PLL's estimated angle and angular velocity.
        """
        return self.interface.update_pll(self.state, phase_error, dt)
