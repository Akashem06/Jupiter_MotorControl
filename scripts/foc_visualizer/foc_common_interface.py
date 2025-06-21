import ctypes
from pathlib import Path

class FOCCommon:
    def __init__(self, lib_path=None):
        if lib_path is None:
            lib_path = Path(__file__).parent.parent.parent / "build" / "libfoc_common.so"
        lib_path = lib_path.resolve()
        print(f"Loading FOC common shared library from: {lib_path}")

        self.lib = ctypes.CDLL(str(lib_path))

        # svpwm_generate
        self.lib.svpwm_generate.argtypes = [
            ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float)
        ]
        self.lib.svpwm_generate.restype = ctypes.c_int

        # clarke_transform_2phase
        self.lib.clarke_transform_2phase.argtypes = [
            ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)
        ]
        self.lib.clarke_transform_2phase.restype = ctypes.c_int

        # clarke_transform_3phase
        self.lib.clarke_transform_3phase.argtypes = [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)
        ]
        self.lib.clarke_transform_3phase.restype = ctypes.c_int

        # park_transform
        self.lib.park_transform.argtypes = [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)
        ]
        self.lib.park_transform.restype = ctypes.c_int

        # inverse_park_transform
        self.lib.inverse_park_transform.argtypes = [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)
        ]
        self.lib.inverse_park_transform.restype = ctypes.c_int

    # SVPWM generation
    def svpwm_generate(self, theta_e, vref_mag):
        dA = ctypes.c_float()
        dB = ctypes.c_float()
        dC = ctypes.c_float()
        err = self.lib.svpwm_generate(
            ctypes.c_float(theta_e), ctypes.c_float(vref_mag),
            ctypes.byref(dA), ctypes.byref(dB), ctypes.byref(dC)
        )
        if err != 0:
            raise RuntimeError(f"SVPWM error code: {err}")
        return dA.value, dB.value, dC.value

    # Clarke 2-phase
    def clarke_2phase(self, ia, ib):
        alpha = ctypes.c_float()
        beta = ctypes.c_float()
        err = self.lib.clarke_transform_2phase(
            ctypes.c_float(ia), ctypes.c_float(ib),
            ctypes.byref(alpha), ctypes.byref(beta)
        )
        if err != 0:
            raise RuntimeError(f"Clarke 2-phase error code: {err}")
        return alpha.value, beta.value

    # Clarke 3-phase
    def clarke_3phase(self, ia, ib, ic):
        alpha = ctypes.c_float()
        beta = ctypes.c_float()
        err = self.lib.clarke_transform_3phase(
            ctypes.c_float(ia), ctypes.c_float(ib), ctypes.c_float(ic),
            ctypes.byref(alpha), ctypes.byref(beta)
        )
        if err != 0:
            raise RuntimeError(f"Clarke 3-phase error code: {err}")
        return alpha.value, beta.value

    # Park transform
    def park(self, alpha, beta, theta):
        d = ctypes.c_float()
        q = ctypes.c_float()
        err = self.lib.park_transform(
            ctypes.c_float(alpha), ctypes.c_float(beta), ctypes.c_float(theta),
            ctypes.byref(d), ctypes.byref(q)
        )
        if err != 0:
            raise RuntimeError(f"Park transform error code: {err}")
        return d.value, q.value

    # Inverse Park transform
    def inverse_park(self, d, q, theta):
        alpha = ctypes.c_float()
        beta = ctypes.c_float()
        err = self.lib.inverse_park_transform(
            ctypes.c_float(d), ctypes.c_float(q), ctypes.c_float(theta),
            ctypes.byref(alpha), ctypes.byref(beta)
        )
        if err != 0:
            raise RuntimeError(f"Inverse Park transform error code: {err}")
        return alpha.value, beta.value
