import mmap
import struct
import numpy as np
import time
import os
import signal
import subprocess

class MotorSimulationBridge:
    def __init__(self):
        self.running = False
        self.sharedmem_name = "/motor_sim_data"
        self.sharedmem_size = 4096

        self.data_format = struct.Struct(
            '=d' +          # Timestamp
            '3d' +          # 3 Phase voltage
            '3d' +          # 3 Phase current
            'd' +           # Rotor angle
            'd' +           # Rotor speed
            'I'             # Status flags
        )
