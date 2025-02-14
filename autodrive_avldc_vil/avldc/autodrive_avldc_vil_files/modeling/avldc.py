#!/usr/bin/env python

import numpy as np
from multiprocessing.shared_memory import SharedMemory
import struct

################################################################################

# Join an existing shared memory via its name

class PythonInterface:
    def __init__(self):
        self.inputs = {}
        self.outputs = {}
        
    def configure(self):
        self.inputs = {
            "PosX": {
                "type": "float",
                "value": -208.2397,
            },
            "PosY": {
                "type": "float",
                "value": -263.0914,
            },
            "PosZ": {
                "type": "float",
                "value": 342.0139,
            },
            "RotX": {
                "type": "float",
                "value": 0.0,
            },
            "RotY": {
                "type": "float",
                "value": 0.0,
            },
            "RotZ": {
                "type": "float",
                "value": 1.80205245,
            }
        }
        self.outputs = {
            "DTC": {
                "type": "float",
                "value": 0.0,
            },
            "AEB": {
                "type": "float",
                "value": 0.0,
            }
        }

    def initialize(self):
        pass

    def do_step(self, currentCommunicationPoint, communicationStepSize):
        shared_mem = SharedMemory(name='AutoDRIVE', create=False)
        try:
            shared_mem.buf[0:8] = struct.pack('d', self.inputs["PosX"]["value"])   # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[9:17] = struct.pack('d', self.inputs["PosY"]["value"])  # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[18:26] = struct.pack('d', self.inputs["PosZ"]["value"]) # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[27:35] = struct.pack('d', self.inputs["RotX"]["value"]) # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[36:44] = struct.pack('d', self.inputs["RotY"]["value"]) # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[45:53] = struct.pack('d', self.inputs["RotZ"]["value"]) # Pack the float to bytes ('d' is for double-precision (64-bit) float
            self.outputs["DTC"]["value"] = struct.unpack('d', shared_mem.buf[54:62])[0] # Unpack the bytes to float
            self.outputs["AEB"]["value"] = struct.unpack('d', shared_mem.buf[63:71])[0] # Unpack the bytes to float
            # Print data in shared memory
            # print("POSX: ", struct.unpack('d', shared_mem.buf[0:8])[0])   # Unpack the bytes to float
            # print("POSY: ", struct.unpack('d', shared_mem.buf[9:17])[0])  # Unpack the bytes to float
            # print("POSZ: ", struct.unpack('d', shared_mem.buf[18:26])[0]) # Unpack the bytes to float
            # print("ROTX: ", struct.unpack('d', shared_mem.buf[27:35])[0]) # Unpack the bytes to float
            # print("ROTY: ", struct.unpack('d', shared_mem.buf[36:44])[0]) # Unpack the bytes to float
            # print("ROTZ: ", struct.unpack('d', shared_mem.buf[45:53])[0]) # Unpack the bytes to float
            # print("DTC : ", struct.unpack('d', shared_mem.buf[54:62])[0]) # Unpack the bytes to float
            # print("AEB : ", struct.unpack('d', shared_mem.buf[63:71])[0]) # Unpack the bytes to float
            # Verbose
            print("DTC: {} m\tAEB: {}".format(np.round(struct.unpack('d', shared_mem.buf[54:62])[0], 2),
                                               struct.unpack('d', shared_mem.buf[63:71])[0]==1))
        finally:
            # Close the shared memory
            shared_mem.close()
