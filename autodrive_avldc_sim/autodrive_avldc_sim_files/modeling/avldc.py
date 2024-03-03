#!/usr/bin/env python

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
            "Throttle": {
                "type": "float",
                "value": 0.0,
            },
            "Steering": {
                "type": "float",
                "value": 0.0,
            },
            "Brake": {
                "type": "float",
                "value": 0.0,
            },
            "Handbrake": {
                "type": "float",
                "value": 0.0,
            }
        }
        self.outputs = {
            "DTC": {
                "type": "float",
                "value": 0.0,
            }
        }

    def initialize(self):
        pass

    def do_step(self, currentCommunicationPoint, communicationStepSize):
        shared_mem = SharedMemory(name='AutoDRIVE', create=False)
        try:
            shared_mem.buf[0:8] = struct.pack('d', self.inputs["Throttle"]["value"])    # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[9:17] = struct.pack('d', self.inputs["Steering"]["value"])   # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[18:26] = struct.pack('d', self.inputs["Brake"]["value"])     # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[27:35] = struct.pack('d', self.inputs["Handbrake"]["value"]) # Pack the float to bytes ('d' is for double-precision (64-bit) float
            self.outputs["DTC"]["value"] = struct.unpack('d', shared_mem.buf[36:44])[0] # Unpack the bytes to float
            # Print data in shared memory
            print("THROTTLE : ", struct.unpack('d', shared_mem.buf[0:8])[0])   # Unpack the bytes to float
            print("STEERING : ", struct.unpack('d', shared_mem.buf[9:17])[0])  # Unpack the bytes to float
            print("BRAKE    : ", struct.unpack('d', shared_mem.buf[18:26])[0]) # Unpack the bytes to float
            print("HANDBRAKE: ", struct.unpack('d', shared_mem.buf[27:35])[0]) # Unpack the bytes to float
            print("DTC      : ", struct.unpack('d', shared_mem.buf[36:44])[0]) # Unpack the bytes to float
        finally:
            # Close the shared memory
            shared_mem.close()
