#!/usr/bin/env python

from multiprocessing.shared_memory import SharedMemory
import numpy as np

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
                "value": 0.0,
            },
            "PosY": {
                "type": "float",
                "value": 0.0,
            },
            "PosZ": {
                "type": "float",
                "value": 0.0,
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
            shared_mem.buf[0] = np.uint8(self.inputs["PosX"]["value"]) # Write data to shared memory
            shared_mem.buf[1] = np.uint8(self.inputs["PosY"]["value"]) # Write data to shared memory
            shared_mem.buf[2] = np.uint8(self.inputs["PosZ"]["value"]) # Write data to shared memory
            shared_mem.buf[3] = np.uint8(self.inputs["RotX"]["value"]) # Write data to shared memory
            shared_mem.buf[4] = np.uint8(self.inputs["RotY"]["value"]) # Write data to shared memory
            shared_mem.buf[5] = np.uint8(self.inputs["RotZ"]["value"]) # Write data to shared memory
            self.outputs["DTC"]["value"] = float(shared_mem.buf[6]) # Read data from shared memory
            # Print data in shared memory
            print("POSX: ", shared_mem.buf[0])
            print("POSY: ", shared_mem.buf[1])
            print("POSZ: ", shared_mem.buf[2])
            print("ROTX: ", shared_mem.buf[3])
            print("ROTY: ", shared_mem.buf[4])
            print("ROTZ: ", shared_mem.buf[5])
            print("DTC : ", shared_mem.buf[6])
        finally:
            # Close the shared memory
            shared_mem.close()
