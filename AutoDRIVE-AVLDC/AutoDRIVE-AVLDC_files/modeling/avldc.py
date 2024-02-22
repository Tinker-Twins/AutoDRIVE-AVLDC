#!/usr/bin/env python

from multiprocessing.shared_memory import SharedMemory

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
            shared_mem.buf[0] = int(self.inputs["Throttle"]["value"]) # Write data to shared memory
            shared_mem.buf[1] = int(self.inputs["Steering"]["value"]) # Write data to shared memory
            shared_mem.buf[2] = int(self.inputs["Brake"]["value"]) # Write data to shared memory
            shared_mem.buf[3] = int(self.inputs["Handbrake"]["value"]) # Write data to shared memory
            self.outputs["DTC"]["value"] = float(shared_mem.buf[4]) # Read data from shared memory
            # Print data in shared memory
            print("THROTTLE : ", shared_mem.buf[0])
            print("STEERING : ", shared_mem.buf[1])
            print("BRAKE    : ", shared_mem.buf[2])
            print("HANDBRAKE: ", shared_mem.buf[3])
            print("DTC      : ", shared_mem.buf[4])
        finally:
            # Close the shared memory
            shared_mem.close()
