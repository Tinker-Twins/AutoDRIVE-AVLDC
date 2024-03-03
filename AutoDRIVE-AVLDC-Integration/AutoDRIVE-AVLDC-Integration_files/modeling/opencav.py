#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
from multiprocessing.shared_memory import SharedMemory
import struct
import numpy as np
from scipy.spatial.transform import Rotation
import autodrive

################################################################################

# Create a shared memory with a name
shared_mem = SharedMemory(name='AutoDRIVE', size=1024, create=True)

# Initialize vehicle(s)
opencav_1 = autodrive.OpenCAV()
opencav_1.id = 'V1'

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__) # '__main__'

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print('Connected!')

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    try:
        if data:
            
            ########################################################################
            # PERCEPTION
            ########################################################################

            # Vehicle data
            opencav_1.parse_data(data, verbose=False)

            ########################################################################
            # PLANNING
            ########################################################################

            DTC = np.linalg.norm(opencav_1.position - np.array([-242.16, -119.00, 341.91])) # Compute DTC
            #ba = bytearray(struct.pack("f", DTC))
            #print([ "0x%02x" % b for b in ba ]) # '0x9b', '0x07', '0x14', '0x43'
            #shared_mem.buf[0:3] = bytearray(struct.pack("f", float(DTC))) # Write data to shared memory
            shared_mem.buf[6] = np.uint8(DTC) # Write data to shared memory
            
            rot = Rotation.from_euler('xyz', [shared_mem.buf[3], shared_mem.buf[4], shared_mem.buf[5]], degrees=False)
            quat = rot.as_quat()
            
            # Print data in shared memory
            print("POSX: ", shared_mem.buf[0])
            print("POSY: ", shared_mem.buf[1])
            print("POSZ: ", shared_mem.buf[2])
            print("ROTX: ", shared_mem.buf[3])
            print("ROTY: ", shared_mem.buf[4])
            print("ROTZ: ", shared_mem.buf[5])
            print("DTC : ", shared_mem.buf[6])

            ########################################################################
            # CONTROL
            ########################################################################

            # Vehicle control mode
            opencav_1.cosim_mode = 1
            # Pose commands (only if cosim_mode==1)
            opencav_1.posX_command = float(shared_mem.buf[0])
            opencav_1.posY_command = float(shared_mem.buf[1])
            opencav_1.posZ_command = float(shared_mem.buf[2])
            opencav_1.rotX_command = float(quat[0])
            opencav_1.rotY_command = float(quat[1])
            opencav_1.rotZ_command = float(quat[2])
            opencav_1.rotW_command = float(quat[3])
            # Actuator commands (only if cosim_mode==0)
            # opencav_1.throttle_command = float(shared_mem.buf[0]) # Read data from shared memory
            # opencav_1.steering_command = float(shared_mem.buf[1]) # Read data from shared memory
            # opencav_1.brake_command = float(shared_mem.buf[2]) # Read data from shared memory
            # opencav_1.handbrake_command = float(shared_mem.buf[3]) # Read data from shared memory
            # if dtc < 20:
                # opencav_1.throttle_command = 0 # [-1, 1]
                # opencav_1.steering_command = 0 # [-1, 1]
                # opencav_1.brake_command = 1 # [-1, 1]
                # opencav_1.handbrake_command = 0 # [-1, 1]
            # else:
                # opencav_1.throttle_command = 0.20 # [-1, 1]
                # opencav_1.steering_command = 0 # [-1, 1]
                # opencav_1.brake_command = 0 # [-1, 1]
                # opencav_1.handbrake_command = 0 # [-1, 1]

            ########################################################################

            json_msg = opencav_1.generate_commands(verbose=False) # Generate vehicle 1 message

            try:
                sio.emit('Bridge', data=json_msg)
            except Exception as exception_instance:
                print(exception_instance)
    except KeyboardInterrupt:
        # Close the shared memory
        shared_mem.close()

        # Destroy the shared memory
        shared_mem.unlink()

################################################################################

if __name__ == '__main__':
    app = socketio.Middleware(sio, app) # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app) # Deploy as an eventlet WSGI server
