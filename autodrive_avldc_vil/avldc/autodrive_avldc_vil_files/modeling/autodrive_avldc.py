#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
import numpy as np
from scipy.spatial.transform import Rotation
from multiprocessing.shared_memory import SharedMemory
import struct
import autodrive

################################################################################

# Create a shared memory with a name
shared_mem = SharedMemory(name='AutoDRIVE', size=1024, create=True)

# Initialize environment
environment = autodrive.Environment()

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

            # Compute distance to collision and AEB trigger
            DTC = np.linalg.norm(opencav_1.position - np.array([-242.16, -119.00, 341.91]))
            AEB = 1 if DTC < 20 else 0

            # Write data to shared memory
            shared_mem.buf[54:62] = struct.pack('d', DTC) # Pack the float to bytes ('d' is for double-precision (64-bit) float
            shared_mem.buf[63:71] = struct.pack('d', AEB) # Pack the float to bytes ('d' is for double-precision (64-bit) float
            
            # Read co-sim states
            eulr = Rotation.from_euler('xyz',
                                       [struct.unpack('d', shared_mem.buf[27:35])[0],
                                        struct.unpack('d', shared_mem.buf[36:44])[0],
                                        struct.unpack('d', shared_mem.buf[45:53])[0]],
                                       degrees=False)
            quat = eulr.as_quat()

            # Print data in shared memory
            # print("POSX: ", struct.unpack('d', shared_mem.buf[0:8])[0])   # Unpack the bytes to float
            # print("POSY: ", struct.unpack('d', shared_mem.buf[9:17])[0])  # Unpack the bytes to float
            # print("POSZ: ", struct.unpack('d', shared_mem.buf[18:26])[0]) # Unpack the bytes to float
            # print("ROTX: ", struct.unpack('d', shared_mem.buf[27:35])[0]) # Unpack the bytes to float
            # print("ROTY: ", struct.unpack('d', shared_mem.buf[36:44])[0]) # Unpack the bytes to float
            # print("ROTZ: ", struct.unpack('d', shared_mem.buf[45:53])[0]) # Unpack the bytes to float
            # print("DTC : ", struct.unpack('d', shared_mem.buf[54:62])[0]) # Unpack the bytes to float
            # print("AEB : ", struct.unpack('d', shared_mem.buf[63:71])[0]) # Unpack the bytes to float

            ########################################################################
            # CONTROL
            ########################################################################

            # Environmental conditions
            environment.auto_time = "False" # ["False", "True"]
            environment.time_scale = 60 # [0, inf) (only used if auto_time==True)
            environment.time_of_day = 560 # [minutes in 24 hour format] (only used if auto_time==False)
            environment.weather_id = 3 # [0=Custom, 1=Sunny, 2=Cloudy, 3=LightFog, 4=HeavyFog, 5=LightRain, 6=HeavyRain, 7=LightSnow, 8=HeavySnow]
            environment.cloud_intensity = 0.0 # [0, 1] (only used if weather_id==0)
            environment.fog_intensity = 0.0 # [0, 1] (only used if weather_id==0)
            environment.rain_intensity = 0.0 # [0, 1] (only used if weather_id==0)
            environment.snow_intensity = 0.0 # [0, 1] (only used if weather_id==0)

            # Co-simulation mode
            opencav_1.cosim_mode = 1
            
            # Pose commands (only if cosim_mode==1)
            opencav_1.posX_command = struct.unpack('d', shared_mem.buf[0:8])[0]   # Unpack the bytes to float
            opencav_1.posY_command = struct.unpack('d', shared_mem.buf[9:17])[0]  # Unpack the bytes to float
            opencav_1.posZ_command = struct.unpack('d', shared_mem.buf[18:26])[0] # Unpack the bytes to float
            opencav_1.rotX_command = quat[0]
            opencav_1.rotY_command = quat[1]
            opencav_1.rotZ_command = quat[2]
            opencav_1.rotW_command = quat[3]

            # Light commands
            opencav_1.headlights_command = 0 # Vehicle headlights command [0 = Disabled, 1 = Low Beam, 2 = High Beam, 3 = Parking Lights, 4 = Fog Lights, 5 = 1+3, 6 = 1+4, 7 = 2+3, 8 = 2+4, 9 = 3+4, 10 = 1+3+4, 11 = 2+3+4]
            if opencav_1.collision_count > 0:
                opencav_1.indicators_command = 3 # Vehicle indicators command [0 = Disabled, 1 = Left Turn Indicator, 2 = Right Turn Indicator, 3 = Hazard Indicators]
            else:
                opencav_1.indicators_command = 0 # Vehicle indicators command [0 = Disabled, 1 = Left Turn Indicator, 2 = Right Turn Indicator, 3 = Hazard Indicators]

            # Verbose
            print("DTC: {} m\tAEB: {}".format(np.round(struct.unpack('d', shared_mem.buf[54:62])[0], 2),
                                               struct.unpack('d', shared_mem.buf[63:71])[0]==1))
            
            ########################################################################

            json_msg = environment.generate_commands(verbose=False) # Generate environment message
            json_msg.update(opencav_1.generate_commands(verbose=False)) # Append vehicle 1 message

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
