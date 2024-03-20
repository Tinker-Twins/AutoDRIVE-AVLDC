<p align="center">
<img src="media/AutoDRIVE-Logo.png" alt="AutoDRIVE" height="100"/> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <img src="media/AVL-Logo.jpg" alt="Autoware" height="100"/>
</p>

## SETUP

1. Clone this repository.
    ```bash
    $ git clone https://github.com/Tinker-Twins/AutoDRIVE-AVLDC.git
    ```
2. Give executable permissions to all the Python scripts.
   ```bash
   $ cd <path/to/cloned/repo>
   $ sudo chmod +x *.py
   ```
4. Install the necessary dependencies as mentioned below.
    [AutoDRIVE Devkit's Python API](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_py) has the following dependencies (tested with Python 3.8.10):
    
    - Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator) and [AutoDRIVE Devkit](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit) (version sensitive):
    
      | Package | Tested Version |
      |---------|----------------|
      | eventlet | 0.33.3 |
      | Flask | 1.1.1 |
      | Flask-SocketIO | 4.1.0 |
      | python-socketio | 4.2.0 |
      | python-engineio | 3.13.0 |
      | greenlet | 1.0.0 |
      | gevent | 21.1.2 |
      | gevent-websocket | 0.10.1 |
      | Jinja2 | 3.0.3 |
      | itsdangerous | 2.0.1 |
      | werkzeug | 2.0.3 |
      
      ```bash
      $ pip3 install eventlet==0.33.3
      $ pip3 install Flask==1.1.1
      $ pip3 install Flask-SocketIO==4.1.0
      $ pip3 install python-socketio==4.2.0
      $ pip3 install python-engineio==3.13.0
      $ pip3 install greenlet==1.0.0
      $ pip3 install gevent==21.1.2
      $ pip3 install gevent-websocket==0.10.1
      $ pip3 install Jinja2==3.0.3
      $ pip3 install itsdangerous==2.0.1
      $ pip3 install werkzeug==2.0.3
      ```
    
    - Generic dependencies for data processing and visualization (usually any version will do the job):
    
      | Package | Tested Version |
      |---------|----------------|
      | numpy | 1.13.3 |
      | pillow | 5.1.0 |
      | opencv-contrib-python | 4.5.1.48 |
      | scipy | 1.10.1 |
      
      ```bash
      $ pip3 install numpy
      $ pip3 install pillow
      $ pip3 install opencv-contrib-python
      $ pip3 install scipy
      ```

## USAGE

### OpenCAV AEB Demo [[Python](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/tree/devel/aeb_py)]

1. Download, unzip and launch the AutoDRIVE Simulator by referring to the detailed instructions given [here](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE/tree/AutoDRIVE-Simulator?tab=readme-ov-file#download-and-run):
    - [Windows](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.1.0/AutoDRIVE_Simulator_Windows.zip)
    - [Linux](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.1.0/AutoDRIVE_Simulator_Linux.zip)
    - [macOS](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.1.0/AutoDRIVE_Simulator_macOS.zip)  

2. Execute the [`opencav_aeb`](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/tree/devel/aeb_py/opencav_aeb.py) Python3 script for demonstrating the automatic emergency braking (AEB) function with OpenCAV, employing the AutoDRIVE Python API.
    ```bash
    $ cd <path/to/opencav_aeb.py>
    $ python3 opencav_aeb.py
    ```

<p align="center">
<img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/devel/media/OpenCAV-AEB-Python.gif"/>
</p>

### OpenCAV AEB Demo [[ROS](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/tree/devel/aeb_ros)]

1. Download, unzip and launch the AutoDRIVE Simulator by referring to the detailed instructions given [here](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE/tree/AutoDRIVE-Simulator?tab=readme-ov-file#download-and-run):
    - [Windows](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.1.0/AutoDRIVE_Simulator_Windows.zip)
    - [Linux](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.1.0/AutoDRIVE_Simulator_Linux.zip)
    - [macOS](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.1.0/AutoDRIVE_Simulator_macOS.zip)

2. Build the [ROS packages](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/tree/devel/aeb_ros) by moving them to the `src` space of your ROS workspace `ros_ws`.
    ```bash
    $ cd <path/to/ros_ws>
    $ catkin_make -DCMAKE_BUILD_TYPE=Release
    ```

3. Execute the [`aeb_emulation.launch`](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/devel/aeb_ros/aeb_function/launch/aeb_emulation.launch) file for demonstrating the automatic emergency braking (AEB) function with OpenCAV in `simulated emulation mode`, employing the AutoDRIVE ROS API.
    ```bash
    $ roslaunch autodrive_opencav simulator_bringup_headless.launch
    $ roslaunch aeb_function aeb_emulation.launch
    ```

<p align="center">
<img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/devel/media/OpenCAV-AEB-ROS-Emulation.gif"/>
</p>

4. Execute the [`aeb_simulation.launch`](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/devel/aeb_ros/aeb_function/launch/aeb_simulation.launch) file for demonstrating the automatic emergency braking (AEB) function with OpenCAV in `full simulation mode`, employing the AutoDRIVE ROS API.
    ```bash
    $ roslaunch autodrive_opencav simulator_bringup_headless.launch
    $ roslaunch aeb_function aeb_simulation.launch
    ```

<p align="center">
<img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/devel/media/OpenCAV-AEB-ROS-Simulation.gif"/>
</p>
