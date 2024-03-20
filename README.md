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

### OpenCAV AEB Demo

1. Download, unzip and launch the AutoDRIVE Simulator by referring to the detailed instructions given [here](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE/tree/AutoDRIVE-Simulator?tab=readme-ov-file#download-and-run):
    - [Windows](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v1.0.0/AutoDRIVE_Simulator_Windows.zip)
    - [Linux](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v1.0.0/AutoDRIVE_Simulator_Linux.zip)
    - [macOS](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v1.0.0/AutoDRIVE_Simulator_macOS.zip)  

2. Execute AEB script (a) or (b), which also creates a shared memory for the [AVL Model.CONNECT project](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim.proj).

   (a) Execute the [`aeb_emulation`](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim_files/modeling/aeb_emulation.py) Python3 script for demonstrating the "emulated" autonomous emergency braking (AEB) function with OpenCAV, employing the [AutoDRIVE Python API](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim_files/modeling/autodrive.py). This algorithm uses ground truth distance to collision (DTC) metric to trigger AEB.

   ```bash
    $ cd <path/to/aeb_emulation.py>
    $ python3 aeb_emulation.py
    ```

    (b) Execute the [`aeb_stimulation`](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim_files/modeling/aeb_stimulation.py) Python3 script for demonstrating the "stimulated" autonomous emergency braking (AEB) function with OpenCAV, employing the [AutoDRIVE Python API](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim_files/modeling/autodrive.py). This algorithm uses camera-based perception for object detection and classification, and triggers the AEB based on the class, size and confidence of the detection.

   ```bash
    $ cd <path/to/aeb_stimulation.py>
    $ python3 aeb_stimulation.py
    ```

3. Launch and run the [AVL Model.CONNECT project](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim.proj), which connects to the shared memory created by the AEB script using [AVL Python API](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim_files/modeling/avldc.py).
