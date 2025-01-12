<p align="center">
<img src="media/AutoDRIVE-Logo.png" alt="AutoDRIVE" height="100"/> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <img src="media/AVL-Logo.jpg" alt="AVL" height="100"/>
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
    [AutoDRIVE Devkit's Python API](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_py) has the following dependencies (tested with Python 3.8, 3.9 and 3.10):
    
    - Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator) and [AutoDRIVE Devkit](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit) (version sensitive):
    
      | Package            | Python 3.8 | Python 3.9 | Python 3.10 |
      |--------------------|------------|------------|-------------|
      | eventlet           | 0.33.3     | 0.33.3     | 0.33.3      |
      | Flask              | 1.1.1      | 1.1.1      | 1.1.1       |
      | Flask-SocketIO     | 4.1.0      | 4.1.0      | 4.1.0       |
      | python-socketio    | 4.2.0      | 4.2.0      | 4.2.0       |
      | python-engineio    | 3.13.0     | 3.13.0     | 3.13.0      |
      | greenlet           | 1.0.0      | 1.0.0      | 1.1.0       |
      | gevent             | 21.1.2     | 21.1.2     | 21.12.0     |
      | gevent-websocket   | 0.10.1     | 0.10.1     | 0.10.1      |
      | Jinja2             | 3.0.3      | 3.0.3      | 3.0.3       |
      | itsdangerous       | 2.0.1      | 2.0.1      | 2.0.1       |
      | werkzeug           | 2.0.3      | 2.0.3      | 2.0.3       |
    
    - Generic dependencies for data processing and visualization (usually any version will do the job):
    
      | Package               | Tested Version |
      |-----------------------|----------------|
      | numpy                 | 1.13.3         |
      | pillow                | 5.1.0          |
      | opencv-contrib-python | 4.5.1.48       |
  
    - Install dependencies using `requirements.txt` file (use the file specific to your Python version &#8594; check using `python3 --version`):

      ```bash
      $ pip3 install -r requirements_python_3.8.txt # Python 3.8
      $ pip3 install -r requirements_python_3.9.txt # Python 3.9
      $ pip3 install -r requirements_python_3.10.txt # Python 3.10
      ```
## USAGE

### OpenCAV Teleoperation Demo

1. Download, unzip and launch the AutoDRIVE Simulator by referring to the detailed instructions given [here](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE/tree/AutoDRIVE-Simulator?tab=readme-ov-file#download-and-run):
    - [Windows](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.2.0/AutoDRIVE_Simulator_Windows.zip)
    - [Linux](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.2.0/AutoDRIVE_Simulator_Linux.zip)
    - [macOS](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.2.0/AutoDRIVE_Simulator_macOS.zip)

2. Launch AutoDRIVE Devkit to create a shared memory for the [AVL Model.CONNECT project](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_sim/autodrive_avldc_sim.proj). Execute the [`opencav`](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_sim/autodrive_avldc_sim_files/modeling/opencav.py) Python3 script for teleoperating the OpenCAV, employing the [AutoDRIVE Python API](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_sim/autodrive_avldc_sim_files/modeling/autodrive.py).

   ```bash
    $ cd <path/to/opencav.py>
    $ python3 opencav.py
    ```

3. Launch and run the [AVL Model.CONNECT project](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_sim/autodrive_avldc_sim.proj), which connects to the shared memory created by the AutoDRIVE Devkit using [AVL Python API](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_sim/autodrive_avldc_sim_files/modeling/avldc.py).

    **Notes:**
    - Launching AutoDRIVE Devkit alone will print the default value for all the bytes that haven't yet been written to (since they are written by the AVL Model.CONNECT project). The bytes written by the AutoDRIVE Devkit itself will be updated and printed accordingly.
    - Launching the AVL Model.CONNECT project before the AutoDRIVE Devkit will throw an error since the shared memory has not yet been created.

<table>
<thead>
  <tr>
    <th colspan="3" align="left">OpenCAV Teleoperation Demo Result</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Teleoperation/AutoDRIVE%20Simulator.gif"</td>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Teleoperation/AutoDRIVE%20Devkit.gif"</td>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Teleoperation/AVL%20Model.CONNECT.gif"</td>
  </tr>
  <tr>
    <td align="center">AutoDRIVE Simulator</td>
    <td align="center">AutoDRIVE Devkit</td>
    <td align="center">AVL Model.CONNECT</td>
  </tr>
</tbody>
</table>

### OpenCAV AEB Demo

1. Download, unzip and launch the AutoDRIVE Simulator by referring to the detailed instructions given [here](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE/tree/AutoDRIVE-Simulator?tab=readme-ov-file#download-and-run):
    - [Windows](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.2.0/AutoDRIVE_Simulator_Windows.zip)
    - [Linux](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.2.0/AutoDRIVE_Simulator_Linux.zip)
    - [macOS](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/releases/download/v0.2.0/AutoDRIVE_Simulator_macOS.zip)

2. Launch AutoDRIVE Devkit to execute AEB script (a) or (b), which also creates a shared memory for the [AVL Model.CONNECT project](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim.proj).

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

3. Launch and run the [AVL Model.CONNECT project](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim.proj), which connects to the shared memory created by the AutoDRIVE Devkit using [AVL Python API](https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/autodrive_avldc_cosim/autodrive_avldc_cosim_files/modeling/avldc.py).

    **Notes:**
    - Launching AutoDRIVE Devkit alone will print the default value for all the bytes that haven't yet been written to (since they are written by the AVL Model.CONNECT project). The bytes written by the AutoDRIVE Devkit itself will be updated and printed accordingly.
    - Launching the AVL Model.CONNECT project before the AutoDRIVE Devkit will throw an error since the shared memory has not yet been created.

<table>
<thead>
  <tr>
    <th colspan="3" align="left">OpenCAV AEB Demo Result: Emulation Mode</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Emulation/AutoDRIVE%20Simulator.gif"</td>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Emulation/AutoDRIVE%20Devkit.gif"</td>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Emulation/AVL%20Model.CONNECT.gif"</td>
  </tr>
  <tr>
    <td align="center">AutoDRIVE Simulator</td>
    <td align="center">AutoDRIVE Devkit</td>
    <td align="center">AVL Model.CONNECT</td>
  </tr>
</tbody>
</table>

<table>
<thead>
  <tr>
    <th colspan="3" align="left">OpenCAV AEB Demo Result: Stimulation Mode</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Stimulation/AutoDRIVE%20Simulator.gif"</td>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Stimulation/AutoDRIVE%20Devkit.gif"</td>
    <td align="center"><img src="https://github.com/Tinker-Twins/AutoDRIVE-AVLDC/blob/main/media/Stimulation/AVL%20Model.CONNECT.gif"</td>
  </tr>
  <tr>
    <td align="center">AutoDRIVE Simulator</td>
    <td align="center">AutoDRIVE Devkit</td>
    <td align="center">AVL Model.CONNECT</td>
  </tr>
</tbody>
</table>
