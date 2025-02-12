# Darknet YOLO with ROS

## Build:
1. Make a directory `ROS_WS` to act as your ROS 1 workspace.
    ```bash
    $ mkdir -p ~/ROS_WS/src/
    ```
2. Clone the repository:
    ```bash
    $ git clone https://github.com/Tinker-Twins/AutoDRIVE-AVLDC.git
    ```
3. Install [`OpenCV`](https://opencv.org) (or [build from source](https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html)).
    ```bash
    $ sudo apt update
    $ sudo apt install libopencv-dev python3-opencv
    ```
4. Build [`boost`](https://www.boost.org).
    ```bash
    $ cd object_detector/boost
    $ ./bootstrap.sh
    $ ./b2 headers
    ```
5. Build the ROS packages (build in `Release` mode to maximize performance).
    ```bash
    $ cd ~/ROS_WS
    $ catkin_make -DCMAKE_BUILD_TYPE=Release
    ```
6. Source the `setup.bash` file of your `ROS_WS`.
    ```bash
    $ echo "source ~/ROS_WS/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

## Execute:
```bash
$ roslaunch darknet_ros darknet_ros.launch
```

## Configure:
- Names and other parameters of the publishers, subscribers and actions can be modified from `darknet_ros/darknet_ros/config/ros.yaml`.
- Parameters related to YOLO object detection algorithm can be modified from `darknet_ros/darknet_ros/config/yolo.yaml`.
- It is recommended to create a copy of the existing configuration file(s) as a template and do necessary modifications.
- Reference the updated configuration file(s) in `darknet_ros/darknet_ros/launch/darknet_ros.launch` or create new launch file(s).
