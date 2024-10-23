# ROS1 Packages for Mobile Robot

## Packages
* amc_base_msg: message definitions
* amc_base_serial: Used for communication between serial port and robot

## Supported Hardware

* LZ_PRO
* LZ_600
* LZ_STD(LZ_500)

**Note:** Before starting the node, modify the parameters of the corresponding vehicle type.

## Basic usage of the ROS packages

1. Copy the packages into your workspace and compile

    ```
    $ mkdir -p ~/ros_ws/src
    $ cd ~/ros_ws/src
    $ copy amc_base_msg
    $ copy amc_base_serial
    $ catkin_make
    ```

2. Launch ROS nodes

* Start the ros system

    ```
    $ roscore
    ```

* Setting environment variables

    ```
    $ source ./devel/setup.bash
    ```
 
* Start the base node for the robot

    * Automatic search serial port
    ```
    $ rosrun amc_base_serial amc_base_node_serial.py
    ```

    ```
    $ rosrun amc_base_serial amc_ros_cpp_node
    ```

    * Select a serial port

    ```
    $ rosrun amc_base_serial amc_base_node_serial.py _serial_port:=ttyUSB0
    ```

    ```
    $ rosrun amc_base_serial amc_ros_cpp_node ttyUSB0
    ```

    * Boot keyboard control
    ```
    $ rosrun amc_base_serial amc_base_cmd_vel_publish.py
    ```

3. Subscribe to Topics

* BMS topics
    ```
    $ rostopic echo /bms
    ```

* Speed topics
    ```
    $ rostopic echo /car_speed
    ```
