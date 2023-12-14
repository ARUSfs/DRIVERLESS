# RS-LiDAR-16

The RS-LiDAR-16 is Real-Time 3D LiDAR sensor launched by RoboSense. The full specifications can be found [here](https://cdn.robosense.cn/20200723161715_42428.pdf).

```{figure} https://ae01.alicdn.com/kf/H2b50d6cd6c6d437ca1f57f012aaedd66c.jpg_640x640Q90.jpg_.webp
:width: 130 px
:alt: RS-LiDAR-16
RS-LiDAR-16
```

## How to connect to ROS Noetic
To connect the RS-Lidar-16 to our computer, we must connect the LiDAR to its controller, and then connect the controller to our computer. To connect the controller to our computer, we must follow the following steps:
 - Connect the controller's ethernet cable to our computer.
 - Configure the IP address of our computer to `192.168.1.102` and the subnet mask to `255.255.255.0`. For this, we can follow the following steps:
    - In Ubuntu, go to `Settings > Network > Wired > Options > IPv4 Settings > Method: Manual`. Then, enter the IP address and subnet mask mentioned above.
 - Clone the [ros_rslidar](https://github.com/RoboSense-LiDAR/rslidar_sdk) repository into the `src` folder of a ROS workspace.
    - In the workspace src directory, run:
    ```bash
    git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
    cd rslidar_sdk
    git submodule init
    git submodule update
    ```
 - Install dependencies:
    - Yaml (Essential)
    ```bash
    sudo apt-get update
    sudo apt-get install -y libyaml-cpp-dev
    ```
    - libpcap (Essential)
    ```bash
    sudo apt-get update
    sudo apt-get install -y libpcap-dev
    ```
 - Edit the `rslidar_sdk/config/config.yaml` file and change the value of `lidar_type` to `RS16`.
 - Compile the workspace. In the workspace `src` folder, run:
    ```bash
    catkin_make
    source devel/setup.bash
    roslaunch rslidar_sdk rslidar.launch
    ```
```{attention}
If any error occurs, check the [official ros_rslidar Readme](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/README.md).
