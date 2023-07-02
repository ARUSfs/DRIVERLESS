# FSSim

## Introduction
FSSim is a vehicle simulator dedicated for Formula Student Driverless Competition. It was developed for autonomous software testing purposes and not for gaming. FSSim is developed by [Juraj Kabzan](https://www.linkedin.com/in/juraj-kabzan-143698a1/), member of [AMZ-Driverless](http://driverless.amzracing.ch/). 
Their more extensive documentation can be found [here](https://github.com/AMZ-Driverless/fssim/blob/master/fssim_doc/index.md).

## Install ROS Noetic
To install ROS Noetic, follow the instructions located [here](https://arusfs.github.io/DRIVERLESS/instalarEntorno.html#installation-of-ros-noetic).

## Prepare Workspace
Now we need to create a workspace for our project. To do so, in a terminal, type:
1. `mkdir ros_ws/src`
2. In the `ros_ws` folder type `catkin init`
3. In the `src` folder type `git clone https://github.com/Huguet57/fssim-2021.git`
4. In the `fssim-2021` folder type `./update_dependencies.sh`
    1. If you get an error, try this and then try again: 
        - `sudo apt-get install python3-rosdep`
        - `sudo rosdep init`
        - `rosdep update`
5. `catkin build`
6. In the `ros_ws` folder type `source devel/setup.bash`
7. To start the simulator, type `roslaunch fssim auto_fssim.launch`
    1. The first time you run this command, it will take a while to download the map.

## Configure a simulation
The basic configuration file (fssim/config/simulation.yaml) for the simulation follows the following model. The following parameters are collected in this:
- **simulation_name**: We choose any name for the simulation.
- **robot_name**: It must be the same name that we have chosen to create our vehicle.
- **kill_after**: Maximum time, in seconds, that our simulation can last.
- **pkg_config_storage**: Path to the directory where we store the configuration files. (Do not touch).
- **repetitions**:
    - **sensors_config_file**: Path to the sensor configuration file.
    - **track_name**: Circuit for the simulation.
        * FSG.sdf
        * FSI.sdf
        * thin.sdf
        * skidpad.sdf
        * acceleration.sdf
        * or any other circuit that we have created.
    - **autonomous_stack**: Here we can place a command (eg. roslaunch arus_pkg arus.launch) that starts the autonomous system pipeline. If the value is left empty, the simulation starts without more.

### Create a vehicle
The vehicles are created in fssim_description > cars. In this repository, we will create a new folder with the name of our new car. Is recomendable making a copy of the existing car (gotthard). Once the copy is created, we are interested in modifying the files in the config folder. We will find three files:
- **car.yalm**: It collects values about the car (weight, length, weight distribution, etc.), the wheels, aerodynamics, transmission and torque. [Example](https://github.com/Huguet57/fssim-2021/blob/master/fssim_description/cars/gotthard/config/car.yaml).
- **distances.yalm**: It collects values about the chassis (base, joints of each wheel and steering hinge). [Example](https://github.com/Huguet57/fssim-2021/blob/master/fssim_description/cars/gotthard/config/distances.yaml).
- **sensors.yalm**: It collects information about the sensors used in the car. It consists of two sections: lidar and camera, in which all the necessary information about them is written. [Example](https://github.com/Huguet57/fssim-2021/blob/master/fssim_description/cars/gotthard/config/sensors.yaml).

### Create a circuit
We can create a new track (skidpad, acceleration or trackdrive) thanks to the FSSIM track editor. To do this, the following steps must be followed:
1. In a terminal, run `roscore`.
2. In another terminal, run `rosrun rqt_fssim_track_editor rqt_fssim_track_editor`
    - **Important:** Do not use python3, it will not work. Use python. Also, install `sudo apt-get install python-lxml`.
3. In the editor, press and hold Ctrl and with the left click draw the center of the track. The origin is always 0,0. The car always starts at the origin to positive X.
Do not close the circuit by hand. Leave a reasonable distance between the first and last point, the editor will take care of closing the circuit when pressing compute track.
4. Click on Compute Track to generate the cones (later is posible add or delete cones manually), choose a name and press Export Track. This will automatically generate the circuit files, so it can already be used in the configuration file (in track_name). The files are generated in fssim/fssim_gazebo/models/track.
5. In fssim/fssim_gazebo/models/track there is a file called track_corrector.py. Run `python3 track_corrector.py  track_name.sdf`. This will correct the circuit so that the car does not leave the track. The corrected file overwrites the previous one (be careful not to overwrite previous circuits). 
6. To use the created circuit, in the simulation configuration file (fssim/config/simulation.yaml) in the repetitions section, in track_name, we put the name of the circuit we have created. If we want to change the name of an already created circuit, modify the track_name.sdf and track_name.yaml file.

### Configure a multiple simulation
A multiple simulation is a simulation that is repeated several times with different parameters. This is useful for testing the performance of our autonomous system in different conditions.  
To configure a multiple simulation, the only parameter that we must modify is the repetitions parameter. [Example](https://github.com/Huguet57/fssim-2021/blob/master/fssim/config/automated_simulation.yaml).  
To start the simulation, run:  
`./src/fssim/fssim/scripts/launch.py --config src/fssim/fssim/config/automated_simulation.yaml --output ~/sim_output`

## Example of a simulation
Is posible to make a simulation example with a basic control system already implemented. To do this, we just need to clone the repository that contains this test. I made a fork adapt it to ROS-Noetic, since it is originally designed for ROS-Kinetic and we will not be able to run it correctly. To run the simulation, the following steps must be followed:
1. Clone the repository (fork to adapt to ROS-Noetic):
    - `git clone https://github.com/jormunrod/fsd_skeleton_noetic.git`
    - `cd fsd_skeleton_noetic`
    - `./update_dependencies.sh -f`
    - `catkin build`
    - `source fsd_environment.sh // source devel/setup.bash`
2. In different terminals, run `roscore` and:
    - `roslaunch fssim_interface fssim.launch`
    - `roslaunch control_meta trackdrive.launch`







