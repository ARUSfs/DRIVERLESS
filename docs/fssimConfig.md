# FSSim Configuration

This is the **FSSim Configuration** of ARUSfs.

## Setup
To configure the simulator, we follow the following steps:
1. Create a ROS workspace.
2. In the src folder of the workspace, clone the [FSSim](https://github.com/ARUSfs/fssim_noetic_23) y el de [DRIVERLESS](https://github.com/ARUSfs/DRIVERLESS) _Pendiente de actualización 07-02-23_ repositories. Compile both repositories. Make sure all dependencies of both repositories are installed.
3. To launch the simulator together with our repository, we must execute the following launchers (automatically with the command `roslaunch fssim auto_fssim.launch`):
- `roslaunch fssim_interface simulation.launch`
- `roslaunch visualization visualization.launch`
- `roslaunch delaunay_detector planning.launch`
- `roslaunch control_pure_pursuit control.launch`

## Simulator configuration
Inside the simulator, we can change different parameters to adjust it to the needs of each moment. We can choose the circuit, the sensors and their configuration, speed, etc.

### Circuit
To change the circuit that will be executed in the simulator, you have to modify the file `fssim/config/simulation.yaml`. We change the _track name_ parameter for the name of the circuit we want.

### Sensors
To change the sensors that will be executed, we must modify the file `driverless/src/fssim_interface/src/transformer.py`. In this file, in the function that subscribes to the topics, we comment the line related to the camera or the LiDAR.

To change the sensor configurations, you have to modify the file `fssim/fssim_description/car/gotthard/config/sensor.yaml`. In this file we have both the parameters of the LiDAR (beginning of the file) and those of the camera (end of the file). Among these parameters, the most relevant are:
- **observation_radius:** observation radius of the LiDAR/Camera.
- **color_observation_radius:** observation radius of the LiDAR/Camera for color.

To deactivate color detection, you have to modify the file `driverless/src/planning/delaunay_detector/config/delaunay.yaml` and set the `color_detection_enabled` parameter to `false`.

### Speed (PID)
To modify the behavior of the car in the simulations, we can modify the parameters of the PID. To do this, we must modify the file `driverless/src/control/control_pure_pursuit/config/pure_pursuit.yaml`. In this file, we can modify the following parameters:
- **target_speed:** constant speed at which we want the car to go. Modifying this value, entails modifying the rest of the parameters.
- **kp:** proportional parameter of the PID.
- **ki:** integral parameter of the PID.
- **kd:** derivative parameter of the PID.
- **lfc:** look ahead distance.
- **k:** look forward gain.

PID calculate the difference between our real speed and the target speed. This difference is multiplied by the proportional, integral and derivative parameters, and the results are added. This value is added to the target speed, and the real speed of the car is obtained. Therefore, if we want the car to go faster, we must increase the value of the target speed. If we want the car to go slower, we must decrease the value of the target speed.

As the car starts with speed 0, initially the error is very large. To correct it, the integral parameter is in charge of adding the errors, so that the total error is smaller. The derivative parameter is in charge of making the car not move abruptly, but to correct the error little by little.


```{figure} https://i.ibb.co/GxgnnZd/Captura-de-pantalla-de-2023-07-02-20-10-31.png
:width: 300 px
:alt: PID explanation
Explanation of the kp, ki and kd.
```

The parameter lfc represents the look ahead distance, that is, the distance at which the car looks ahead to calculate the curve it must follow. If we increase the value of this parameter, the car will look further ahead, and therefore, the curve it will follow will be smoother. If we decrease the value of this parameter, the car will look closer, and therefore, the curve it will follow will be more abrupt.

The parameter k represents the look forward gain, that is, the gain that is applied to the look ahead distance. 





