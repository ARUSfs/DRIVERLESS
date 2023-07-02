# Localization Node

This is the **Localization Node documentation** of ARUSfs.

```{warning}
This documentation is under development
```

## Objective
The objective of the Localization node is to allow the car to know its position and any object perceived by it, being able to apply transformations in the space easily. It also allows us to know other important aspects, such as speed and acceleration. In addition, all this in function of time.

## Structure
The Localization node is composed of two parts:
- The first part is in charge of the communication with the IMU. It selectively chooses the messages that interest us from this, and sends them through different topics so that we can make use of them later.
- The second part of the node reads the messages that were previously sent through the topics, to implement the TF2 system.

### IMU communication
To communicate with the IMU, the SBG drivers for ROS are used. With them, we have access to a wide range of parameters.
At the moment, the values collected by Localization are:
- Speed.
- Speed (GPS).
- Position (GPS).
- Acceleration.
All of them are sent through new topics, with a Vector3 type message (a vector with three parameters).

The implementation consists of several parts:
- Subscription to topics: Communicates the node with the outputs of the data emitted by the IMU. It is implemented through the subscribe_topics() function.
- "Send" functions: They are functions that are in charge of reading the messages sent by the IMU and transforming them into Vector3 type with the relevant data from the message.
- Publication to topics: Creates new topics through which it sends the messages created by the "send" functions. It is implemented through the publish_topics() function.

### TF2 system
The TF2 library is the second version of the Transform library of ROS. It allows the tracking of frames over time, being able to apply transformations in the different reference systems easily. TF2 allows you to create as many frames as you need. In localization there are the following:

```{figure} https://i.ibb.co/hDDrnyx/imagen.png
:width: 250 px
:alt: TF2 tree
```

The world frame is the main one, by default in TF2. It is from it that start, in charge of registering the position of the IMU at the moment the node is started. After this, imu is deployed, which collects in real time the position of the IMU itself. To be more precise, there is a fourth frame that comes from imu, called camera. This last one defines the position in which the camera will be with respect to the IMU, since the camera will be the one that perceives the cones, not the IMU.

*Currently the distance between the camera and the IMU is not defined.

The implementation of TF2 is carried out by creating a new function for each new frame. These functions are composed as follows:
- Call to the broadcaster, which will be in charge of sending the transformation later. In this call, we define whether our frame will be static or not, depending on the type of broadcaster that we define. A transformation is also created at the beginning.
- Call to the listener, which will be in charge of listening to the transformations that are sent. In this call, we define the frame that we want to listen to, and the frame that we want to transform.
- Call to the transform, which will be in charge of applying the transformation to the data that we want. In this call, we define the frame that we want to transform, the frame to which we want to transform it, and the data that we want to transform.

