# Vision Cone Detector

```{note}
This documentation may contain errors. If you find any, please report it.
```

## Introduction
The Vision Cone Detector node receives the image from the camera and returns the position of the cones in the world. It has two output topics: one for the position of the cones applying a homography matrix and another for the position of the cones in the pixels of the image and their bounding boxes (this one is only used for SLAM).

## Setup
1. For the node to work, it uses the Darknet library. To install it, the following procedure must be followed:
    1. Download the [Darknet](https://github.com/ARUSfs/darknet) repository:
        * `git clone https://github.com/ARUSfs/darknet`
    2. Modify the CMakeLists.txt file:
        * If our computer does not have an Nvidia GPU compatible with CUDA, put the variables related to this technology in `OFF`.
        * If our computer does not have OpenCV, put the variables related to this technology in `OFF`.
    3. Compile the repository:
        * `mkdir build && cd build`
        * `cmake ..`, the cmake version must be greater than 2.17.
        * `make -jX`, where X is the number of cores of our computer.
    4. Copy the `libdarknet.so` file to the utils folder of vision_cone_detector:
        * `cp libdarknet.so path/DRIVERLESS/src/perception/vision_cone_detector/src/detector/utils`
2. Download the weights of the neural network and configure the paths:
    1. Download the [weights](https://github.com/ARUSfs/DRIVERLESS/releases/download/weights/weights.zip).
    2. Unzip the file and copy the `WEIGHTS` folder to the desired location.
    3. Edit the `WEIGHTS/cones.data` file with the correct paths for our system.
    4. Edit the `DRIVERLESS/src/perception/vision_cone_detector/config/vision.yaml` file with the correct paths for our system. Also, we can add here the distance coefficients and the homography matrix.
3. Send the video to the node. We have several options:
    1. If we want to use a live camera, we must look for the device number assigned to the camera (on a laptop, it will usually be 2). To do this, we run the following command:
    * `ls -ltrh /dev/video*`, the device number is the last number that appears on the line.
    * Edit the `DRIVERLESS/src/perception/imycamera/config/camera.yaml` file with the correct device number. We must also modify the `fps` and the `width` and `height` of the image.
    2. If we want to use a video already recorded we can simulate a video device with the following command:
    * `sudo modprobe v4l2loopback video_nr=2 card_label="MyFakeCam" exclusive_caps=1`, the device number is the one we want.
    * `ffmpeg -re -i video.mp4 -map 0:v -f v4l2 /dev/video2`, where `video.mp4` is the video we want to send and `video2` is the device number we have chosen.
    3. We can also use a video already recorded without simulating a video device:
    * Edit the `DRIVERLESS/src/perception/imycamera/config/camera.yaml` file with the correct video path. We must also modify the `fps` and the `width` and `height` of the image.
    4. Finally, we can send this video frame to frame to the topic using the following [script](https://github.com/ARUSfs/tools_and_ideas/blob/main/Jorge/Perception_utils/publish_video_frames.py):
    * Edit the script with the correct video path and the fps we want.
    * Run with `./publish_video_frames.py`
4. Run the node:
    * `roscore` in a terminal.
    * `roslaunch vision_cone_detector vision_cone_detector.launch` in another terminal.


