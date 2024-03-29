# CUDA Camera Perception 

## Introduction

We use CUDA in order to make faster detections and use it while the car is on track. CUDA instalation is explained in [Environment setup](https://arusfs.github.io/DRIVERLESS/content/environment_setup/instalarEntorno.html#installation-of-cuda-and-cudnn). In this page I will only explain how to run the Camera Perception Node using the CUDA acceleration and I will consider that our GitHub [repository](https://github.com/ARUSfs/DRIVERLESS) is already installed.

## Installation of dependencies

1. Make sure your CUDA installation is correct and check your GPU model:
    - Run `nvidia-smi`. If everything is OK continue with the next step.

2. Check the number of cores of your system GPU:
    - Run `nproc`

3. Search your GPU's compute capability (GPUcc):
    - Find this information in the official NVIDIA [website](https://developer.nvidia.com/cuda-gpus).

4. Clone the `darknet` fork in your system:
    - Run `git clone https://github.com/ARUSfs/darknet.git` in a terminal.

5. Navigate to the installed darknet folder:
    - Run `cd darknet`

6. Modify the _Makefile_ file with a text editor (i.e. _nano_) to fit your system's specifications. These are my specifications:
    - Set `GPU=1`
    - Set `CUDNN=1`
    - Set `CUDNN_HALF=0`
    - Set `OPENCV=0`
    - Set `AVX=0`
    - Set `OPENMP=0`
    - Set `LIBSO=1`
    - Set `ZED_CAMERA=0`
    - Set `ZED_CAMERA_v2_8=0`
    - Set `NUMPY=1`
    - Set `ARCH= -gencode arch=compute_86,code=[sm_86,compute_86]` to fit 10 times of your GPUcc (mine is 8.6).

7. In the terminal:
    - Run `make -jX`. Change X to your GPU number of cores (i.e. `make -j12`).

8. If no errors occurs, copy the _libdarknet.so_ file to the `/cam_perception/src/darknet` and `/cam_calibration/darknet` folders in the `perception` package.

### Known errors and fixes


```{figure} error_opencv.png
:alt: ERROR-OpenCV
:scale: 25
:align: "center"
:target: https://gyazo.com/26d62015526a693f723b709b7a5396a3

Do not use `OPENCV=1`, change it to the `0` value.
```
