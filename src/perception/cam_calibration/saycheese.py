#!/usr/bin/env python3

import cv2
import time
import rospkg
# Class to get the path of ROS packages
rospack=rospkg.RosPack()

### Configuration parameters

# Cam info
cam_index = 0

# Video duration in seconds
T=45

# ROS pkg path
path_calib = rospack.get_path('cam_calibration')


### Functions
def foto(index=0):
    cam = cv2.VideoCapture(index)
    ret, img = cam.read()
    if ret:
        cv2.imwrite(path_calib + '/data/conos.png',img)
    cam.release()
    
def video(dur, index=0):
    cam = cv2.VideoCapture(index)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    w = 1920
    h = 1088
    fps = cam.get(cv2.CAP_PROP_FPS)
    out = cv2.VideoWriter(path_calib + '/data/chessvid.mp4', fourcc, fps, (w,h))
    t0=time.time()

    while ((time.time() - t0) <= dur):
        ret, frame = cam.read()
        if not ret:
            raise Exception('Camera error.')
        if cv2.waitKey(1) == ord('q'):
            break
        cv2.imshow('Recording...', frame)
        if (frame.shape[1] != w) or (frame.shape[0] != h):
            frame = cv2.resize(frame, (w,h))
        out.write(frame)
    cam.release()
    out.release()
    cv2.destroyAllWindows()


### Main
if __name__ == '__main__':
    input('Presiona enter para hacer una foto.')
    foto(index=cam_index)
    input('Presiona enter para hacer un vídeo.')
    video(T, index=cam_index)
    print('Completado.')
