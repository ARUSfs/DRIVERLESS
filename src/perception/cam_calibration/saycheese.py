#!/usr/bin/env python3

import cv2
import time
import rospkg
rospack=rospkg.RosPack()

# Datos de la cámara
cam_index = 0
T=45 # segundos de vídeo
path_calib = rospack.get_path('cam_calibration')

def foto(index=0):
    cam = cv2.VideoCapture(index)
    ret, img = cam.read()
    if ret:
        cv2.imwrite(path_calib + '/data/conosimg.png',img)
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
            print('Error en la cámara')
            break
        if cv2.waitKey(1) == ord('q'):
            break
        cv2.imshow('r', frame)
        if (frame.shape[1] != w) or (frame.shape[0] != h):
            frame = cv2.resize(frame, (w,h))
        out.write(frame)
    cam.release()
    out.release()
    cv2.destroyAllWindows()



input('Presiona enter para hacer una foto.')
foto(index=cam_index)
input('Presiona enter para hacer un vídeo.')
video(T, index=cam_index)
print('Completado.')
