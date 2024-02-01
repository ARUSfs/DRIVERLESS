#!/usr/bin/env python3

import cv2
import time

def foto():
    cam = cv2.VideoCapture(0)
    ret, img = cam.read()
    if ret:
        cv2.imwrite('data/conosimg.png',img)
    cam.release()
    
def video(dur):
    cam = cv2.VideoCapture(0)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    w = 1920
    h = 1088
    fps = cam.get(cv2.CAP_PROP_FPS)
    out = cv2.VideoWriter('data/chessvid.mp4', fourcc, fps, (w,h))
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
foto()
input('Presiona enter para hacer un vídeo.')
video(45)
print('Completado.')
