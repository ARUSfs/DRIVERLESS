#!/usr/bin/env python3

import cv2
import numpy as np
from darknet import darknet
import time
from tqdm import tqdm
import rospkg

# Class to get the path of ROS packages
rospack=rospkg.RosPack()

def frames(file: str):
    """Crea un array de algunos frames del vídeo 
    
    Args:
        file (str): ruta al vídeo
        
    Return:
        numpy.array: vector con los frames extraídos"""
    vid = cv2.VideoCapture(file)
    lista = []
    i = 0
    while vid.grab():
        ret, frame = vid.read()
        if ret and (i%10==0):
            frame = cv2.resize(frame, (1920,1088))
            lista.append(frame)
        i += 1
    vid.release()
    return lista

def intrinsecas(video:str, chess_size=(9,6), square_size=0.00239):
    """
    Devuelve mtx, dist, rvecs, tvecs
    """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((chess_size[0]*chess_size[1],3), np.float32)
    objp[:,:2] = square_size*np.mgrid[0:chess_size[0],0:chess_size[1]].T.reshape(-1,2)
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    imgs = frames(video)

    for img in tqdm(imgs):
        gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gris, chess_size)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gris,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
    ret,mtx,dist,rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gris.shape[::-1],None,None)
    return cv2.getOptimalNewCameraMatrix(mtx,dist,gris.shape[::-1],0)[0], dist, rvecs, tvecs

def sort_detections_manual(cones_detected_info, pixel_error):
    sorted_cones = []
    y_sort = sorted(cones_detected_info, key=lambda cone: cone[3][1], reverse=True)
    for tip in y_sort:
        x_sort = []
        level = list(filter(lambda cone: tip[3][1] - pixel_error < cone[3][1] < tip[3][1] + pixel_error, cones_detected_info))
        if len(level) > 0:
            x_sort = sorted(level, key=lambda cone: cone[3][0])
            sorted_cones.extend(x_sort)
            
    sorted_cones_set = []
    for i,cone in enumerate(sorted_cones):
        if cone not in sorted_cones_set:
            sorted_cones_set.append(cone)

    return sorted_cones_set

def cones_perception(detection):
    """Obtener las bounding box de los conos de la imagen dada.
    
    Args:
        detection (list): contains darknet detections
    
    Return:
        list: coordinates of each bounding box with its prediction
    """
    
    cones_detected_info = []
    for label,confidence,box in detection:
        xmin,ymin,xmax,_ = darknet.bbox2points(box)
        mid = xmin + ((xmax-xmin) / 2)
        cones_detected_info.append((label, confidence, box, [mid, ymin]))
    
    # Sorting cones_info first by x axis, then, y axis
    sorted_detections = sort_detections_manual(cones_detected_info, 20)
        
    return np.array([t for _,_,_, t in sorted_detections],np.float32)
    
def yolo_detect(img, cfg, weights, data):
    """Obtener los pixeles de la punta de los conos
    
    Args:
        img (numpy.ndarray): image from which get pixels
        cfg (str): config file path
        weights (str): weights file path
        data (str): data file path
    
    Return:
        list: detected cones 
    """
    net, names, _ = darknet.load_network(cfg, data, weights)
    w_net, h_net = darknet.network_width(net), darknet.network_height(net)
    frame = cv2.resize(img, (w_net, h_net))
    d_img = darknet.make_image(w_net, h_net, 3) 
    darknet.copy_image_from_bytes(d_img,frame.tobytes())
    detections = darknet.detect_image(net, names, d_img)
    return detections

def ordena_pixeles(cones_detected):
    """Sort the cones detected in the image"""
    
    c1 = sorted(cones_detected, key=lambda x: x[1])[::-1][:2]
    c2 = sorted(c1, key=lambda x: x[0])
    c3 = sorted(cones_detected, key=lambda x: x[1])[:2]
    c4 = sorted(c3, key=lambda x: x[0])
    c = np.zeros((4,2))
    c[0], c[1], c[2], c[3] = c2[0], c2[1], c4[0], c4[1]
    return c

def escribe_txt(file,mat):
    """Crea un archivo txt (o escribe en él si existe) y escribe en él la matriz
    
    Args:
        file (str): directorio del archivo a escribir
        mat (numpy.array): matriz para escribir
    """
    np.savetxt(file, mat, fmt='%-1g')




# Paths
path_detect = rospack.get_path('cam_detect')
path_calib = rospack.get_path('cam_calibration')

# Correct path in cones.data automatically
with open(path_detect + '/weights/cones.data', 'r') as data_file:
        lines = data_file.readlines()
lines[3] = 'names = ' + path_detect + '/weights/cones.names' + '\n'  
with open(path_detect + '/weights/cones.data', 'w') as archivo:
    archivo.writelines(lines)

cfg = path_detect + '/weights/cones-customanchors.cfg'
data = path_detect + '/weights/cones.data'
weights = path_detect + '/weights/cones5.weights'

vid = path_calib + '/data/chessvid.mp4' # Chessboard video
image = path_calib + '/data/conos.png' # Cones image
text = path_calib + '/data/conos.txt' # File with the cones real coordinates







# Get real cones coordinates from file
cones = np.loadtxt(text, dtype=np.float64)

# Get intrinsic matrix and distortion coefficients
m_int, dist, _, _ = intrinsecas(video=vid)

# Set up the image for detection
img_dist = cv2.imread(image)
img_res = cv2.resize(img_dist, (1920,1088))
img_und = cv2.undistort(img_res, m_int, dist)
img = cv2.cvtColor(img_und, cv2.COLOR_BGR2RGB)

# Detection
detections = yolo_detect(img, cfg, weights, data)
cones_detected = cones_perception(detections)
sorted_cones = ordena_pixeles(cones_detected)

# Get the rotation and translation vectors
_, rvec1, tvec1 = cv2.solvePnP(cones, sorted_cones, m_int, dist, cv2.SOLVEPNP_IPPE)
_, rvec2, tvec2 = cv2.solvePnP(cones, sorted_cones, m_int, dist, rvec1, tvec1, True)

rmat = cv2.Rodrigues(rvec2)[0]
hmat = rmat.copy()
hmat[:, 2] = tvec2.T
H = np.linalg.inv(m_int @ hmat)

# Calcula la matriz de la homografía por findHomografy. Otra opción posible
#H = cv2.findHomography(sorted_cones, cones)[0]

#Guardado de datos
escribe_txt(path_detect + '/data/mat_int.txt', m_int) # Matriz intrínseca
escribe_txt(path_detect + '/data/dist.txt', dist) # Coeficientes de distorsión
escribe_txt(path_calib + '/detecciones.txt', sorted_cones) # Coordenadas de los conos detectados
escribe_txt(path_detect + '/data/mat_hom.txt', H) # Matriz de homografía
