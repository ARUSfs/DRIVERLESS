#!/usr/bin/env python3

import cv2
import numpy as np
import darknet
import time


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
        ret, frame = vid.retrieve()
        if ret and (i%15==0):
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

    for img in imgs:
        gris = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gris, chess_size)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gris,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
    ret,mtx,dist,rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gris.shape[::-1],None,None)
    return cv2.getOptimalNewCameraMatrix(mtx,dist,gris.shape[::-1],0)[0], dist, rvecs, tvecs

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

def real_coords_txt(file):
    """Obtener las posiciones reales de los conos desde un .txt.    
    FORMATO: en cada línea 3 números separados por comas representando cada cono.
    
    Args:
        file (str): path to txt
    
    Return
        numpy.array: matriz con las coordenadas de los conos
    """
    conos = open(file,'r')
    conos_c = [(x.replace('\n','')).split(',') for x in conos.readlines() if x!='\n']
    conos_coords = np.array([(float(x),float(y),float(z)) for x,y,z in conos_c],np.float32)
    conos.close()
    return conos_coords

def homo_mat(video, img, txt, cfg, weights, data):
    """Obtiene los vectores de rotación y traslación
    
    Args:
        img (numpy.ndarray): imagen de los conos
        txt (str): ruta del txt con coords de los conos
        cfg (str): ruta archivo config
        weights (str): ruta archivo weights 
        data (str): ruta archivo data
        folder (str='.'): ruta carpeta con fotos tablero
        typ (str='png'): tipo de imagen a
        
    Return:
        numpy.array: mtx, dist, rvec, tvec
    """
    mtx, dist, _, _ = intrinsecas(video)
    img0 = cv2.imread(img)
    img_und = cv2.undistort((cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)),mtx,dist)
    detection = yolo_detect(img_und, cfg, weights, data)
    conos_pixeles = cones_perception(detection)
    conos_coords = real_coords_txt(txt)
    _, rvec,tvec = cv2.solvePnP(conos_coords, conos_pixeles, mtx, dist, flags=cv2.SOLVEPNP_IPPE)
    return mtx, dist, rvec, tvec, conos_pixeles

def escribe_txt(file,mat):
    """Crea un archivo txt (o escribe en él si existe) y escribe en él la matriz
    
    Args:
        file (str): directorio del archivo a escribir
        mat (numpy.array): matriz para escribir
    """
    np.savetxt(file,mat, fmt='%-1g')


#Parámetros propios
cfg = '/home/igsais/ros_ws/src/WEIGHTS/cones-customanchors.cfg'
data = '/home/igsais/ros_ws/src/WEIGHTS/cones.data'
weights = '/home/igsais/ros_ws/src/WEIGHTS/cones5.weights'
v = 'chessvid.mp4'
i = 'conosimg.png'
t = 'conos.txt'

mtx, dist, rvec, tvec, conos_pix = homo_mat(v, i, t,cfg, weights, data)

#Cálculos para hallar la matriz de homografía H
rmat = cv2.Rodrigues(rvec)[0]
hmat = rmat.copy()
hmat[:, 2] = hmat[:, 2] * 0.32 + tvec.T
H = np.linalg.inv(mtx @ hmat)

#Guardado de datos
escribe_txt('../matint.txt',mtx)
escribe_txt('./detecciones.txt', conos_pix)
escribe_txt('../mathom.txt', H)
escribe_txt('./dist.txt', dist)
