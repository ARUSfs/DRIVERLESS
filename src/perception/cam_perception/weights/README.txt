This zip includes the necessary files for running the perception package. The main files are

 + `cones5.weights`: Weights for yolov4-tiny
 + `keypoint_weights.pth`: Weights for keypoint NN

 The files cones.data and cones-customanchors.cfg are also needed for YOLO, and must be configured
 in the `vision.yaml` config file.

 ============================================ IMPORTANT ============================================
 Paths in the `vision.yaml` and `cones.data` files must be absolute paths. Thus, it must be changed
 the first time they are used.
