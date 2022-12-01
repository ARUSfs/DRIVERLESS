;; Auto-generated. Do not edit!


(when (boundp 'common_msgs::Cone)
  (if (not (find-package "COMMON_MSGS"))
    (make-package "COMMON_MSGS"))
  (shadow 'Cone (find-package "COMMON_MSGS")))
(unless (find-package "COMMON_MSGS::CONE")
  (make-package "COMMON_MSGS::CONE"))

(in-package "ROS")
;;//! \htmlinclude Cone.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass common_msgs::Cone
  :super ros::object
  :slots (_position _color _confidence ))

(defmethod common_msgs::Cone
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:color __color) "")
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _position __position)
   (setq _color (string __color))
   (setq _confidence (float __confidence))
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; string _color
    4 (length _color)
    ;; float64 _confidence
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;; float64 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _confidence
     (setq _confidence (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get common_msgs::Cone :md5sum-) "43bb11ea9f3016ef6978a1dc3a27791d")
(setf (get common_msgs::Cone :datatype-) "common_msgs/Cone")
(setf (get common_msgs::Cone :definition-)
      "geometry_msgs/Point position #x,y,z [m]
string color # 'b', 'y' or 'o'
float64 confidence # Confidence of class

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :common_msgs/Cone "43bb11ea9f3016ef6978a1dc3a27791d")


