;; Auto-generated. Do not edit!


(when (boundp 'common_msgs::Map)
  (if (not (find-package "COMMON_MSGS"))
    (make-package "COMMON_MSGS"))
  (shadow 'Map (find-package "COMMON_MSGS")))
(unless (find-package "COMMON_MSGS::MAP")
  (make-package "COMMON_MSGS::MAP"))

(in-package "ROS")
;;//! \htmlinclude Map.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass common_msgs::Map
  :super ros::object
  :slots (_header _cones ))

(defmethod common_msgs::Map
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cones __cones) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _cones __cones)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cones
   (&rest __cones)
   (if (keywordp (car __cones))
       (send* _cones __cones)
     (progn
       (if __cones (setq _cones (car __cones)))
       _cones)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; common_msgs/Cone[] _cones
    (apply #'+ (send-all _cones :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; common_msgs/Cone[] _cones
     (write-long (length _cones) s)
     (dolist (elem _cones)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; common_msgs/Cone[] _cones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cones (let (r) (dotimes (i n) (push (instance common_msgs::Cone :init) r)) r))
     (dolist (elem- _cones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get common_msgs::Map :md5sum-) "22b932b22361d024acd209fcef745d8a")
(setf (get common_msgs::Map :datatype-) "common_msgs/Map")
(setf (get common_msgs::Map :definition-)
      "std_msgs/Header header

common_msgs/Cone[] cones

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: common_msgs/Cone
geometry_msgs/Point position #x,y,z [m]
string color # 'b', 'y' or 'o'
float64 confidence # Confidence of class

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :common_msgs/Map "22b932b22361d024acd209fcef745d8a")


