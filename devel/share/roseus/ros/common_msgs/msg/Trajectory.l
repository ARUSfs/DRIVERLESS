;; Auto-generated. Do not edit!


(when (boundp 'common_msgs::Trajectory)
  (if (not (find-package "COMMON_MSGS"))
    (make-package "COMMON_MSGS"))
  (shadow 'Trajectory (find-package "COMMON_MSGS")))
(unless (find-package "COMMON_MSGS::TRAJECTORY")
  (make-package "COMMON_MSGS::TRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude Trajectory.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass common_msgs::Trajectory
  :super ros::object
  :slots (_header _trajectory ))

(defmethod common_msgs::Trajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:trajectory __trajectory) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _trajectory __trajectory)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _trajectory
    (apply #'+ (send-all _trajectory :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _trajectory
     (write-long (length _trajectory) s)
     (dolist (elem _trajectory)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _trajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectory (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _trajectory)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get common_msgs::Trajectory :md5sum-) "28d4d6487a6df8edfb1f7eac984e8e4b")
(setf (get common_msgs::Trajectory :datatype-) "common_msgs/Trajectory")
(setf (get common_msgs::Trajectory :definition-)
      "std_msgs/Header header

geometry_msgs/Point[] trajectory

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :common_msgs/Trajectory "28d4d6487a6df8edfb1f7eac984e8e4b")


