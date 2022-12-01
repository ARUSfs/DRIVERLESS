;; Auto-generated. Do not edit!


(when (boundp 'common_msgs::Controls)
  (if (not (find-package "COMMON_MSGS"))
    (make-package "COMMON_MSGS"))
  (shadow 'Controls (find-package "COMMON_MSGS")))
(unless (find-package "COMMON_MSGS::CONTROLS")
  (make-package "COMMON_MSGS::CONTROLS"))

(in-package "ROS")
;;//! \htmlinclude Controls.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass common_msgs::Controls
  :super ros::object
  :slots (_header _steering _accelerator ))

(defmethod common_msgs::Controls
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:steering __steering) 0.0)
    ((:accelerator __accelerator) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _steering (float __steering))
   (setq _accelerator (float __accelerator))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:accelerator
   (&optional __accelerator)
   (if __accelerator (setq _accelerator __accelerator)) _accelerator)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _steering
    8
    ;; float64 _accelerator
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _steering
       (sys::poke _steering (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _accelerator
       (sys::poke _accelerator (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _steering
     (setq _steering (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _accelerator
     (setq _accelerator (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get common_msgs::Controls :md5sum-) "cbd78afea6162eac878009c60d81e295")
(setf (get common_msgs::Controls :datatype-) "common_msgs/Controls")
(setf (get common_msgs::Controls :definition-)
      "std_msgs/Header header

float64 steering
float64 accelerator






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

")



(provide :common_msgs/Controls "cbd78afea6162eac878009c60d81e295")


