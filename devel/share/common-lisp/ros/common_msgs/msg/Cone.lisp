; Auto-generated. Do not edit!


(cl:in-package common_msgs-msg)


;//! \htmlinclude Cone.msg.html

(cl:defclass <Cone> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (color
    :reader color
    :initarg :color
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass Cone (<Cone>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cone>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cone)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name common_msgs-msg:<Cone> is deprecated: use common_msgs-msg:Cone instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Cone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:position-val is deprecated.  Use common_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <Cone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:color-val is deprecated.  Use common_msgs-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <Cone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader common_msgs-msg:confidence-val is deprecated.  Use common_msgs-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cone>) ostream)
  "Serializes a message object of type '<Cone>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'color))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cone>) istream)
  "Deserializes a message object of type '<Cone>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'color) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cone>)))
  "Returns string type for a message object of type '<Cone>"
  "common_msgs/Cone")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cone)))
  "Returns string type for a message object of type 'Cone"
  "common_msgs/Cone")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cone>)))
  "Returns md5sum for a message object of type '<Cone>"
  "43bb11ea9f3016ef6978a1dc3a27791d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cone)))
  "Returns md5sum for a message object of type 'Cone"
  "43bb11ea9f3016ef6978a1dc3a27791d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cone>)))
  "Returns full string definition for message of type '<Cone>"
  (cl:format cl:nil "geometry_msgs/Point position #x,y,z [m]~%string color # 'b', 'y' or 'o'~%float64 confidence # Confidence of class~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cone)))
  "Returns full string definition for message of type 'Cone"
  (cl:format cl:nil "geometry_msgs/Point position #x,y,z [m]~%string color # 'b', 'y' or 'o'~%float64 confidence # Confidence of class~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cone>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4 (cl:length (cl:slot-value msg 'color))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cone>))
  "Converts a ROS message object to a list"
  (cl:list 'Cone
    (cl:cons ':position (position msg))
    (cl:cons ':color (color msg))
    (cl:cons ':confidence (confidence msg))
))
