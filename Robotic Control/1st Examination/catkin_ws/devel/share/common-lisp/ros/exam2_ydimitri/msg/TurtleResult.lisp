; Auto-generated. Do not edit!


(cl:in-package exam2_ydimitri-msg)


;//! \htmlinclude TurtleResult.msg.html

(cl:defclass <TurtleResult> (roslisp-msg-protocol:ros-message)
  ((elapsed_time
    :reader elapsed_time
    :initarg :elapsed_time
    :type cl:real
    :initform 0))
)

(cl:defclass TurtleResult (<TurtleResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TurtleResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TurtleResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name exam2_ydimitri-msg:<TurtleResult> is deprecated: use exam2_ydimitri-msg:TurtleResult instead.")))

(cl:ensure-generic-function 'elapsed_time-val :lambda-list '(m))
(cl:defmethod elapsed_time-val ((m <TurtleResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader exam2_ydimitri-msg:elapsed_time-val is deprecated.  Use exam2_ydimitri-msg:elapsed_time instead.")
  (elapsed_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TurtleResult>) ostream)
  "Serializes a message object of type '<TurtleResult>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'elapsed_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'elapsed_time) (cl:floor (cl:slot-value msg 'elapsed_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TurtleResult>) istream)
  "Deserializes a message object of type '<TurtleResult>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'elapsed_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TurtleResult>)))
  "Returns string type for a message object of type '<TurtleResult>"
  "exam2_ydimitri/TurtleResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TurtleResult)))
  "Returns string type for a message object of type 'TurtleResult"
  "exam2_ydimitri/TurtleResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TurtleResult>)))
  "Returns md5sum for a message object of type '<TurtleResult>"
  "1e3d6f1eb13c6d1493c23dbce73b8e5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TurtleResult)))
  "Returns md5sum for a message object of type 'TurtleResult"
  "1e3d6f1eb13c6d1493c23dbce73b8e5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TurtleResult>)))
  "Returns full string definition for message of type '<TurtleResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%duration elapsed_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TurtleResult)))
  "Returns full string definition for message of type 'TurtleResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%duration elapsed_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TurtleResult>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TurtleResult>))
  "Converts a ROS message object to a list"
  (cl:list 'TurtleResult
    (cl:cons ':elapsed_time (elapsed_time msg))
))