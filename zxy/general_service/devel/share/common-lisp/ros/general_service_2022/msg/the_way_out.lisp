; Auto-generated. Do not edit!


(cl:in-package general_service_2022-msg)


;//! \htmlinclude the_way_out.msg.html

(cl:defclass <the_way_out> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:integer
    :initform 0)
   (max_length
    :reader max_length
    :initarg :max_length
    :type cl:float
    :initform 0.0)
   (the_angle_of_max_length
    :reader the_angle_of_max_length
    :initarg :the_angle_of_max_length
    :type cl:integer
    :initform 0))
)

(cl:defclass the_way_out (<the_way_out>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <the_way_out>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'the_way_out)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name general_service_2022-msg:<the_way_out> is deprecated: use general_service_2022-msg:the_way_out instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <the_way_out>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader general_service_2022-msg:angle-val is deprecated.  Use general_service_2022-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'max_length-val :lambda-list '(m))
(cl:defmethod max_length-val ((m <the_way_out>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader general_service_2022-msg:max_length-val is deprecated.  Use general_service_2022-msg:max_length instead.")
  (max_length m))

(cl:ensure-generic-function 'the_angle_of_max_length-val :lambda-list '(m))
(cl:defmethod the_angle_of_max_length-val ((m <the_way_out>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader general_service_2022-msg:the_angle_of_max_length-val is deprecated.  Use general_service_2022-msg:the_angle_of_max_length instead.")
  (the_angle_of_max_length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <the_way_out>) ostream)
  "Serializes a message object of type '<the_way_out>"
  (cl:let* ((signed (cl:slot-value msg 'angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'the_angle_of_max_length)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <the_way_out>) istream)
  "Deserializes a message object of type '<the_way_out>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_length) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'the_angle_of_max_length) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<the_way_out>)))
  "Returns string type for a message object of type '<the_way_out>"
  "general_service_2022/the_way_out")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'the_way_out)))
  "Returns string type for a message object of type 'the_way_out"
  "general_service_2022/the_way_out")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<the_way_out>)))
  "Returns md5sum for a message object of type '<the_way_out>"
  "be349f47f00195efdd7bf46fc8b5963c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'the_way_out)))
  "Returns md5sum for a message object of type 'the_way_out"
  "be349f47f00195efdd7bf46fc8b5963c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<the_way_out>)))
  "Returns full string definition for message of type '<the_way_out>"
  (cl:format cl:nil "int32 angle~%float64 max_length~%int32 the_angle_of_max_length~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'the_way_out)))
  "Returns full string definition for message of type 'the_way_out"
  (cl:format cl:nil "int32 angle~%float64 max_length~%int32 the_angle_of_max_length~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <the_way_out>))
  (cl:+ 0
     4
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <the_way_out>))
  "Converts a ROS message object to a list"
  (cl:list 'the_way_out
    (cl:cons ':angle (angle msg))
    (cl:cons ':max_length (max_length msg))
    (cl:cons ':the_angle_of_max_length (the_angle_of_max_length msg))
))
