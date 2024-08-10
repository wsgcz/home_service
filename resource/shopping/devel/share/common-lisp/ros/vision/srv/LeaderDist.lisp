; Auto-generated. Do not edit!


(cl:in-package vision-srv)


;//! \htmlinclude LeaderDist-request.msg.html

(cl:defclass <LeaderDist-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass LeaderDist-request (<LeaderDist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LeaderDist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LeaderDist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<LeaderDist-request> is deprecated: use vision-srv:LeaderDist-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LeaderDist-request>) ostream)
  "Serializes a message object of type '<LeaderDist-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LeaderDist-request>) istream)
  "Deserializes a message object of type '<LeaderDist-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LeaderDist-request>)))
  "Returns string type for a service object of type '<LeaderDist-request>"
  "vision/LeaderDistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LeaderDist-request)))
  "Returns string type for a service object of type 'LeaderDist-request"
  "vision/LeaderDistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LeaderDist-request>)))
  "Returns md5sum for a message object of type '<LeaderDist-request>"
  "acff7fda0c683c872875f5c4b069f124")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LeaderDist-request)))
  "Returns md5sum for a message object of type 'LeaderDist-request"
  "acff7fda0c683c872875f5c4b069f124")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LeaderDist-request>)))
  "Returns full string definition for message of type '<LeaderDist-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LeaderDist-request)))
  "Returns full string definition for message of type 'LeaderDist-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LeaderDist-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LeaderDist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LeaderDist-request
))
;//! \htmlinclude LeaderDist-response.msg.html

(cl:defclass <LeaderDist-response> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass LeaderDist-response (<LeaderDist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LeaderDist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LeaderDist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<LeaderDist-response> is deprecated: use vision-srv:LeaderDist-response instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <LeaderDist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:distance-val is deprecated.  Use vision-srv:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LeaderDist-response>) ostream)
  "Serializes a message object of type '<LeaderDist-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LeaderDist-response>) istream)
  "Deserializes a message object of type '<LeaderDist-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LeaderDist-response>)))
  "Returns string type for a service object of type '<LeaderDist-response>"
  "vision/LeaderDistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LeaderDist-response)))
  "Returns string type for a service object of type 'LeaderDist-response"
  "vision/LeaderDistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LeaderDist-response>)))
  "Returns md5sum for a message object of type '<LeaderDist-response>"
  "acff7fda0c683c872875f5c4b069f124")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LeaderDist-response)))
  "Returns md5sum for a message object of type 'LeaderDist-response"
  "acff7fda0c683c872875f5c4b069f124")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LeaderDist-response>)))
  "Returns full string definition for message of type '<LeaderDist-response>"
  (cl:format cl:nil "float64 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LeaderDist-response)))
  "Returns full string definition for message of type 'LeaderDist-response"
  (cl:format cl:nil "float64 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LeaderDist-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LeaderDist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LeaderDist-response
    (cl:cons ':distance (distance msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LeaderDist)))
  'LeaderDist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LeaderDist)))
  'LeaderDist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LeaderDist)))
  "Returns string type for a service object of type '<LeaderDist>"
  "vision/LeaderDist")