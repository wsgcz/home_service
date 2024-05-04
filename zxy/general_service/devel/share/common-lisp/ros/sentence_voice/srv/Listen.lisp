; Auto-generated. Do not edit!


(cl:in-package sentence_voice-srv)


;//! \htmlinclude Listen-request.msg.html

(cl:defclass <Listen-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (duration
    :reader duration
    :initarg :duration
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Listen-request (<Listen-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Listen-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Listen-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sentence_voice-srv:<Listen-request> is deprecated: use sentence_voice-srv:Listen-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Listen-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sentence_voice-srv:header-val is deprecated.  Use sentence_voice-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Listen-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sentence_voice-srv:duration-val is deprecated.  Use sentence_voice-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Listen-request>) ostream)
  "Serializes a message object of type '<Listen-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'duration)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Listen-request>) istream)
  "Deserializes a message object of type '<Listen-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duration) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Listen-request>)))
  "Returns string type for a service object of type '<Listen-request>"
  "sentence_voice/ListenRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Listen-request)))
  "Returns string type for a service object of type 'Listen-request"
  "sentence_voice/ListenRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Listen-request>)))
  "Returns md5sum for a message object of type '<Listen-request>"
  "9960a830d15d58a17858801d6d464529")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Listen-request)))
  "Returns md5sum for a message object of type 'Listen-request"
  "9960a830d15d58a17858801d6d464529")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Listen-request>)))
  "Returns full string definition for message of type '<Listen-request>"
  (cl:format cl:nil "# 发送的请求为持续听的时间~%std_msgs/Header header~%int16 duration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Listen-request)))
  "Returns full string definition for message of type 'Listen-request"
  (cl:format cl:nil "# 发送的请求为持续听的时间~%std_msgs/Header header~%int16 duration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Listen-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Listen-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Listen-request
    (cl:cons ':header (header msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude Listen-response.msg.html

(cl:defclass <Listen-response> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (sentence
    :reader sentence
    :initarg :sentence
    :type cl:string
    :initform ""))
)

(cl:defclass Listen-response (<Listen-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Listen-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Listen-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sentence_voice-srv:<Listen-response> is deprecated: use sentence_voice-srv:Listen-response instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Listen-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sentence_voice-srv:header-val is deprecated.  Use sentence_voice-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'sentence-val :lambda-list '(m))
(cl:defmethod sentence-val ((m <Listen-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sentence_voice-srv:sentence-val is deprecated.  Use sentence_voice-srv:sentence instead.")
  (sentence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Listen-response>) ostream)
  "Serializes a message object of type '<Listen-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sentence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sentence))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Listen-response>) istream)
  "Deserializes a message object of type '<Listen-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sentence) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sentence) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Listen-response>)))
  "Returns string type for a service object of type '<Listen-response>"
  "sentence_voice/ListenResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Listen-response)))
  "Returns string type for a service object of type 'Listen-response"
  "sentence_voice/ListenResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Listen-response>)))
  "Returns md5sum for a message object of type '<Listen-response>"
  "9960a830d15d58a17858801d6d464529")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Listen-response)))
  "Returns md5sum for a message object of type 'Listen-response"
  "9960a830d15d58a17858801d6d464529")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Listen-response>)))
  "Returns full string definition for message of type '<Listen-response>"
  (cl:format cl:nil "# 返回的响应为听写的结果~%std_msgs/Header header~%string sentence~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Listen-response)))
  "Returns full string definition for message of type 'Listen-response"
  (cl:format cl:nil "# 返回的响应为听写的结果~%std_msgs/Header header~%string sentence~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Listen-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'sentence))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Listen-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Listen-response
    (cl:cons ':header (header msg))
    (cl:cons ':sentence (sentence msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Listen)))
  'Listen-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Listen)))
  'Listen-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Listen)))
  "Returns string type for a service object of type '<Listen>"
  "sentence_voice/Listen")