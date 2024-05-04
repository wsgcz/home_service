; Auto-generated. Do not edit!


(cl:in-package sentence_voice-srv)


;//! \htmlinclude Speak-request.msg.html

(cl:defclass <Speak-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Speak-request (<Speak-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Speak-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Speak-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sentence_voice-srv:<Speak-request> is deprecated: use sentence_voice-srv:Speak-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Speak-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sentence_voice-srv:header-val is deprecated.  Use sentence_voice-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'sentence-val :lambda-list '(m))
(cl:defmethod sentence-val ((m <Speak-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sentence_voice-srv:sentence-val is deprecated.  Use sentence_voice-srv:sentence instead.")
  (sentence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Speak-request>) ostream)
  "Serializes a message object of type '<Speak-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sentence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sentence))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Speak-request>) istream)
  "Deserializes a message object of type '<Speak-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Speak-request>)))
  "Returns string type for a service object of type '<Speak-request>"
  "sentence_voice/SpeakRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speak-request)))
  "Returns string type for a service object of type 'Speak-request"
  "sentence_voice/SpeakRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Speak-request>)))
  "Returns md5sum for a message object of type '<Speak-request>"
  "6ec12d5a2bfa19afaef4554a69aed1b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Speak-request)))
  "Returns md5sum for a message object of type 'Speak-request"
  "6ec12d5a2bfa19afaef4554a69aed1b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Speak-request>)))
  "Returns full string definition for message of type '<Speak-request>"
  (cl:format cl:nil "# 发送的请求是要说的话~%std_msgs/Header header~%string sentence~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Speak-request)))
  "Returns full string definition for message of type 'Speak-request"
  (cl:format cl:nil "# 发送的请求是要说的话~%std_msgs/Header header~%string sentence~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Speak-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'sentence))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Speak-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Speak-request
    (cl:cons ':header (header msg))
    (cl:cons ':sentence (sentence msg))
))
;//! \htmlinclude Speak-response.msg.html

(cl:defclass <Speak-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Speak-response (<Speak-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Speak-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Speak-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sentence_voice-srv:<Speak-response> is deprecated: use sentence_voice-srv:Speak-response instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Speak-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sentence_voice-srv:header-val is deprecated.  Use sentence_voice-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <Speak-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sentence_voice-srv:duration-val is deprecated.  Use sentence_voice-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Speak-response>) ostream)
  "Serializes a message object of type '<Speak-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'duration)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Speak-response>) istream)
  "Deserializes a message object of type '<Speak-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duration) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Speak-response>)))
  "Returns string type for a service object of type '<Speak-response>"
  "sentence_voice/SpeakResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speak-response)))
  "Returns string type for a service object of type 'Speak-response"
  "sentence_voice/SpeakResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Speak-response>)))
  "Returns md5sum for a message object of type '<Speak-response>"
  "6ec12d5a2bfa19afaef4554a69aed1b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Speak-response)))
  "Returns md5sum for a message object of type 'Speak-response"
  "6ec12d5a2bfa19afaef4554a69aed1b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Speak-response>)))
  "Returns full string definition for message of type '<Speak-response>"
  (cl:format cl:nil "# 返回的响应是说话的时间，-1表示说话失败~%std_msgs/Header header~%int16 duration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Speak-response)))
  "Returns full string definition for message of type 'Speak-response"
  (cl:format cl:nil "# 返回的响应是说话的时间，-1表示说话失败~%std_msgs/Header header~%int16 duration~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Speak-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Speak-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Speak-response
    (cl:cons ':header (header msg))
    (cl:cons ':duration (duration msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Speak)))
  'Speak-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Speak)))
  'Speak-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speak)))
  "Returns string type for a service object of type '<Speak>"
  "sentence_voice/Speak")