; Auto-generated. Do not edit!


(cl:in-package general_service_2022-msg)


;//! \htmlinclude YoloResult.msg.html

(cl:defclass <YoloResult> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (result
    :reader result
    :initarg :result
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass YoloResult (<YoloResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YoloResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YoloResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name general_service_2022-msg:<YoloResult> is deprecated: use general_service_2022-msg:YoloResult instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <YoloResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader general_service_2022-msg:header-val is deprecated.  Use general_service_2022-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <YoloResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader general_service_2022-msg:image-val is deprecated.  Use general_service_2022-msg:image instead.")
  (image m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <YoloResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader general_service_2022-msg:result-val is deprecated.  Use general_service_2022-msg:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YoloResult>) ostream)
  "Serializes a message object of type '<YoloResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'result) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YoloResult>) istream)
  "Deserializes a message object of type '<YoloResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'result) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YoloResult>)))
  "Returns string type for a message object of type '<YoloResult>"
  "general_service_2022/YoloResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YoloResult)))
  "Returns string type for a message object of type 'YoloResult"
  "general_service_2022/YoloResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YoloResult>)))
  "Returns md5sum for a message object of type '<YoloResult>"
  "b1b7078dec013296b04efb925d9d4b21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YoloResult)))
  "Returns md5sum for a message object of type 'YoloResult"
  "b1b7078dec013296b04efb925d9d4b21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YoloResult>)))
  "Returns full string definition for message of type '<YoloResult>"
  (cl:format cl:nil "Header header~%sensor_msgs/Image image~%std_msgs/String result~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YoloResult)))
  "Returns full string definition for message of type 'YoloResult"
  (cl:format cl:nil "Header header~%sensor_msgs/Image image~%std_msgs/String result~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YoloResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YoloResult>))
  "Converts a ROS message object to a list"
  (cl:list 'YoloResult
    (cl:cons ':header (header msg))
    (cl:cons ':image (image msg))
    (cl:cons ':result (result msg))
))
