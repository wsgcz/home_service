;; Auto-generated. Do not edit!


(when (boundp 'sentence_voice::Listen)
  (if (not (find-package "SENTENCE_VOICE"))
    (make-package "SENTENCE_VOICE"))
  (shadow 'Listen (find-package "SENTENCE_VOICE")))
(unless (find-package "SENTENCE_VOICE::LISTEN")
  (make-package "SENTENCE_VOICE::LISTEN"))
(unless (find-package "SENTENCE_VOICE::LISTENREQUEST")
  (make-package "SENTENCE_VOICE::LISTENREQUEST"))
(unless (find-package "SENTENCE_VOICE::LISTENRESPONSE")
  (make-package "SENTENCE_VOICE::LISTENRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sentence_voice::ListenRequest
  :super ros::object
  :slots (_header _duration ))

(defmethod sentence_voice::ListenRequest
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:duration __duration) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _duration (round __duration))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _duration
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _duration
       (write-word _duration s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _duration
     (setq _duration (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass sentence_voice::ListenResponse
  :super ros::object
  :slots (_header _sentence ))

(defmethod sentence_voice::ListenResponse
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sentence __sentence) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _sentence (string __sentence))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sentence
   (&optional __sentence)
   (if __sentence (setq _sentence __sentence)) _sentence)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _sentence
    4 (length _sentence)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _sentence
       (write-long (length _sentence) s) (princ _sentence s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _sentence
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sentence (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass sentence_voice::Listen
  :super ros::object
  :slots ())

(setf (get sentence_voice::Listen :md5sum-) "9960a830d15d58a17858801d6d464529")
(setf (get sentence_voice::Listen :datatype-) "sentence_voice/Listen")
(setf (get sentence_voice::Listen :request) sentence_voice::ListenRequest)
(setf (get sentence_voice::Listen :response) sentence_voice::ListenResponse)

(defmethod sentence_voice::ListenRequest
  (:response () (instance sentence_voice::ListenResponse :init)))

(setf (get sentence_voice::ListenRequest :md5sum-) "9960a830d15d58a17858801d6d464529")
(setf (get sentence_voice::ListenRequest :datatype-) "sentence_voice/ListenRequest")
(setf (get sentence_voice::ListenRequest :definition-)
      "# 发送的请求为持续听的时间
std_msgs/Header header
int16 duration

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
---
# 返回的响应为听写的结果
std_msgs/Header header
string sentence

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

(setf (get sentence_voice::ListenResponse :md5sum-) "9960a830d15d58a17858801d6d464529")
(setf (get sentence_voice::ListenResponse :datatype-) "sentence_voice/ListenResponse")
(setf (get sentence_voice::ListenResponse :definition-)
      "# 发送的请求为持续听的时间
std_msgs/Header header
int16 duration

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
---
# 返回的响应为听写的结果
std_msgs/Header header
string sentence

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



(provide :sentence_voice/Listen "9960a830d15d58a17858801d6d464529")


