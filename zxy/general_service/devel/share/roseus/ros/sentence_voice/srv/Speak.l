;; Auto-generated. Do not edit!


(when (boundp 'sentence_voice::Speak)
  (if (not (find-package "SENTENCE_VOICE"))
    (make-package "SENTENCE_VOICE"))
  (shadow 'Speak (find-package "SENTENCE_VOICE")))
(unless (find-package "SENTENCE_VOICE::SPEAK")
  (make-package "SENTENCE_VOICE::SPEAK"))
(unless (find-package "SENTENCE_VOICE::SPEAKREQUEST")
  (make-package "SENTENCE_VOICE::SPEAKREQUEST"))
(unless (find-package "SENTENCE_VOICE::SPEAKRESPONSE")
  (make-package "SENTENCE_VOICE::SPEAKRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sentence_voice::SpeakRequest
  :super ros::object
  :slots (_header _sentence ))

(defmethod sentence_voice::SpeakRequest
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

(defclass sentence_voice::SpeakResponse
  :super ros::object
  :slots (_header _duration ))

(defmethod sentence_voice::SpeakResponse
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

(defclass sentence_voice::Speak
  :super ros::object
  :slots ())

(setf (get sentence_voice::Speak :md5sum-) "6ec12d5a2bfa19afaef4554a69aed1b2")
(setf (get sentence_voice::Speak :datatype-) "sentence_voice/Speak")
(setf (get sentence_voice::Speak :request) sentence_voice::SpeakRequest)
(setf (get sentence_voice::Speak :response) sentence_voice::SpeakResponse)

(defmethod sentence_voice::SpeakRequest
  (:response () (instance sentence_voice::SpeakResponse :init)))

(setf (get sentence_voice::SpeakRequest :md5sum-) "6ec12d5a2bfa19afaef4554a69aed1b2")
(setf (get sentence_voice::SpeakRequest :datatype-) "sentence_voice/SpeakRequest")
(setf (get sentence_voice::SpeakRequest :definition-)
      "# 发送的请求是要说的话
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
---
# 返回的响应是说话的时间，-1表示说话失败
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
")

(setf (get sentence_voice::SpeakResponse :md5sum-) "6ec12d5a2bfa19afaef4554a69aed1b2")
(setf (get sentence_voice::SpeakResponse :datatype-) "sentence_voice/SpeakResponse")
(setf (get sentence_voice::SpeakResponse :definition-)
      "# 发送的请求是要说的话
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
---
# 返回的响应是说话的时间，-1表示说话失败
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
")



(provide :sentence_voice/Speak "6ec12d5a2bfa19afaef4554a69aed1b2")


