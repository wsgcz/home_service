;; Auto-generated. Do not edit!


(when (boundp 'sentence_voice::Sentence)
  (if (not (find-package "SENTENCE_VOICE"))
    (make-package "SENTENCE_VOICE"))
  (shadow 'Sentence (find-package "SENTENCE_VOICE")))
(unless (find-package "SENTENCE_VOICE::SENTENCE")
  (make-package "SENTENCE_VOICE::SENTENCE"))

(in-package "ROS")
;;//! \htmlinclude Sentence.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sentence_voice::Sentence
  :super ros::object
  :slots (_header _sentence ))

(defmethod sentence_voice::Sentence
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

(setf (get sentence_voice::Sentence :md5sum-) "9f221efc5f4b3bac7ce4af102b32308b")
(setf (get sentence_voice::Sentence :datatype-) "sentence_voice/Sentence")
(setf (get sentence_voice::Sentence :definition-)
      "std_msgs/Header header
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



(provide :sentence_voice/Sentence "9f221efc5f4b3bac7ce4af102b32308b")


