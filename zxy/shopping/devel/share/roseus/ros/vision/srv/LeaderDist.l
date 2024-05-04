;; Auto-generated. Do not edit!


(when (boundp 'vision::LeaderDist)
  (if (not (find-package "VISION"))
    (make-package "VISION"))
  (shadow 'LeaderDist (find-package "VISION")))
(unless (find-package "VISION::LEADERDIST")
  (make-package "VISION::LEADERDIST"))
(unless (find-package "VISION::LEADERDISTREQUEST")
  (make-package "VISION::LEADERDISTREQUEST"))
(unless (find-package "VISION::LEADERDISTRESPONSE")
  (make-package "VISION::LEADERDISTRESPONSE"))

(in-package "ROS")





(defclass vision::LeaderDistRequest
  :super ros::object
  :slots ())

(defmethod vision::LeaderDistRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass vision::LeaderDistResponse
  :super ros::object
  :slots (_distance ))

(defmethod vision::LeaderDistResponse
  (:init
   (&key
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _distance (float __distance))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float64 _distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass vision::LeaderDist
  :super ros::object
  :slots ())

(setf (get vision::LeaderDist :md5sum-) "acff7fda0c683c872875f5c4b069f124")
(setf (get vision::LeaderDist :datatype-) "vision/LeaderDist")
(setf (get vision::LeaderDist :request) vision::LeaderDistRequest)
(setf (get vision::LeaderDist :response) vision::LeaderDistResponse)

(defmethod vision::LeaderDistRequest
  (:response () (instance vision::LeaderDistResponse :init)))

(setf (get vision::LeaderDistRequest :md5sum-) "acff7fda0c683c872875f5c4b069f124")
(setf (get vision::LeaderDistRequest :datatype-) "vision/LeaderDistRequest")
(setf (get vision::LeaderDistRequest :definition-)
      "---
float64 distance
")

(setf (get vision::LeaderDistResponse :md5sum-) "acff7fda0c683c872875f5c4b069f124")
(setf (get vision::LeaderDistResponse :datatype-) "vision/LeaderDistResponse")
(setf (get vision::LeaderDistResponse :definition-)
      "---
float64 distance
")



(provide :vision/LeaderDist "acff7fda0c683c872875f5c4b069f124")


