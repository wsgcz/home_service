;; Auto-generated. Do not edit!


(when (boundp 'judge_arrive::judge_arrive)
  (if (not (find-package "JUDGE_ARRIVE"))
    (make-package "JUDGE_ARRIVE"))
  (shadow 'judge_arrive (find-package "JUDGE_ARRIVE")))
(unless (find-package "JUDGE_ARRIVE::JUDGE_ARRIVE")
  (make-package "JUDGE_ARRIVE::JUDGE_ARRIVE"))

(in-package "ROS")
;;//! \htmlinclude judge_arrive.msg.html


(defclass judge_arrive::judge_arrive
  :super ros::object
  :slots (_x _y ))

(defmethod judge_arrive::judge_arrive
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get judge_arrive::judge_arrive :md5sum-) "209f516d3eb691f0663e25cb750d67c1")
(setf (get judge_arrive::judge_arrive :datatype-) "judge_arrive/judge_arrive")
(setf (get judge_arrive::judge_arrive :definition-)
      "float64 x
float64 y
")



(provide :judge_arrive/judge_arrive "209f516d3eb691f0663e25cb750d67c1")


