;; Auto-generated. Do not edit!


(when (boundp 'general_service_2022::the_way_out)
  (if (not (find-package "GENERAL_SERVICE_2022"))
    (make-package "GENERAL_SERVICE_2022"))
  (shadow 'the_way_out (find-package "GENERAL_SERVICE_2022")))
(unless (find-package "GENERAL_SERVICE_2022::THE_WAY_OUT")
  (make-package "GENERAL_SERVICE_2022::THE_WAY_OUT"))

(in-package "ROS")
;;//! \htmlinclude the_way_out.msg.html


(defclass general_service_2022::the_way_out
  :super ros::object
  :slots (_angle _max_length _the_angle_of_max_length ))

(defmethod general_service_2022::the_way_out
  (:init
   (&key
    ((:angle __angle) 0)
    ((:max_length __max_length) 0.0)
    ((:the_angle_of_max_length __the_angle_of_max_length) 0)
    )
   (send-super :init)
   (setq _angle (round __angle))
   (setq _max_length (float __max_length))
   (setq _the_angle_of_max_length (round __the_angle_of_max_length))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:max_length
   (&optional __max_length)
   (if __max_length (setq _max_length __max_length)) _max_length)
  (:the_angle_of_max_length
   (&optional __the_angle_of_max_length)
   (if __the_angle_of_max_length (setq _the_angle_of_max_length __the_angle_of_max_length)) _the_angle_of_max_length)
  (:serialization-length
   ()
   (+
    ;; int32 _angle
    4
    ;; float64 _max_length
    8
    ;; int32 _the_angle_of_max_length
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _angle
       (write-long _angle s)
     ;; float64 _max_length
       (sys::poke _max_length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _the_angle_of_max_length
       (write-long _the_angle_of_max_length s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _angle
     (setq _angle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _max_length
     (setq _max_length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _the_angle_of_max_length
     (setq _the_angle_of_max_length (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get general_service_2022::the_way_out :md5sum-) "be349f47f00195efdd7bf46fc8b5963c")
(setf (get general_service_2022::the_way_out :datatype-) "general_service_2022/the_way_out")
(setf (get general_service_2022::the_way_out :definition-)
      "int32 angle
float64 max_length
int32 the_angle_of_max_length
")



(provide :general_service_2022/the_way_out "be349f47f00195efdd7bf46fc8b5963c")


