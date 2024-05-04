
(cl:in-package :asdf)

(defsystem "general_service_2022-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :move_base_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Goals_name" :depends-on ("_package_Goals_name"))
    (:file "_package_Goals_name" :depends-on ("_package"))
    (:file "YoloResult" :depends-on ("_package_YoloResult"))
    (:file "_package_YoloResult" :depends-on ("_package"))
    (:file "the_way_out" :depends-on ("_package_the_way_out"))
    (:file "_package_the_way_out" :depends-on ("_package"))
  ))