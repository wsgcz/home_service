
(cl:in-package :asdf)

(defsystem "sentence_voice-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Listen" :depends-on ("_package_Listen"))
    (:file "_package_Listen" :depends-on ("_package"))
    (:file "Speak" :depends-on ("_package_Speak"))
    (:file "_package_Speak" :depends-on ("_package"))
  ))