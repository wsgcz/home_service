
(cl:in-package :asdf)

(defsystem "sentence_voice-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Sentence" :depends-on ("_package_Sentence"))
    (:file "_package_Sentence" :depends-on ("_package"))
  ))