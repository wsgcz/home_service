
(cl:in-package :asdf)

(defsystem "judge_arrive-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "judge_arrive" :depends-on ("_package_judge_arrive"))
    (:file "_package_judge_arrive" :depends-on ("_package"))
  ))