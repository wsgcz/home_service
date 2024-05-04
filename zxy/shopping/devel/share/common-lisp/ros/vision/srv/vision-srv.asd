
(cl:in-package :asdf)

(defsystem "vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LeaderDist" :depends-on ("_package_LeaderDist"))
    (:file "_package_LeaderDist" :depends-on ("_package"))
  ))