
(cl:in-package :asdf)

(defsystem "common_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Spawn" :depends-on ("_package_Spawn"))
    (:file "_package_Spawn" :depends-on ("_package"))
  ))