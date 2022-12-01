
(cl:in-package :asdf)

(defsystem "common_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Camera_dofs" :depends-on ("_package_Camera_dofs"))
    (:file "_package_Camera_dofs" :depends-on ("_package"))
    (:file "Cone" :depends-on ("_package_Cone"))
    (:file "_package_Cone" :depends-on ("_package"))
    (:file "Controls" :depends-on ("_package_Controls"))
    (:file "_package_Controls" :depends-on ("_package"))
    (:file "GpsPos" :depends-on ("_package_GpsPos"))
    (:file "_package_GpsPos" :depends-on ("_package"))
    (:file "Map" :depends-on ("_package_Map"))
    (:file "_package_Map" :depends-on ("_package"))
    (:file "Mission" :depends-on ("_package_Mission"))
    (:file "_package_Mission" :depends-on ("_package"))
    (:file "Trajectory" :depends-on ("_package_Trajectory"))
    (:file "_package_Trajectory" :depends-on ("_package"))
    (:file "velState" :depends-on ("_package_velState"))
    (:file "_package_velState" :depends-on ("_package"))
  ))