
(cl:in-package :asdf)

(defsystem "nubot_common-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BallHandle" :depends-on ("_package_BallHandle"))
    (:file "_package_BallHandle" :depends-on ("_package"))
    (:file "GetModelState" :depends-on ("_package_GetModelState"))
    (:file "_package_GetModelState" :depends-on ("_package"))
    (:file "Shoot" :depends-on ("_package_Shoot"))
    (:file "_package_Shoot" :depends-on ("_package"))
  ))