
(cl:in-package :asdf)

(defsystem "nubot_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "VelCmd" :depends-on ("_package_VelCmd"))
    (:file "_package_VelCmd" :depends-on ("_package"))
  ))