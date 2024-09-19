
(cl:in-package :asdf)

(defsystem "cloud_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "cloud_info" :depends-on ("_package_cloud_info"))
    (:file "_package_cloud_info" :depends-on ("_package"))
    (:file "slope" :depends-on ("_package_slope"))
    (:file "_package_slope" :depends-on ("_package"))
  ))