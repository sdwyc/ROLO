
(cl:in-package :asdf)

(defsystem "rolo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CloudInfoStamp" :depends-on ("_package_CloudInfoStamp"))
    (:file "_package_CloudInfoStamp" :depends-on ("_package"))
  ))