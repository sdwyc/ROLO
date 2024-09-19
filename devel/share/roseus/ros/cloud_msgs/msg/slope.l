;; Auto-generated. Do not edit!


(when (boundp 'cloud_msgs::slope)
  (if (not (find-package "CLOUD_MSGS"))
    (make-package "CLOUD_MSGS"))
  (shadow 'slope (find-package "CLOUD_MSGS")))
(unless (find-package "CLOUD_MSGS::SLOPE")
  (make-package "CLOUD_MSGS::SLOPE"))

(in-package "ROS")
;;//! \htmlinclude slope.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cloud_msgs::slope
  :super ros::object
  :slots (_header _slope _slope_pos ))

(defmethod cloud_msgs::slope
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:slope __slope) (instance std_msgs::Float64 :init))
    ((:slope_pos __slope_pos) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _slope __slope)
   (setq _slope_pos __slope_pos)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:slope
   (&rest __slope)
   (if (keywordp (car __slope))
       (send* _slope __slope)
     (progn
       (if __slope (setq _slope (car __slope)))
       _slope)))
  (:slope_pos
   (&rest __slope_pos)
   (if (keywordp (car __slope_pos))
       (send* _slope_pos __slope_pos)
     (progn
       (if __slope_pos (setq _slope_pos (car __slope_pos)))
       _slope_pos)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Float64 _slope
    (send _slope :serialization-length)
    ;; geometry_msgs/Point _slope_pos
    (send _slope_pos :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Float64 _slope
       (send _slope :serialize s)
     ;; geometry_msgs/Point _slope_pos
       (send _slope_pos :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Float64 _slope
     (send _slope :deserialize buf ptr-) (incf ptr- (send _slope :serialization-length))
   ;; geometry_msgs/Point _slope_pos
     (send _slope_pos :deserialize buf ptr-) (incf ptr- (send _slope_pos :serialization-length))
   ;;
   self)
  )

(setf (get cloud_msgs::slope :md5sum-) "31b6c83548d75f9ffe937b49d92a3010")
(setf (get cloud_msgs::slope :datatype-) "cloud_msgs/slope")
(setf (get cloud_msgs::slope :definition-)
      "std_msgs/Header header
std_msgs/Float64 slope
geometry_msgs/Point slope_pos
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :cloud_msgs/slope "31b6c83548d75f9ffe937b49d92a3010")


