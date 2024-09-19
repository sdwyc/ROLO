; Auto-generated. Do not edit!


(cl:in-package cloud_msgs-msg)


;//! \htmlinclude slope.msg.html

(cl:defclass <slope> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (slope
    :reader slope
    :initarg :slope
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (slope_pos
    :reader slope_pos
    :initarg :slope_pos
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass slope (<slope>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <slope>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'slope)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cloud_msgs-msg:<slope> is deprecated: use cloud_msgs-msg:slope instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <slope>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:header-val is deprecated.  Use cloud_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'slope-val :lambda-list '(m))
(cl:defmethod slope-val ((m <slope>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:slope-val is deprecated.  Use cloud_msgs-msg:slope instead.")
  (slope m))

(cl:ensure-generic-function 'slope_pos-val :lambda-list '(m))
(cl:defmethod slope_pos-val ((m <slope>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cloud_msgs-msg:slope_pos-val is deprecated.  Use cloud_msgs-msg:slope_pos instead.")
  (slope_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <slope>) ostream)
  "Serializes a message object of type '<slope>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'slope) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'slope_pos) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <slope>) istream)
  "Deserializes a message object of type '<slope>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'slope) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'slope_pos) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<slope>)))
  "Returns string type for a message object of type '<slope>"
  "cloud_msgs/slope")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'slope)))
  "Returns string type for a message object of type 'slope"
  "cloud_msgs/slope")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<slope>)))
  "Returns md5sum for a message object of type '<slope>"
  "31b6c83548d75f9ffe937b49d92a3010")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'slope)))
  "Returns md5sum for a message object of type 'slope"
  "31b6c83548d75f9ffe937b49d92a3010")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<slope>)))
  "Returns full string definition for message of type '<slope>"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Float64 slope~%geometry_msgs/Point slope_pos~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'slope)))
  "Returns full string definition for message of type 'slope"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/Float64 slope~%geometry_msgs/Point slope_pos~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <slope>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'slope))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'slope_pos))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <slope>))
  "Converts a ROS message object to a list"
  (cl:list 'slope
    (cl:cons ':header (header msg))
    (cl:cons ':slope (slope msg))
    (cl:cons ':slope_pos (slope_pos msg))
))
