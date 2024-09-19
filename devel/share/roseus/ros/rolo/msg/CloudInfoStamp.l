;; Auto-generated. Do not edit!


(when (boundp 'rolo::CloudInfoStamp)
  (if (not (find-package "ROLO"))
    (make-package "ROLO"))
  (shadow 'CloudInfoStamp (find-package "ROLO")))
(unless (find-package "ROLO::CLOUDINFOSTAMP")
  (make-package "ROLO::CLOUDINFOSTAMP"))

(in-package "ROS")
;;//! \htmlinclude CloudInfoStamp.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rolo::CloudInfoStamp
  :super ros::object
  :slots (_header _startRingIndex _endRingIndex _pointColInd _pointRange _startOrientation _endOrientation _orientationDiff _initialGuessX _initialGuessY _initialGuessZ _initialGuessRoll _initialGuessPitch _initialGuessYaw _odomAvailable _cloud_projected _extracted_corner _extracted_surface _extracted_normal ))

(defmethod rolo::CloudInfoStamp
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:startRingIndex __startRingIndex) (make-array 0 :initial-element 0 :element-type :integer))
    ((:endRingIndex __endRingIndex) (make-array 0 :initial-element 0 :element-type :integer))
    ((:pointColInd __pointColInd) (make-array 0 :initial-element 0 :element-type :integer))
    ((:pointRange __pointRange) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:startOrientation __startOrientation) 0.0)
    ((:endOrientation __endOrientation) 0.0)
    ((:orientationDiff __orientationDiff) 0.0)
    ((:initialGuessX __initialGuessX) 0.0)
    ((:initialGuessY __initialGuessY) 0.0)
    ((:initialGuessZ __initialGuessZ) 0.0)
    ((:initialGuessRoll __initialGuessRoll) 0.0)
    ((:initialGuessPitch __initialGuessPitch) 0.0)
    ((:initialGuessYaw __initialGuessYaw) 0.0)
    ((:odomAvailable __odomAvailable) nil)
    ((:cloud_projected __cloud_projected) (instance sensor_msgs::PointCloud2 :init))
    ((:extracted_corner __extracted_corner) (instance sensor_msgs::PointCloud2 :init))
    ((:extracted_surface __extracted_surface) (instance sensor_msgs::PointCloud2 :init))
    ((:extracted_normal __extracted_normal) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _startRingIndex __startRingIndex)
   (setq _endRingIndex __endRingIndex)
   (setq _pointColInd __pointColInd)
   (setq _pointRange __pointRange)
   (setq _startOrientation (float __startOrientation))
   (setq _endOrientation (float __endOrientation))
   (setq _orientationDiff (float __orientationDiff))
   (setq _initialGuessX (float __initialGuessX))
   (setq _initialGuessY (float __initialGuessY))
   (setq _initialGuessZ (float __initialGuessZ))
   (setq _initialGuessRoll (float __initialGuessRoll))
   (setq _initialGuessPitch (float __initialGuessPitch))
   (setq _initialGuessYaw (float __initialGuessYaw))
   (setq _odomAvailable __odomAvailable)
   (setq _cloud_projected __cloud_projected)
   (setq _extracted_corner __extracted_corner)
   (setq _extracted_surface __extracted_surface)
   (setq _extracted_normal __extracted_normal)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:startRingIndex
   (&optional __startRingIndex)
   (if __startRingIndex (setq _startRingIndex __startRingIndex)) _startRingIndex)
  (:endRingIndex
   (&optional __endRingIndex)
   (if __endRingIndex (setq _endRingIndex __endRingIndex)) _endRingIndex)
  (:pointColInd
   (&optional __pointColInd)
   (if __pointColInd (setq _pointColInd __pointColInd)) _pointColInd)
  (:pointRange
   (&optional __pointRange)
   (if __pointRange (setq _pointRange __pointRange)) _pointRange)
  (:startOrientation
   (&optional __startOrientation)
   (if __startOrientation (setq _startOrientation __startOrientation)) _startOrientation)
  (:endOrientation
   (&optional __endOrientation)
   (if __endOrientation (setq _endOrientation __endOrientation)) _endOrientation)
  (:orientationDiff
   (&optional __orientationDiff)
   (if __orientationDiff (setq _orientationDiff __orientationDiff)) _orientationDiff)
  (:initialGuessX
   (&optional __initialGuessX)
   (if __initialGuessX (setq _initialGuessX __initialGuessX)) _initialGuessX)
  (:initialGuessY
   (&optional __initialGuessY)
   (if __initialGuessY (setq _initialGuessY __initialGuessY)) _initialGuessY)
  (:initialGuessZ
   (&optional __initialGuessZ)
   (if __initialGuessZ (setq _initialGuessZ __initialGuessZ)) _initialGuessZ)
  (:initialGuessRoll
   (&optional __initialGuessRoll)
   (if __initialGuessRoll (setq _initialGuessRoll __initialGuessRoll)) _initialGuessRoll)
  (:initialGuessPitch
   (&optional __initialGuessPitch)
   (if __initialGuessPitch (setq _initialGuessPitch __initialGuessPitch)) _initialGuessPitch)
  (:initialGuessYaw
   (&optional __initialGuessYaw)
   (if __initialGuessYaw (setq _initialGuessYaw __initialGuessYaw)) _initialGuessYaw)
  (:odomAvailable
   (&optional __odomAvailable)
   (if __odomAvailable (setq _odomAvailable __odomAvailable)) _odomAvailable)
  (:cloud_projected
   (&rest __cloud_projected)
   (if (keywordp (car __cloud_projected))
       (send* _cloud_projected __cloud_projected)
     (progn
       (if __cloud_projected (setq _cloud_projected (car __cloud_projected)))
       _cloud_projected)))
  (:extracted_corner
   (&rest __extracted_corner)
   (if (keywordp (car __extracted_corner))
       (send* _extracted_corner __extracted_corner)
     (progn
       (if __extracted_corner (setq _extracted_corner (car __extracted_corner)))
       _extracted_corner)))
  (:extracted_surface
   (&rest __extracted_surface)
   (if (keywordp (car __extracted_surface))
       (send* _extracted_surface __extracted_surface)
     (progn
       (if __extracted_surface (setq _extracted_surface (car __extracted_surface)))
       _extracted_surface)))
  (:extracted_normal
   (&rest __extracted_normal)
   (if (keywordp (car __extracted_normal))
       (send* _extracted_normal __extracted_normal)
     (progn
       (if __extracted_normal (setq _extracted_normal (car __extracted_normal)))
       _extracted_normal)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32[] _startRingIndex
    (* 4    (length _startRingIndex)) 4
    ;; int32[] _endRingIndex
    (* 4    (length _endRingIndex)) 4
    ;; int32[] _pointColInd
    (* 4    (length _pointColInd)) 4
    ;; float32[] _pointRange
    (* 4    (length _pointRange)) 4
    ;; float32 _startOrientation
    4
    ;; float32 _endOrientation
    4
    ;; float32 _orientationDiff
    4
    ;; float32 _initialGuessX
    4
    ;; float32 _initialGuessY
    4
    ;; float32 _initialGuessZ
    4
    ;; float32 _initialGuessRoll
    4
    ;; float32 _initialGuessPitch
    4
    ;; float32 _initialGuessYaw
    4
    ;; bool _odomAvailable
    1
    ;; sensor_msgs/PointCloud2 _cloud_projected
    (send _cloud_projected :serialization-length)
    ;; sensor_msgs/PointCloud2 _extracted_corner
    (send _extracted_corner :serialization-length)
    ;; sensor_msgs/PointCloud2 _extracted_surface
    (send _extracted_surface :serialization-length)
    ;; sensor_msgs/PointCloud2 _extracted_normal
    (send _extracted_normal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32[] _startRingIndex
     (write-long (length _startRingIndex) s)
     (dotimes (i (length _startRingIndex))
       (write-long (elt _startRingIndex i) s)
       )
     ;; int32[] _endRingIndex
     (write-long (length _endRingIndex) s)
     (dotimes (i (length _endRingIndex))
       (write-long (elt _endRingIndex i) s)
       )
     ;; int32[] _pointColInd
     (write-long (length _pointColInd) s)
     (dotimes (i (length _pointColInd))
       (write-long (elt _pointColInd i) s)
       )
     ;; float32[] _pointRange
     (write-long (length _pointRange) s)
     (dotimes (i (length _pointRange))
       (sys::poke (elt _pointRange i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _startOrientation
       (sys::poke _startOrientation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _endOrientation
       (sys::poke _endOrientation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientationDiff
       (sys::poke _orientationDiff (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initialGuessX
       (sys::poke _initialGuessX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initialGuessY
       (sys::poke _initialGuessY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initialGuessZ
       (sys::poke _initialGuessZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initialGuessRoll
       (sys::poke _initialGuessRoll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initialGuessPitch
       (sys::poke _initialGuessPitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _initialGuessYaw
       (sys::poke _initialGuessYaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _odomAvailable
       (if _odomAvailable (write-byte -1 s) (write-byte 0 s))
     ;; sensor_msgs/PointCloud2 _cloud_projected
       (send _cloud_projected :serialize s)
     ;; sensor_msgs/PointCloud2 _extracted_corner
       (send _extracted_corner :serialize s)
     ;; sensor_msgs/PointCloud2 _extracted_surface
       (send _extracted_surface :serialize s)
     ;; sensor_msgs/PointCloud2 _extracted_normal
       (send _extracted_normal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32[] _startRingIndex
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _startRingIndex (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _startRingIndex i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _endRingIndex
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _endRingIndex (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _endRingIndex i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _pointColInd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pointColInd (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _pointColInd i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float32[] _pointRange
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pointRange (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pointRange i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _startOrientation
     (setq _startOrientation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _endOrientation
     (setq _endOrientation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientationDiff
     (setq _orientationDiff (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initialGuessX
     (setq _initialGuessX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initialGuessY
     (setq _initialGuessY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initialGuessZ
     (setq _initialGuessZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initialGuessRoll
     (setq _initialGuessRoll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initialGuessPitch
     (setq _initialGuessPitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _initialGuessYaw
     (setq _initialGuessYaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _odomAvailable
     (setq _odomAvailable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; sensor_msgs/PointCloud2 _cloud_projected
     (send _cloud_projected :deserialize buf ptr-) (incf ptr- (send _cloud_projected :serialization-length))
   ;; sensor_msgs/PointCloud2 _extracted_corner
     (send _extracted_corner :deserialize buf ptr-) (incf ptr- (send _extracted_corner :serialization-length))
   ;; sensor_msgs/PointCloud2 _extracted_surface
     (send _extracted_surface :deserialize buf ptr-) (incf ptr- (send _extracted_surface :serialization-length))
   ;; sensor_msgs/PointCloud2 _extracted_normal
     (send _extracted_normal :deserialize buf ptr-) (incf ptr- (send _extracted_normal :serialization-length))
   ;;
   self)
  )

(setf (get rolo::CloudInfoStamp :md5sum-) "95edffafd888edf477d4a6245ab452a2")
(setf (get rolo::CloudInfoStamp :datatype-) "rolo/CloudInfoStamp")
(setf (get rolo::CloudInfoStamp :definition-)
      "Header header 

int32[] startRingIndex
int32[] endRingIndex

int32[]  pointColInd # point column index in range image
float32[] pointRange # point range

float32 startOrientation
float32 endOrientation
float32 orientationDiff

# Initial guess from front lidar odometry
float32 initialGuessX
float32 initialGuessY
float32 initialGuessZ
float32 initialGuessRoll
float32 initialGuessPitch
float32 initialGuessYaw
bool odomAvailable

# Point cloud messages
sensor_msgs/PointCloud2 cloud_projected  # original cloud
sensor_msgs/PointCloud2 extracted_corner    # extracted corner feature
sensor_msgs/PointCloud2 extracted_surface   # extracted surface feature
sensor_msgs/PointCloud2 extracted_normal   # extracted normal point(for back-end)
# sensor_msgs/PointCloud2 extracted_ground   # extracted ground


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
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

")



(provide :rolo/CloudInfoStamp "95edffafd888edf477d4a6245ab452a2")


