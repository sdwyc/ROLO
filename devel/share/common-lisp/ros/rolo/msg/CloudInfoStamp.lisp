; Auto-generated. Do not edit!


(cl:in-package rolo-msg)


;//! \htmlinclude CloudInfoStamp.msg.html

(cl:defclass <CloudInfoStamp> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (startRingIndex
    :reader startRingIndex
    :initarg :startRingIndex
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (endRingIndex
    :reader endRingIndex
    :initarg :endRingIndex
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (pointColInd
    :reader pointColInd
    :initarg :pointColInd
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (pointRange
    :reader pointRange
    :initarg :pointRange
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (startOrientation
    :reader startOrientation
    :initarg :startOrientation
    :type cl:float
    :initform 0.0)
   (endOrientation
    :reader endOrientation
    :initarg :endOrientation
    :type cl:float
    :initform 0.0)
   (orientationDiff
    :reader orientationDiff
    :initarg :orientationDiff
    :type cl:float
    :initform 0.0)
   (initialGuessX
    :reader initialGuessX
    :initarg :initialGuessX
    :type cl:float
    :initform 0.0)
   (initialGuessY
    :reader initialGuessY
    :initarg :initialGuessY
    :type cl:float
    :initform 0.0)
   (initialGuessZ
    :reader initialGuessZ
    :initarg :initialGuessZ
    :type cl:float
    :initform 0.0)
   (initialGuessRoll
    :reader initialGuessRoll
    :initarg :initialGuessRoll
    :type cl:float
    :initform 0.0)
   (initialGuessPitch
    :reader initialGuessPitch
    :initarg :initialGuessPitch
    :type cl:float
    :initform 0.0)
   (initialGuessYaw
    :reader initialGuessYaw
    :initarg :initialGuessYaw
    :type cl:float
    :initform 0.0)
   (odomAvailable
    :reader odomAvailable
    :initarg :odomAvailable
    :type cl:boolean
    :initform cl:nil)
   (cloud_projected
    :reader cloud_projected
    :initarg :cloud_projected
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (extracted_corner
    :reader extracted_corner
    :initarg :extracted_corner
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (extracted_surface
    :reader extracted_surface
    :initarg :extracted_surface
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2))
   (extracted_normal
    :reader extracted_normal
    :initarg :extracted_normal
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass CloudInfoStamp (<CloudInfoStamp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CloudInfoStamp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CloudInfoStamp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rolo-msg:<CloudInfoStamp> is deprecated: use rolo-msg:CloudInfoStamp instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:header-val is deprecated.  Use rolo-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'startRingIndex-val :lambda-list '(m))
(cl:defmethod startRingIndex-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:startRingIndex-val is deprecated.  Use rolo-msg:startRingIndex instead.")
  (startRingIndex m))

(cl:ensure-generic-function 'endRingIndex-val :lambda-list '(m))
(cl:defmethod endRingIndex-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:endRingIndex-val is deprecated.  Use rolo-msg:endRingIndex instead.")
  (endRingIndex m))

(cl:ensure-generic-function 'pointColInd-val :lambda-list '(m))
(cl:defmethod pointColInd-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:pointColInd-val is deprecated.  Use rolo-msg:pointColInd instead.")
  (pointColInd m))

(cl:ensure-generic-function 'pointRange-val :lambda-list '(m))
(cl:defmethod pointRange-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:pointRange-val is deprecated.  Use rolo-msg:pointRange instead.")
  (pointRange m))

(cl:ensure-generic-function 'startOrientation-val :lambda-list '(m))
(cl:defmethod startOrientation-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:startOrientation-val is deprecated.  Use rolo-msg:startOrientation instead.")
  (startOrientation m))

(cl:ensure-generic-function 'endOrientation-val :lambda-list '(m))
(cl:defmethod endOrientation-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:endOrientation-val is deprecated.  Use rolo-msg:endOrientation instead.")
  (endOrientation m))

(cl:ensure-generic-function 'orientationDiff-val :lambda-list '(m))
(cl:defmethod orientationDiff-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:orientationDiff-val is deprecated.  Use rolo-msg:orientationDiff instead.")
  (orientationDiff m))

(cl:ensure-generic-function 'initialGuessX-val :lambda-list '(m))
(cl:defmethod initialGuessX-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:initialGuessX-val is deprecated.  Use rolo-msg:initialGuessX instead.")
  (initialGuessX m))

(cl:ensure-generic-function 'initialGuessY-val :lambda-list '(m))
(cl:defmethod initialGuessY-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:initialGuessY-val is deprecated.  Use rolo-msg:initialGuessY instead.")
  (initialGuessY m))

(cl:ensure-generic-function 'initialGuessZ-val :lambda-list '(m))
(cl:defmethod initialGuessZ-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:initialGuessZ-val is deprecated.  Use rolo-msg:initialGuessZ instead.")
  (initialGuessZ m))

(cl:ensure-generic-function 'initialGuessRoll-val :lambda-list '(m))
(cl:defmethod initialGuessRoll-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:initialGuessRoll-val is deprecated.  Use rolo-msg:initialGuessRoll instead.")
  (initialGuessRoll m))

(cl:ensure-generic-function 'initialGuessPitch-val :lambda-list '(m))
(cl:defmethod initialGuessPitch-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:initialGuessPitch-val is deprecated.  Use rolo-msg:initialGuessPitch instead.")
  (initialGuessPitch m))

(cl:ensure-generic-function 'initialGuessYaw-val :lambda-list '(m))
(cl:defmethod initialGuessYaw-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:initialGuessYaw-val is deprecated.  Use rolo-msg:initialGuessYaw instead.")
  (initialGuessYaw m))

(cl:ensure-generic-function 'odomAvailable-val :lambda-list '(m))
(cl:defmethod odomAvailable-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:odomAvailable-val is deprecated.  Use rolo-msg:odomAvailable instead.")
  (odomAvailable m))

(cl:ensure-generic-function 'cloud_projected-val :lambda-list '(m))
(cl:defmethod cloud_projected-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:cloud_projected-val is deprecated.  Use rolo-msg:cloud_projected instead.")
  (cloud_projected m))

(cl:ensure-generic-function 'extracted_corner-val :lambda-list '(m))
(cl:defmethod extracted_corner-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:extracted_corner-val is deprecated.  Use rolo-msg:extracted_corner instead.")
  (extracted_corner m))

(cl:ensure-generic-function 'extracted_surface-val :lambda-list '(m))
(cl:defmethod extracted_surface-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:extracted_surface-val is deprecated.  Use rolo-msg:extracted_surface instead.")
  (extracted_surface m))

(cl:ensure-generic-function 'extracted_normal-val :lambda-list '(m))
(cl:defmethod extracted_normal-val ((m <CloudInfoStamp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rolo-msg:extracted_normal-val is deprecated.  Use rolo-msg:extracted_normal instead.")
  (extracted_normal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CloudInfoStamp>) ostream)
  "Serializes a message object of type '<CloudInfoStamp>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'startRingIndex))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'startRingIndex))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'endRingIndex))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'endRingIndex))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pointColInd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'pointColInd))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pointRange))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pointRange))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'startOrientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'endOrientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientationDiff))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initialGuessX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initialGuessY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initialGuessZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initialGuessRoll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initialGuessPitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'initialGuessYaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'odomAvailable) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud_projected) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'extracted_corner) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'extracted_surface) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'extracted_normal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CloudInfoStamp>) istream)
  "Deserializes a message object of type '<CloudInfoStamp>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'startRingIndex) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'startRingIndex)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'endRingIndex) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'endRingIndex)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pointColInd) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pointColInd)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pointRange) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pointRange)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'startOrientation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'endOrientation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientationDiff) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initialGuessX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initialGuessY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initialGuessZ) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initialGuessRoll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initialGuessPitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'initialGuessYaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'odomAvailable) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud_projected) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'extracted_corner) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'extracted_surface) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'extracted_normal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CloudInfoStamp>)))
  "Returns string type for a message object of type '<CloudInfoStamp>"
  "rolo/CloudInfoStamp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CloudInfoStamp)))
  "Returns string type for a message object of type 'CloudInfoStamp"
  "rolo/CloudInfoStamp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CloudInfoStamp>)))
  "Returns md5sum for a message object of type '<CloudInfoStamp>"
  "95edffafd888edf477d4a6245ab452a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CloudInfoStamp)))
  "Returns md5sum for a message object of type 'CloudInfoStamp"
  "95edffafd888edf477d4a6245ab452a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CloudInfoStamp>)))
  "Returns full string definition for message of type '<CloudInfoStamp>"
  (cl:format cl:nil "Header header ~%~%int32[] startRingIndex~%int32[] endRingIndex~%~%int32[]  pointColInd # point column index in range image~%float32[] pointRange # point range~%~%float32 startOrientation~%float32 endOrientation~%float32 orientationDiff~%~%# Initial guess from front lidar odometry~%float32 initialGuessX~%float32 initialGuessY~%float32 initialGuessZ~%float32 initialGuessRoll~%float32 initialGuessPitch~%float32 initialGuessYaw~%bool odomAvailable~%~%# Point cloud messages~%sensor_msgs/PointCloud2 cloud_projected  # original cloud~%sensor_msgs/PointCloud2 extracted_corner    # extracted corner feature~%sensor_msgs/PointCloud2 extracted_surface   # extracted surface feature~%sensor_msgs/PointCloud2 extracted_normal   # extracted normal point(for back-end)~%# sensor_msgs/PointCloud2 extracted_ground   # extracted ground~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CloudInfoStamp)))
  "Returns full string definition for message of type 'CloudInfoStamp"
  (cl:format cl:nil "Header header ~%~%int32[] startRingIndex~%int32[] endRingIndex~%~%int32[]  pointColInd # point column index in range image~%float32[] pointRange # point range~%~%float32 startOrientation~%float32 endOrientation~%float32 orientationDiff~%~%# Initial guess from front lidar odometry~%float32 initialGuessX~%float32 initialGuessY~%float32 initialGuessZ~%float32 initialGuessRoll~%float32 initialGuessPitch~%float32 initialGuessYaw~%bool odomAvailable~%~%# Point cloud messages~%sensor_msgs/PointCloud2 cloud_projected  # original cloud~%sensor_msgs/PointCloud2 extracted_corner    # extracted corner feature~%sensor_msgs/PointCloud2 extracted_surface   # extracted surface feature~%sensor_msgs/PointCloud2 extracted_normal   # extracted normal point(for back-end)~%# sensor_msgs/PointCloud2 extracted_ground   # extracted ground~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CloudInfoStamp>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'startRingIndex) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'endRingIndex) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pointColInd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pointRange) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud_projected))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'extracted_corner))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'extracted_surface))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'extracted_normal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CloudInfoStamp>))
  "Converts a ROS message object to a list"
  (cl:list 'CloudInfoStamp
    (cl:cons ':header (header msg))
    (cl:cons ':startRingIndex (startRingIndex msg))
    (cl:cons ':endRingIndex (endRingIndex msg))
    (cl:cons ':pointColInd (pointColInd msg))
    (cl:cons ':pointRange (pointRange msg))
    (cl:cons ':startOrientation (startOrientation msg))
    (cl:cons ':endOrientation (endOrientation msg))
    (cl:cons ':orientationDiff (orientationDiff msg))
    (cl:cons ':initialGuessX (initialGuessX msg))
    (cl:cons ':initialGuessY (initialGuessY msg))
    (cl:cons ':initialGuessZ (initialGuessZ msg))
    (cl:cons ':initialGuessRoll (initialGuessRoll msg))
    (cl:cons ':initialGuessPitch (initialGuessPitch msg))
    (cl:cons ':initialGuessYaw (initialGuessYaw msg))
    (cl:cons ':odomAvailable (odomAvailable msg))
    (cl:cons ':cloud_projected (cloud_projected msg))
    (cl:cons ':extracted_corner (extracted_corner msg))
    (cl:cons ':extracted_surface (extracted_surface msg))
    (cl:cons ':extracted_normal (extracted_normal msg))
))
