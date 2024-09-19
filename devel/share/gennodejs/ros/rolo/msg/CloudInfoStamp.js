// Auto-generated. Do not edit!

// (in-package rolo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CloudInfoStamp {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.startRingIndex = null;
      this.endRingIndex = null;
      this.pointColInd = null;
      this.pointRange = null;
      this.startOrientation = null;
      this.endOrientation = null;
      this.orientationDiff = null;
      this.initialGuessX = null;
      this.initialGuessY = null;
      this.initialGuessZ = null;
      this.initialGuessRoll = null;
      this.initialGuessPitch = null;
      this.initialGuessYaw = null;
      this.odomAvailable = null;
      this.cloud_projected = null;
      this.extracted_corner = null;
      this.extracted_surface = null;
      this.extracted_normal = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('startRingIndex')) {
        this.startRingIndex = initObj.startRingIndex
      }
      else {
        this.startRingIndex = [];
      }
      if (initObj.hasOwnProperty('endRingIndex')) {
        this.endRingIndex = initObj.endRingIndex
      }
      else {
        this.endRingIndex = [];
      }
      if (initObj.hasOwnProperty('pointColInd')) {
        this.pointColInd = initObj.pointColInd
      }
      else {
        this.pointColInd = [];
      }
      if (initObj.hasOwnProperty('pointRange')) {
        this.pointRange = initObj.pointRange
      }
      else {
        this.pointRange = [];
      }
      if (initObj.hasOwnProperty('startOrientation')) {
        this.startOrientation = initObj.startOrientation
      }
      else {
        this.startOrientation = 0.0;
      }
      if (initObj.hasOwnProperty('endOrientation')) {
        this.endOrientation = initObj.endOrientation
      }
      else {
        this.endOrientation = 0.0;
      }
      if (initObj.hasOwnProperty('orientationDiff')) {
        this.orientationDiff = initObj.orientationDiff
      }
      else {
        this.orientationDiff = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessX')) {
        this.initialGuessX = initObj.initialGuessX
      }
      else {
        this.initialGuessX = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessY')) {
        this.initialGuessY = initObj.initialGuessY
      }
      else {
        this.initialGuessY = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessZ')) {
        this.initialGuessZ = initObj.initialGuessZ
      }
      else {
        this.initialGuessZ = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessRoll')) {
        this.initialGuessRoll = initObj.initialGuessRoll
      }
      else {
        this.initialGuessRoll = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessPitch')) {
        this.initialGuessPitch = initObj.initialGuessPitch
      }
      else {
        this.initialGuessPitch = 0.0;
      }
      if (initObj.hasOwnProperty('initialGuessYaw')) {
        this.initialGuessYaw = initObj.initialGuessYaw
      }
      else {
        this.initialGuessYaw = 0.0;
      }
      if (initObj.hasOwnProperty('odomAvailable')) {
        this.odomAvailable = initObj.odomAvailable
      }
      else {
        this.odomAvailable = false;
      }
      if (initObj.hasOwnProperty('cloud_projected')) {
        this.cloud_projected = initObj.cloud_projected
      }
      else {
        this.cloud_projected = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('extracted_corner')) {
        this.extracted_corner = initObj.extracted_corner
      }
      else {
        this.extracted_corner = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('extracted_surface')) {
        this.extracted_surface = initObj.extracted_surface
      }
      else {
        this.extracted_surface = new sensor_msgs.msg.PointCloud2();
      }
      if (initObj.hasOwnProperty('extracted_normal')) {
        this.extracted_normal = initObj.extracted_normal
      }
      else {
        this.extracted_normal = new sensor_msgs.msg.PointCloud2();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CloudInfoStamp
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [startRingIndex]
    bufferOffset = _arraySerializer.int32(obj.startRingIndex, buffer, bufferOffset, null);
    // Serialize message field [endRingIndex]
    bufferOffset = _arraySerializer.int32(obj.endRingIndex, buffer, bufferOffset, null);
    // Serialize message field [pointColInd]
    bufferOffset = _arraySerializer.int32(obj.pointColInd, buffer, bufferOffset, null);
    // Serialize message field [pointRange]
    bufferOffset = _arraySerializer.float32(obj.pointRange, buffer, bufferOffset, null);
    // Serialize message field [startOrientation]
    bufferOffset = _serializer.float32(obj.startOrientation, buffer, bufferOffset);
    // Serialize message field [endOrientation]
    bufferOffset = _serializer.float32(obj.endOrientation, buffer, bufferOffset);
    // Serialize message field [orientationDiff]
    bufferOffset = _serializer.float32(obj.orientationDiff, buffer, bufferOffset);
    // Serialize message field [initialGuessX]
    bufferOffset = _serializer.float32(obj.initialGuessX, buffer, bufferOffset);
    // Serialize message field [initialGuessY]
    bufferOffset = _serializer.float32(obj.initialGuessY, buffer, bufferOffset);
    // Serialize message field [initialGuessZ]
    bufferOffset = _serializer.float32(obj.initialGuessZ, buffer, bufferOffset);
    // Serialize message field [initialGuessRoll]
    bufferOffset = _serializer.float32(obj.initialGuessRoll, buffer, bufferOffset);
    // Serialize message field [initialGuessPitch]
    bufferOffset = _serializer.float32(obj.initialGuessPitch, buffer, bufferOffset);
    // Serialize message field [initialGuessYaw]
    bufferOffset = _serializer.float32(obj.initialGuessYaw, buffer, bufferOffset);
    // Serialize message field [odomAvailable]
    bufferOffset = _serializer.bool(obj.odomAvailable, buffer, bufferOffset);
    // Serialize message field [cloud_projected]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.cloud_projected, buffer, bufferOffset);
    // Serialize message field [extracted_corner]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.extracted_corner, buffer, bufferOffset);
    // Serialize message field [extracted_surface]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.extracted_surface, buffer, bufferOffset);
    // Serialize message field [extracted_normal]
    bufferOffset = sensor_msgs.msg.PointCloud2.serialize(obj.extracted_normal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CloudInfoStamp
    let len;
    let data = new CloudInfoStamp(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [startRingIndex]
    data.startRingIndex = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [endRingIndex]
    data.endRingIndex = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [pointColInd]
    data.pointColInd = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [pointRange]
    data.pointRange = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [startOrientation]
    data.startOrientation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [endOrientation]
    data.endOrientation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [orientationDiff]
    data.orientationDiff = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessX]
    data.initialGuessX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessY]
    data.initialGuessY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessZ]
    data.initialGuessZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessRoll]
    data.initialGuessRoll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessPitch]
    data.initialGuessPitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [initialGuessYaw]
    data.initialGuessYaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [odomAvailable]
    data.odomAvailable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cloud_projected]
    data.cloud_projected = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [extracted_corner]
    data.extracted_corner = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [extracted_surface]
    data.extracted_surface = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    // Deserialize message field [extracted_normal]
    data.extracted_normal = sensor_msgs.msg.PointCloud2.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.startRingIndex.length;
    length += 4 * object.endRingIndex.length;
    length += 4 * object.pointColInd.length;
    length += 4 * object.pointRange.length;
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.cloud_projected);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.extracted_corner);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.extracted_surface);
    length += sensor_msgs.msg.PointCloud2.getMessageSize(object.extracted_normal);
    return length + 53;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rolo/CloudInfoStamp';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95edffafd888edf477d4a6245ab452a2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header 
    
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
    # contents of the "fields" array.
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CloudInfoStamp(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.startRingIndex !== undefined) {
      resolved.startRingIndex = msg.startRingIndex;
    }
    else {
      resolved.startRingIndex = []
    }

    if (msg.endRingIndex !== undefined) {
      resolved.endRingIndex = msg.endRingIndex;
    }
    else {
      resolved.endRingIndex = []
    }

    if (msg.pointColInd !== undefined) {
      resolved.pointColInd = msg.pointColInd;
    }
    else {
      resolved.pointColInd = []
    }

    if (msg.pointRange !== undefined) {
      resolved.pointRange = msg.pointRange;
    }
    else {
      resolved.pointRange = []
    }

    if (msg.startOrientation !== undefined) {
      resolved.startOrientation = msg.startOrientation;
    }
    else {
      resolved.startOrientation = 0.0
    }

    if (msg.endOrientation !== undefined) {
      resolved.endOrientation = msg.endOrientation;
    }
    else {
      resolved.endOrientation = 0.0
    }

    if (msg.orientationDiff !== undefined) {
      resolved.orientationDiff = msg.orientationDiff;
    }
    else {
      resolved.orientationDiff = 0.0
    }

    if (msg.initialGuessX !== undefined) {
      resolved.initialGuessX = msg.initialGuessX;
    }
    else {
      resolved.initialGuessX = 0.0
    }

    if (msg.initialGuessY !== undefined) {
      resolved.initialGuessY = msg.initialGuessY;
    }
    else {
      resolved.initialGuessY = 0.0
    }

    if (msg.initialGuessZ !== undefined) {
      resolved.initialGuessZ = msg.initialGuessZ;
    }
    else {
      resolved.initialGuessZ = 0.0
    }

    if (msg.initialGuessRoll !== undefined) {
      resolved.initialGuessRoll = msg.initialGuessRoll;
    }
    else {
      resolved.initialGuessRoll = 0.0
    }

    if (msg.initialGuessPitch !== undefined) {
      resolved.initialGuessPitch = msg.initialGuessPitch;
    }
    else {
      resolved.initialGuessPitch = 0.0
    }

    if (msg.initialGuessYaw !== undefined) {
      resolved.initialGuessYaw = msg.initialGuessYaw;
    }
    else {
      resolved.initialGuessYaw = 0.0
    }

    if (msg.odomAvailable !== undefined) {
      resolved.odomAvailable = msg.odomAvailable;
    }
    else {
      resolved.odomAvailable = false
    }

    if (msg.cloud_projected !== undefined) {
      resolved.cloud_projected = sensor_msgs.msg.PointCloud2.Resolve(msg.cloud_projected)
    }
    else {
      resolved.cloud_projected = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.extracted_corner !== undefined) {
      resolved.extracted_corner = sensor_msgs.msg.PointCloud2.Resolve(msg.extracted_corner)
    }
    else {
      resolved.extracted_corner = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.extracted_surface !== undefined) {
      resolved.extracted_surface = sensor_msgs.msg.PointCloud2.Resolve(msg.extracted_surface)
    }
    else {
      resolved.extracted_surface = new sensor_msgs.msg.PointCloud2()
    }

    if (msg.extracted_normal !== undefined) {
      resolved.extracted_normal = sensor_msgs.msg.PointCloud2.Resolve(msg.extracted_normal)
    }
    else {
      resolved.extracted_normal = new sensor_msgs.msg.PointCloud2()
    }

    return resolved;
    }
};

module.exports = CloudInfoStamp;
