// Auto-generated. Do not edit!

// (in-package object_list.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ObjectList = require('./ObjectList.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObjectsList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.obj_list = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('obj_list')) {
        this.obj_list = initObj.obj_list
      }
      else {
        this.obj_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectsList
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [obj_list]
    // Serialize the length for message field [obj_list]
    bufferOffset = _serializer.uint32(obj.obj_list.length, buffer, bufferOffset);
    obj.obj_list.forEach((val) => {
      bufferOffset = ObjectList.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectsList
    let len;
    let data = new ObjectsList(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [obj_list]
    // Deserialize array length for message field [obj_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obj_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obj_list[i] = ObjectList.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 424 * object.obj_list.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_list/ObjectsList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ad78ad5b64d1034693ee039c59e27240';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    ObjectList[] obj_list
    
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
    MSG: object_list/ObjectList
    int32 obj_id
    Geometric geometric
    float64[36] covariance
    Dimension dimension
    float32 prop_existence
    float32 prop_mov 
    Classification classification
    Features features
    
    ================================================================================
    MSG: object_list/Geometric
    float64 x
    float64 y
    float64 vx
    float64 vy
    float64 ax
    float64 ay
    float64 yaw
    float64 yawrate
    
    ================================================================================
    MSG: object_list/Dimension
    float64 length
    float64 width
    float64 height
    
    ================================================================================
    MSG: object_list/Classification
    float32 car
    float32 truck
    float32 motorcycle
    float32 bicycle
    float32 pedestrian
    float32 stacionary
    float32 other
    
    ================================================================================
    MSG: object_list/Features
    bool FL
    bool FM
    bool FR
    bool MR
    bool RR
    bool RM
    bool RL
    bool ML
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectsList(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.obj_list !== undefined) {
      resolved.obj_list = new Array(msg.obj_list.length);
      for (let i = 0; i < resolved.obj_list.length; ++i) {
        resolved.obj_list[i] = ObjectList.Resolve(msg.obj_list[i]);
      }
    }
    else {
      resolved.obj_list = []
    }

    return resolved;
    }
};

module.exports = ObjectsList;
