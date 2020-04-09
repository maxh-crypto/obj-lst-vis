// Auto-generated. Do not edit!

// (in-package object_list.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Features {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.FL = null;
      this.FM = null;
      this.FR = null;
      this.MR = null;
      this.RR = null;
      this.RM = null;
      this.RL = null;
      this.ML = null;
    }
    else {
      if (initObj.hasOwnProperty('FL')) {
        this.FL = initObj.FL
      }
      else {
        this.FL = 0;
      }
      if (initObj.hasOwnProperty('FM')) {
        this.FM = initObj.FM
      }
      else {
        this.FM = 0;
      }
      if (initObj.hasOwnProperty('FR')) {
        this.FR = initObj.FR
      }
      else {
        this.FR = 0;
      }
      if (initObj.hasOwnProperty('MR')) {
        this.MR = initObj.MR
      }
      else {
        this.MR = 0;
      }
      if (initObj.hasOwnProperty('RR')) {
        this.RR = initObj.RR
      }
      else {
        this.RR = 0;
      }
      if (initObj.hasOwnProperty('RM')) {
        this.RM = initObj.RM
      }
      else {
        this.RM = 0;
      }
      if (initObj.hasOwnProperty('RL')) {
        this.RL = initObj.RL
      }
      else {
        this.RL = 0;
      }
      if (initObj.hasOwnProperty('ML')) {
        this.ML = initObj.ML
      }
      else {
        this.ML = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Features
    // Serialize message field [FL]
    bufferOffset = _serializer.uint8(obj.FL, buffer, bufferOffset);
    // Serialize message field [FM]
    bufferOffset = _serializer.uint8(obj.FM, buffer, bufferOffset);
    // Serialize message field [FR]
    bufferOffset = _serializer.uint8(obj.FR, buffer, bufferOffset);
    // Serialize message field [MR]
    bufferOffset = _serializer.uint8(obj.MR, buffer, bufferOffset);
    // Serialize message field [RR]
    bufferOffset = _serializer.uint8(obj.RR, buffer, bufferOffset);
    // Serialize message field [RM]
    bufferOffset = _serializer.uint8(obj.RM, buffer, bufferOffset);
    // Serialize message field [RL]
    bufferOffset = _serializer.uint8(obj.RL, buffer, bufferOffset);
    // Serialize message field [ML]
    bufferOffset = _serializer.uint8(obj.ML, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Features
    let len;
    let data = new Features(null);
    // Deserialize message field [FL]
    data.FL = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [FM]
    data.FM = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [FR]
    data.FR = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [MR]
    data.MR = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [RR]
    data.RR = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [RM]
    data.RM = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [RL]
    data.RL = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ML]
    data.ML = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_list/Features';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'acfb5ca82687e271a6722833317ebf1a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 FL
    uint8 FM
    uint8 FR
    uint8 MR
    uint8 RR
    uint8 RM
    uint8 RL
    uint8 ML
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Features(null);
    if (msg.FL !== undefined) {
      resolved.FL = msg.FL;
    }
    else {
      resolved.FL = 0
    }

    if (msg.FM !== undefined) {
      resolved.FM = msg.FM;
    }
    else {
      resolved.FM = 0
    }

    if (msg.FR !== undefined) {
      resolved.FR = msg.FR;
    }
    else {
      resolved.FR = 0
    }

    if (msg.MR !== undefined) {
      resolved.MR = msg.MR;
    }
    else {
      resolved.MR = 0
    }

    if (msg.RR !== undefined) {
      resolved.RR = msg.RR;
    }
    else {
      resolved.RR = 0
    }

    if (msg.RM !== undefined) {
      resolved.RM = msg.RM;
    }
    else {
      resolved.RM = 0
    }

    if (msg.RL !== undefined) {
      resolved.RL = msg.RL;
    }
    else {
      resolved.RL = 0
    }

    if (msg.ML !== undefined) {
      resolved.ML = msg.ML;
    }
    else {
      resolved.ML = 0
    }

    return resolved;
    }
};

module.exports = Features;
