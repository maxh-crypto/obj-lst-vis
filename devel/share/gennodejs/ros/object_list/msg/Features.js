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
        this.FL = false;
      }
      if (initObj.hasOwnProperty('FM')) {
        this.FM = initObj.FM
      }
      else {
        this.FM = false;
      }
      if (initObj.hasOwnProperty('FR')) {
        this.FR = initObj.FR
      }
      else {
        this.FR = false;
      }
      if (initObj.hasOwnProperty('MR')) {
        this.MR = initObj.MR
      }
      else {
        this.MR = false;
      }
      if (initObj.hasOwnProperty('RR')) {
        this.RR = initObj.RR
      }
      else {
        this.RR = false;
      }
      if (initObj.hasOwnProperty('RM')) {
        this.RM = initObj.RM
      }
      else {
        this.RM = false;
      }
      if (initObj.hasOwnProperty('RL')) {
        this.RL = initObj.RL
      }
      else {
        this.RL = false;
      }
      if (initObj.hasOwnProperty('ML')) {
        this.ML = initObj.ML
      }
      else {
        this.ML = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Features
    // Serialize message field [FL]
    bufferOffset = _serializer.bool(obj.FL, buffer, bufferOffset);
    // Serialize message field [FM]
    bufferOffset = _serializer.bool(obj.FM, buffer, bufferOffset);
    // Serialize message field [FR]
    bufferOffset = _serializer.bool(obj.FR, buffer, bufferOffset);
    // Serialize message field [MR]
    bufferOffset = _serializer.bool(obj.MR, buffer, bufferOffset);
    // Serialize message field [RR]
    bufferOffset = _serializer.bool(obj.RR, buffer, bufferOffset);
    // Serialize message field [RM]
    bufferOffset = _serializer.bool(obj.RM, buffer, bufferOffset);
    // Serialize message field [RL]
    bufferOffset = _serializer.bool(obj.RL, buffer, bufferOffset);
    // Serialize message field [ML]
    bufferOffset = _serializer.bool(obj.ML, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Features
    let len;
    let data = new Features(null);
    // Deserialize message field [FL]
    data.FL = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [FM]
    data.FM = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [FR]
    data.FR = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [MR]
    data.MR = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [RR]
    data.RR = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [RM]
    data.RM = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [RL]
    data.RL = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ML]
    data.ML = _deserializer.bool(buffer, bufferOffset);
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
    return 'a00bfafb469c58244d0cd8e717c0b7f8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Features(null);
    if (msg.FL !== undefined) {
      resolved.FL = msg.FL;
    }
    else {
      resolved.FL = false
    }

    if (msg.FM !== undefined) {
      resolved.FM = msg.FM;
    }
    else {
      resolved.FM = false
    }

    if (msg.FR !== undefined) {
      resolved.FR = msg.FR;
    }
    else {
      resolved.FR = false
    }

    if (msg.MR !== undefined) {
      resolved.MR = msg.MR;
    }
    else {
      resolved.MR = false
    }

    if (msg.RR !== undefined) {
      resolved.RR = msg.RR;
    }
    else {
      resolved.RR = false
    }

    if (msg.RM !== undefined) {
      resolved.RM = msg.RM;
    }
    else {
      resolved.RM = false
    }

    if (msg.RL !== undefined) {
      resolved.RL = msg.RL;
    }
    else {
      resolved.RL = false
    }

    if (msg.ML !== undefined) {
      resolved.ML = msg.ML;
    }
    else {
      resolved.ML = false
    }

    return resolved;
    }
};

module.exports = Features;
