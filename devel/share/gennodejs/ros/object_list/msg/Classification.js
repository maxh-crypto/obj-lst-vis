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

class Classification {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.car = null;
      this.truck = null;
      this.motorcycle = null;
      this.bicycle = null;
      this.pedestrian = null;
      this.stacionary = null;
      this.other = null;
    }
    else {
      if (initObj.hasOwnProperty('car')) {
        this.car = initObj.car
      }
      else {
        this.car = 0.0;
      }
      if (initObj.hasOwnProperty('truck')) {
        this.truck = initObj.truck
      }
      else {
        this.truck = 0.0;
      }
      if (initObj.hasOwnProperty('motorcycle')) {
        this.motorcycle = initObj.motorcycle
      }
      else {
        this.motorcycle = 0.0;
      }
      if (initObj.hasOwnProperty('bicycle')) {
        this.bicycle = initObj.bicycle
      }
      else {
        this.bicycle = 0.0;
      }
      if (initObj.hasOwnProperty('pedestrian')) {
        this.pedestrian = initObj.pedestrian
      }
      else {
        this.pedestrian = 0.0;
      }
      if (initObj.hasOwnProperty('stacionary')) {
        this.stacionary = initObj.stacionary
      }
      else {
        this.stacionary = 0.0;
      }
      if (initObj.hasOwnProperty('other')) {
        this.other = initObj.other
      }
      else {
        this.other = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Classification
    // Serialize message field [car]
    bufferOffset = _serializer.float32(obj.car, buffer, bufferOffset);
    // Serialize message field [truck]
    bufferOffset = _serializer.float32(obj.truck, buffer, bufferOffset);
    // Serialize message field [motorcycle]
    bufferOffset = _serializer.float32(obj.motorcycle, buffer, bufferOffset);
    // Serialize message field [bicycle]
    bufferOffset = _serializer.float32(obj.bicycle, buffer, bufferOffset);
    // Serialize message field [pedestrian]
    bufferOffset = _serializer.float32(obj.pedestrian, buffer, bufferOffset);
    // Serialize message field [stacionary]
    bufferOffset = _serializer.float32(obj.stacionary, buffer, bufferOffset);
    // Serialize message field [other]
    bufferOffset = _serializer.float32(obj.other, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Classification
    let len;
    let data = new Classification(null);
    // Deserialize message field [car]
    data.car = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [truck]
    data.truck = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motorcycle]
    data.motorcycle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bicycle]
    data.bicycle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pedestrian]
    data.pedestrian = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [stacionary]
    data.stacionary = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [other]
    data.other = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_list/Classification';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37b53ddc70d71a526ada035ab3f28e33';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 car
    float32 truck
    float32 motorcycle
    float32 bicycle
    float32 pedestrian
    float32 stacionary
    float32 other
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Classification(null);
    if (msg.car !== undefined) {
      resolved.car = msg.car;
    }
    else {
      resolved.car = 0.0
    }

    if (msg.truck !== undefined) {
      resolved.truck = msg.truck;
    }
    else {
      resolved.truck = 0.0
    }

    if (msg.motorcycle !== undefined) {
      resolved.motorcycle = msg.motorcycle;
    }
    else {
      resolved.motorcycle = 0.0
    }

    if (msg.bicycle !== undefined) {
      resolved.bicycle = msg.bicycle;
    }
    else {
      resolved.bicycle = 0.0
    }

    if (msg.pedestrian !== undefined) {
      resolved.pedestrian = msg.pedestrian;
    }
    else {
      resolved.pedestrian = 0.0
    }

    if (msg.stacionary !== undefined) {
      resolved.stacionary = msg.stacionary;
    }
    else {
      resolved.stacionary = 0.0
    }

    if (msg.other !== undefined) {
      resolved.other = msg.other;
    }
    else {
      resolved.other = 0.0
    }

    return resolved;
    }
};

module.exports = Classification;
