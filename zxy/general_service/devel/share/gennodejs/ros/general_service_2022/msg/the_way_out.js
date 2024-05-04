// Auto-generated. Do not edit!

// (in-package general_service_2022.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class the_way_out {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle = null;
      this.max_length = null;
      this.the_angle_of_max_length = null;
    }
    else {
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0;
      }
      if (initObj.hasOwnProperty('max_length')) {
        this.max_length = initObj.max_length
      }
      else {
        this.max_length = 0.0;
      }
      if (initObj.hasOwnProperty('the_angle_of_max_length')) {
        this.the_angle_of_max_length = initObj.the_angle_of_max_length
      }
      else {
        this.the_angle_of_max_length = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type the_way_out
    // Serialize message field [angle]
    bufferOffset = _serializer.int32(obj.angle, buffer, bufferOffset);
    // Serialize message field [max_length]
    bufferOffset = _serializer.float64(obj.max_length, buffer, bufferOffset);
    // Serialize message field [the_angle_of_max_length]
    bufferOffset = _serializer.int32(obj.the_angle_of_max_length, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type the_way_out
    let len;
    let data = new the_way_out(null);
    // Deserialize message field [angle]
    data.angle = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_length]
    data.max_length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [the_angle_of_max_length]
    data.the_angle_of_max_length = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'general_service_2022/the_way_out';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be349f47f00195efdd7bf46fc8b5963c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 angle
    float64 max_length
    int32 the_angle_of_max_length
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new the_way_out(null);
    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0
    }

    if (msg.max_length !== undefined) {
      resolved.max_length = msg.max_length;
    }
    else {
      resolved.max_length = 0.0
    }

    if (msg.the_angle_of_max_length !== undefined) {
      resolved.the_angle_of_max_length = msg.the_angle_of_max_length;
    }
    else {
      resolved.the_angle_of_max_length = 0
    }

    return resolved;
    }
};

module.exports = the_way_out;
