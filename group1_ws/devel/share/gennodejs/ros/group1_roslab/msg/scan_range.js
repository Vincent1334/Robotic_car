// Auto-generated. Do not edit!

// (in-package group1_roslab.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class scan_range {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.max_v = null;
      this.min_v = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('max_v')) {
        this.max_v = initObj.max_v
      }
      else {
        this.max_v = 0.0;
      }
      if (initObj.hasOwnProperty('min_v')) {
        this.min_v = initObj.min_v
      }
      else {
        this.min_v = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type scan_range
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [max_v]
    bufferOffset = _serializer.float64(obj.max_v, buffer, bufferOffset);
    // Serialize message field [min_v]
    bufferOffset = _serializer.float64(obj.min_v, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type scan_range
    let len;
    let data = new scan_range(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_v]
    data.max_v = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [min_v]
    data.min_v = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'group1_roslab/scan_range';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f6032bf04499a2f200f7b14dd796c86';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 max_v
    float64 min_v
    
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new scan_range(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.max_v !== undefined) {
      resolved.max_v = msg.max_v;
    }
    else {
      resolved.max_v = 0.0
    }

    if (msg.min_v !== undefined) {
      resolved.min_v = msg.min_v;
    }
    else {
      resolved.min_v = 0.0
    }

    return resolved;
    }
};

module.exports = scan_range;
