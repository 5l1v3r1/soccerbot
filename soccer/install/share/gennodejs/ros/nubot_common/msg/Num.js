// Auto-generated. Do not edit!

// (in-package nubot_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Num {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.body_name = null;
      this.reference_frame = null;
      this.reference_point = null;
      this.wrench = null;
      this.start_time = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('body_name')) {
        this.body_name = initObj.body_name
      }
      else {
        this.body_name = '';
      }
      if (initObj.hasOwnProperty('reference_frame')) {
        this.reference_frame = initObj.reference_frame
      }
      else {
        this.reference_frame = '';
      }
      if (initObj.hasOwnProperty('reference_point')) {
        this.reference_point = initObj.reference_point
      }
      else {
        this.reference_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('wrench')) {
        this.wrench = initObj.wrench
      }
      else {
        this.wrench = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Num
    // Serialize message field [body_name]
    bufferOffset = _serializer.string(obj.body_name, buffer, bufferOffset);
    // Serialize message field [reference_frame]
    bufferOffset = _serializer.string(obj.reference_frame, buffer, bufferOffset);
    // Serialize message field [reference_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.reference_point, buffer, bufferOffset);
    // Serialize message field [wrench]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.wrench, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.duration(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Num
    let len;
    let data = new Num(null);
    // Deserialize message field [body_name]
    data.body_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [reference_frame]
    data.reference_frame = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [reference_point]
    data.reference_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [wrench]
    data.wrench = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.duration(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.body_name.length;
    length += object.reference_frame.length;
    return length + 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nubot_common/Num';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e37e6adf97eba5095baa77dffb71e5bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string body_name                          # Gazebo body to apply wrench (linear force and torque)
                                              # wrench is applied in the gazebo world by default
                                              # body names are prefixed by model name, e.g. pr2::base_link
    string reference_frame                    # wrench is defined in the reference frame of this entity
                                              # use inertial frame if left empty
                                              # frame names are bodies prefixed by model name, e.g. pr2::base_link
    geometry_msgs/Point  reference_point      # wrench is defined at this location in the reference frame
    geometry_msgs/Wrench wrench               # wrench applied to the origin of the body
    time start_time                           # (optional) wrench application start time (seconds)
                                              # if start_time is not specified, or
                                              # start_time < current time, start as soon as possible
    duration duration                         # optional duration of wrench application time (seconds)
                                              # if duration < 0, apply wrench continuously without end
                                              # if duration = 0, do nothing
                                              # if duration < step size, apply wrench
                                              # for one step size
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Num(null);
    if (msg.body_name !== undefined) {
      resolved.body_name = msg.body_name;
    }
    else {
      resolved.body_name = ''
    }

    if (msg.reference_frame !== undefined) {
      resolved.reference_frame = msg.reference_frame;
    }
    else {
      resolved.reference_frame = ''
    }

    if (msg.reference_point !== undefined) {
      resolved.reference_point = geometry_msgs.msg.Point.Resolve(msg.reference_point)
    }
    else {
      resolved.reference_point = new geometry_msgs.msg.Point()
    }

    if (msg.wrench !== undefined) {
      resolved.wrench = geometry_msgs.msg.Wrench.Resolve(msg.wrench)
    }
    else {
      resolved.wrench = new geometry_msgs.msg.Wrench()
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = Num;
