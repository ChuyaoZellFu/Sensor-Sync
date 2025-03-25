// Auto-generated. Do not edit!

// (in-package syn_cpp.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BBoxData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.target_info = null;
      this.det_bboxes = null;
      this.tracks_bbox = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('target_info')) {
        this.target_info = initObj.target_info
      }
      else {
        this.target_info = [];
      }
      if (initObj.hasOwnProperty('det_bboxes')) {
        this.det_bboxes = initObj.det_bboxes
      }
      else {
        this.det_bboxes = [];
      }
      if (initObj.hasOwnProperty('tracks_bbox')) {
        this.tracks_bbox = initObj.tracks_bbox
      }
      else {
        this.tracks_bbox = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BBoxData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [target_info]
    bufferOffset = _arraySerializer.float32(obj.target_info, buffer, bufferOffset, null);
    // Serialize message field [det_bboxes]
    bufferOffset = _arraySerializer.float32(obj.det_bboxes, buffer, bufferOffset, null);
    // Serialize message field [tracks_bbox]
    bufferOffset = _arraySerializer.float32(obj.tracks_bbox, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BBoxData
    let len;
    let data = new BBoxData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_info]
    data.target_info = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [det_bboxes]
    data.det_bboxes = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [tracks_bbox]
    data.tracks_bbox = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.target_info.length;
    length += 4 * object.det_bboxes.length;
    length += 4 * object.tracks_bbox.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'syn_cpp/BBoxData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7d80f33b42f9edc1cf91b21bdb199f7d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # BboxData.msg
    Header header                  # ROS 标准 Header（包含时间戳和坐标系）
    float32[] target_info          # 6 个元素，格式: [id, x1, y1, x2, y2, status]
    float32[] det_bboxes           # 检测框数组，每 5 个元素表示一个框: [x_min, y_min, x_max, y_max, score]
    float32[] tracks_bbox          # 跟踪框数组，每 4 个元素表示一个框: [x_min, y_min, x_max, y_max]
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BBoxData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.target_info !== undefined) {
      resolved.target_info = msg.target_info;
    }
    else {
      resolved.target_info = []
    }

    if (msg.det_bboxes !== undefined) {
      resolved.det_bboxes = msg.det_bboxes;
    }
    else {
      resolved.det_bboxes = []
    }

    if (msg.tracks_bbox !== undefined) {
      resolved.tracks_bbox = msg.tracks_bbox;
    }
    else {
      resolved.tracks_bbox = []
    }

    return resolved;
    }
};

module.exports = BBoxData;
