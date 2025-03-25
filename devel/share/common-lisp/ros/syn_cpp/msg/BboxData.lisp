; Auto-generated. Do not edit!


(cl:in-package syn_cpp-msg)


;//! \htmlinclude BboxData.msg.html

(cl:defclass <BboxData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (target_info
    :reader target_info
    :initarg :target_info
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass BboxData (<BboxData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BboxData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BboxData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name syn_cpp-msg:<BboxData> is deprecated: use syn_cpp-msg:BboxData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BboxData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syn_cpp-msg:header-val is deprecated.  Use syn_cpp-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'target_info-val :lambda-list '(m))
(cl:defmethod target_info-val ((m <BboxData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syn_cpp-msg:target_info-val is deprecated.  Use syn_cpp-msg:target_info instead.")
  (target_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BboxData>) ostream)
  "Serializes a message object of type '<BboxData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'target_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'target_info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BboxData>) istream)
  "Deserializes a message object of type '<BboxData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'target_info) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'target_info)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BboxData>)))
  "Returns string type for a message object of type '<BboxData>"
  "syn_cpp/BboxData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BboxData)))
  "Returns string type for a message object of type 'BboxData"
  "syn_cpp/BboxData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BboxData>)))
  "Returns md5sum for a message object of type '<BboxData>"
  "eb9a702374fae8e6b561aa485440b9bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BboxData)))
  "Returns md5sum for a message object of type 'BboxData"
  "eb9a702374fae8e6b561aa485440b9bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BboxData>)))
  "Returns full string definition for message of type '<BboxData>"
  (cl:format cl:nil "# BboxData.msg~%Header header                  # ROS 标准 Header（包含时间戳和坐标系）~%float32[] target_info          # 6 个元素，格式: [id, x1, y1, x2, y2, status]~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BboxData)))
  "Returns full string definition for message of type 'BboxData"
  (cl:format cl:nil "# BboxData.msg~%Header header                  # ROS 标准 Header（包含时间戳和坐标系）~%float32[] target_info          # 6 个元素，格式: [id, x1, y1, x2, y2, status]~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BboxData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BboxData>))
  "Converts a ROS message object to a list"
  (cl:list 'BboxData
    (cl:cons ':header (header msg))
    (cl:cons ':target_info (target_info msg))
))
