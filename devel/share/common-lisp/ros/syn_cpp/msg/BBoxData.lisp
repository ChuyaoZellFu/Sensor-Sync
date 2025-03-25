; Auto-generated. Do not edit!


(cl:in-package syn_cpp-msg)


;//! \htmlinclude BBoxData.msg.html

(cl:defclass <BBoxData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (target_info
    :reader target_info
    :initarg :target_info
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (det_bboxes
    :reader det_bboxes
    :initarg :det_bboxes
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (tracks_bbox
    :reader tracks_bbox
    :initarg :tracks_bbox
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass BBoxData (<BBoxData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BBoxData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BBoxData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name syn_cpp-msg:<BBoxData> is deprecated: use syn_cpp-msg:BBoxData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BBoxData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syn_cpp-msg:header-val is deprecated.  Use syn_cpp-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'target_info-val :lambda-list '(m))
(cl:defmethod target_info-val ((m <BBoxData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syn_cpp-msg:target_info-val is deprecated.  Use syn_cpp-msg:target_info instead.")
  (target_info m))

(cl:ensure-generic-function 'det_bboxes-val :lambda-list '(m))
(cl:defmethod det_bboxes-val ((m <BBoxData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syn_cpp-msg:det_bboxes-val is deprecated.  Use syn_cpp-msg:det_bboxes instead.")
  (det_bboxes m))

(cl:ensure-generic-function 'tracks_bbox-val :lambda-list '(m))
(cl:defmethod tracks_bbox-val ((m <BBoxData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader syn_cpp-msg:tracks_bbox-val is deprecated.  Use syn_cpp-msg:tracks_bbox instead.")
  (tracks_bbox m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BBoxData>) ostream)
  "Serializes a message object of type '<BBoxData>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'det_bboxes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'det_bboxes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracks_bbox))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'tracks_bbox))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BBoxData>) istream)
  "Deserializes a message object of type '<BBoxData>"
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
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'det_bboxes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'det_bboxes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracks_bbox) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracks_bbox)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BBoxData>)))
  "Returns string type for a message object of type '<BBoxData>"
  "syn_cpp/BBoxData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BBoxData)))
  "Returns string type for a message object of type 'BBoxData"
  "syn_cpp/BBoxData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BBoxData>)))
  "Returns md5sum for a message object of type '<BBoxData>"
  "7d80f33b42f9edc1cf91b21bdb199f7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BBoxData)))
  "Returns md5sum for a message object of type 'BBoxData"
  "7d80f33b42f9edc1cf91b21bdb199f7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BBoxData>)))
  "Returns full string definition for message of type '<BBoxData>"
  (cl:format cl:nil "# BboxData.msg~%Header header                  # ROS 标准 Header（包含时间戳和坐标系）~%float32[] target_info          # 6 个元素，格式: [id, x1, y1, x2, y2, status]~%float32[] det_bboxes           # 检测框数组，每 5 个元素表示一个框: [x_min, y_min, x_max, y_max, score]~%float32[] tracks_bbox          # 跟踪框数组，每 4 个元素表示一个框: [x_min, y_min, x_max, y_max]~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BBoxData)))
  "Returns full string definition for message of type 'BBoxData"
  (cl:format cl:nil "# BboxData.msg~%Header header                  # ROS 标准 Header（包含时间戳和坐标系）~%float32[] target_info          # 6 个元素，格式: [id, x1, y1, x2, y2, status]~%float32[] det_bboxes           # 检测框数组，每 5 个元素表示一个框: [x_min, y_min, x_max, y_max, score]~%float32[] tracks_bbox          # 跟踪框数组，每 4 个元素表示一个框: [x_min, y_min, x_max, y_max]~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BBoxData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'target_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'det_bboxes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracks_bbox) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BBoxData>))
  "Converts a ROS message object to a list"
  (cl:list 'BBoxData
    (cl:cons ':header (header msg))
    (cl:cons ':target_info (target_info msg))
    (cl:cons ':det_bboxes (det_bboxes msg))
    (cl:cons ':tracks_bbox (tracks_bbox msg))
))
