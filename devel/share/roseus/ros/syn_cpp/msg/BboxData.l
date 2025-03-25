;; Auto-generated. Do not edit!


(when (boundp 'syn_cpp::BboxData)
  (if (not (find-package "SYN_CPP"))
    (make-package "SYN_CPP"))
  (shadow 'BboxData (find-package "SYN_CPP")))
(unless (find-package "SYN_CPP::BBOXDATA")
  (make-package "SYN_CPP::BBOXDATA"))

(in-package "ROS")
;;//! \htmlinclude BboxData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass syn_cpp::BboxData
  :super ros::object
  :slots (_header _target_info ))

(defmethod syn_cpp::BboxData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:target_info __target_info) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _target_info __target_info)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:target_info
   (&optional __target_info)
   (if __target_info (setq _target_info __target_info)) _target_info)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[] _target_info
    (* 4    (length _target_info)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[] _target_info
     (write-long (length _target_info) s)
     (dotimes (i (length _target_info))
       (sys::poke (elt _target_info i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[] _target_info
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _target_info (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _target_info i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get syn_cpp::BboxData :md5sum-) "eb9a702374fae8e6b561aa485440b9bb")
(setf (get syn_cpp::BboxData :datatype-) "syn_cpp/BboxData")
(setf (get syn_cpp::BboxData :definition-)
      "# BboxData.msg
Header header                  # ROS 标准 Header（包含时间戳和坐标系）
float32[] target_info          # 6 个元素，格式: [id, x1, y1, x2, y2, status]
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

")



(provide :syn_cpp/BboxData "eb9a702374fae8e6b561aa485440b9bb")


