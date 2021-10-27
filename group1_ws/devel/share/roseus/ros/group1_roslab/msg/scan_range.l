;; Auto-generated. Do not edit!


(when (boundp 'group1_roslab::scan_range)
  (if (not (find-package "GROUP1_ROSLAB"))
    (make-package "GROUP1_ROSLAB"))
  (shadow 'scan_range (find-package "GROUP1_ROSLAB")))
(unless (find-package "GROUP1_ROSLAB::SCAN_RANGE")
  (make-package "GROUP1_ROSLAB::SCAN_RANGE"))

(in-package "ROS")
;;//! \htmlinclude scan_range.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass group1_roslab::scan_range
  :super ros::object
  :slots (_header _max_v _min_v ))

(defmethod group1_roslab::scan_range
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:max_v __max_v) 0.0)
    ((:min_v __min_v) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _max_v (float __max_v))
   (setq _min_v (float __min_v))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:max_v
   (&optional __max_v)
   (if __max_v (setq _max_v __max_v)) _max_v)
  (:min_v
   (&optional __min_v)
   (if __min_v (setq _min_v __min_v)) _min_v)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _max_v
    8
    ;; float64 _min_v
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _max_v
       (sys::poke _max_v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _min_v
       (sys::poke _min_v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _max_v
     (setq _max_v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _min_v
     (setq _min_v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get group1_roslab::scan_range :md5sum-) "4f6032bf04499a2f200f7b14dd796c86")
(setf (get group1_roslab::scan_range :datatype-) "group1_roslab/scan_range")
(setf (get group1_roslab::scan_range :definition-)
      "Header header
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

")



(provide :group1_roslab/scan_range "4f6032bf04499a2f200f7b14dd796c86")


