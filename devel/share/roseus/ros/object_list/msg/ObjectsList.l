;; Auto-generated. Do not edit!


(when (boundp 'object_list::ObjectsList)
  (if (not (find-package "OBJECT_LIST"))
    (make-package "OBJECT_LIST"))
  (shadow 'ObjectsList (find-package "OBJECT_LIST")))
(unless (find-package "OBJECT_LIST::OBJECTSLIST")
  (make-package "OBJECT_LIST::OBJECTSLIST"))

(in-package "ROS")
;;//! \htmlinclude ObjectsList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass object_list::ObjectsList
  :super ros::object
  :slots (_header _obj_list ))

(defmethod object_list::ObjectsList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obj_list __obj_list) (let (r) (dotimes (i 0) (push (instance object_list::ObjectList :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _obj_list __obj_list)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obj_list
   (&rest __obj_list)
   (if (keywordp (car __obj_list))
       (send* _obj_list __obj_list)
     (progn
       (if __obj_list (setq _obj_list (car __obj_list)))
       _obj_list)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; object_list/ObjectList[] _obj_list
    (apply #'+ (send-all _obj_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; object_list/ObjectList[] _obj_list
     (write-long (length _obj_list) s)
     (dolist (elem _obj_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; object_list/ObjectList[] _obj_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obj_list (let (r) (dotimes (i n) (push (instance object_list::ObjectList :init) r)) r))
     (dolist (elem- _obj_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get object_list::ObjectsList :md5sum-) "214beebd8035ba15b863e0e4f0dac47e")
(setf (get object_list::ObjectsList :datatype-) "object_list/ObjectsList")
(setf (get object_list::ObjectsList :definition-)
      "Header header
ObjectList[] obj_list

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

================================================================================
MSG: object_list/ObjectList
int32 obj_id
Geometric geometric
float64[36] covariance
Dimension dimension
float32 prop_existence
float32 prop_mov 
Classification classification
Features features

================================================================================
MSG: object_list/Geometric
float64 x
float64 y
float64 vx
float64 vy
float64 ax
float64 ay
float64 yaw
float64 yawrate

================================================================================
MSG: object_list/Dimension
float64 length
float64 width

================================================================================
MSG: object_list/Classification
float32 car
float32 truck
float32 motorcycle
float32 bicycle
float32 pedestrian
float32 stacionary
float32 other

================================================================================
MSG: object_list/Features
uint8 FL
uint8 FM
uint8 FR
uint8 MR
uint8 RR
uint8 RM
uint8 RL
uint8 ML

")



(provide :object_list/ObjectsList "214beebd8035ba15b863e0e4f0dac47e")


