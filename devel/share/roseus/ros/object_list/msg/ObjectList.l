;; Auto-generated. Do not edit!


(when (boundp 'object_list::ObjectList)
  (if (not (find-package "OBJECT_LIST"))
    (make-package "OBJECT_LIST"))
  (shadow 'ObjectList (find-package "OBJECT_LIST")))
(unless (find-package "OBJECT_LIST::OBJECTLIST")
  (make-package "OBJECT_LIST::OBJECTLIST"))

(in-package "ROS")
;;//! \htmlinclude ObjectList.msg.html


(defclass object_list::ObjectList
  :super ros::object
  :slots (_obj_id _geometric _covariance _dimension _prop_existence _prop_mov _classification _features ))

(defmethod object_list::ObjectList
  (:init
   (&key
    ((:obj_id __obj_id) 0)
    ((:geometric __geometric) (instance object_list::Geometric :init))
    ((:covariance __covariance) (make-array 36 :initial-element 0.0 :element-type :float))
    ((:dimension __dimension) (instance object_list::Dimension :init))
    ((:prop_existence __prop_existence) 0.0)
    ((:prop_mov __prop_mov) 0.0)
    ((:classification __classification) (instance object_list::Classification :init))
    ((:features __features) (instance object_list::Features :init))
    )
   (send-super :init)
   (setq _obj_id (round __obj_id))
   (setq _geometric __geometric)
   (setq _covariance __covariance)
   (setq _dimension __dimension)
   (setq _prop_existence (float __prop_existence))
   (setq _prop_mov (float __prop_mov))
   (setq _classification __classification)
   (setq _features __features)
   self)
  (:obj_id
   (&optional __obj_id)
   (if __obj_id (setq _obj_id __obj_id)) _obj_id)
  (:geometric
   (&rest __geometric)
   (if (keywordp (car __geometric))
       (send* _geometric __geometric)
     (progn
       (if __geometric (setq _geometric (car __geometric)))
       _geometric)))
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:dimension
   (&rest __dimension)
   (if (keywordp (car __dimension))
       (send* _dimension __dimension)
     (progn
       (if __dimension (setq _dimension (car __dimension)))
       _dimension)))
  (:prop_existence
   (&optional __prop_existence)
   (if __prop_existence (setq _prop_existence __prop_existence)) _prop_existence)
  (:prop_mov
   (&optional __prop_mov)
   (if __prop_mov (setq _prop_mov __prop_mov)) _prop_mov)
  (:classification
   (&rest __classification)
   (if (keywordp (car __classification))
       (send* _classification __classification)
     (progn
       (if __classification (setq _classification (car __classification)))
       _classification)))
  (:features
   (&rest __features)
   (if (keywordp (car __features))
       (send* _features __features)
     (progn
       (if __features (setq _features (car __features)))
       _features)))
  (:serialization-length
   ()
   (+
    ;; int32 _obj_id
    4
    ;; object_list/Geometric _geometric
    (send _geometric :serialization-length)
    ;; float64[36] _covariance
    (* 8    36)
    ;; object_list/Dimension _dimension
    (send _dimension :serialization-length)
    ;; float32 _prop_existence
    4
    ;; float32 _prop_mov
    4
    ;; object_list/Classification _classification
    (send _classification :serialization-length)
    ;; object_list/Features _features
    (send _features :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _obj_id
       (write-long _obj_id s)
     ;; object_list/Geometric _geometric
       (send _geometric :serialize s)
     ;; float64[36] _covariance
     (dotimes (i 36)
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; object_list/Dimension _dimension
       (send _dimension :serialize s)
     ;; float32 _prop_existence
       (sys::poke _prop_existence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _prop_mov
       (sys::poke _prop_mov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; object_list/Classification _classification
       (send _classification :serialize s)
     ;; object_list/Features _features
       (send _features :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _obj_id
     (setq _obj_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; object_list/Geometric _geometric
     (send _geometric :deserialize buf ptr-) (incf ptr- (send _geometric :serialization-length))
   ;; float64[36] _covariance
   (dotimes (i (length _covariance))
     (setf (elt _covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; object_list/Dimension _dimension
     (send _dimension :deserialize buf ptr-) (incf ptr- (send _dimension :serialization-length))
   ;; float32 _prop_existence
     (setq _prop_existence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _prop_mov
     (setq _prop_mov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; object_list/Classification _classification
     (send _classification :deserialize buf ptr-) (incf ptr- (send _classification :serialization-length))
   ;; object_list/Features _features
     (send _features :deserialize buf ptr-) (incf ptr- (send _features :serialization-length))
   ;;
   self)
  )

(setf (get object_list::ObjectList :md5sum-) "2a58c1f2f7bc179c5919b6ef13df4d0d")
(setf (get object_list::ObjectList :datatype-) "object_list/ObjectList")
(setf (get object_list::ObjectList :definition-)
      "int32 obj_id
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
float64 height

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
bool FL
bool FM
bool FR
bool MR
bool RR
bool RM
bool RL
bool ML

")



(provide :object_list/ObjectList "2a58c1f2f7bc179c5919b6ef13df4d0d")


