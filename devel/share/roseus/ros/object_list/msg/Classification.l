;; Auto-generated. Do not edit!


(when (boundp 'object_list::Classification)
  (if (not (find-package "OBJECT_LIST"))
    (make-package "OBJECT_LIST"))
  (shadow 'Classification (find-package "OBJECT_LIST")))
(unless (find-package "OBJECT_LIST::CLASSIFICATION")
  (make-package "OBJECT_LIST::CLASSIFICATION"))

(in-package "ROS")
;;//! \htmlinclude Classification.msg.html


(defclass object_list::Classification
  :super ros::object
  :slots (_car _truck _motorcycle _bicycle _pedestrian _stacionary _other ))

(defmethod object_list::Classification
  (:init
   (&key
    ((:car __car) 0.0)
    ((:truck __truck) 0.0)
    ((:motorcycle __motorcycle) 0.0)
    ((:bicycle __bicycle) 0.0)
    ((:pedestrian __pedestrian) 0.0)
    ((:stacionary __stacionary) 0.0)
    ((:other __other) 0.0)
    )
   (send-super :init)
   (setq _car (float __car))
   (setq _truck (float __truck))
   (setq _motorcycle (float __motorcycle))
   (setq _bicycle (float __bicycle))
   (setq _pedestrian (float __pedestrian))
   (setq _stacionary (float __stacionary))
   (setq _other (float __other))
   self)
  (:car
   (&optional __car)
   (if __car (setq _car __car)) _car)
  (:truck
   (&optional __truck)
   (if __truck (setq _truck __truck)) _truck)
  (:motorcycle
   (&optional __motorcycle)
   (if __motorcycle (setq _motorcycle __motorcycle)) _motorcycle)
  (:bicycle
   (&optional __bicycle)
   (if __bicycle (setq _bicycle __bicycle)) _bicycle)
  (:pedestrian
   (&optional __pedestrian)
   (if __pedestrian (setq _pedestrian __pedestrian)) _pedestrian)
  (:stacionary
   (&optional __stacionary)
   (if __stacionary (setq _stacionary __stacionary)) _stacionary)
  (:other
   (&optional __other)
   (if __other (setq _other __other)) _other)
  (:serialization-length
   ()
   (+
    ;; float32 _car
    4
    ;; float32 _truck
    4
    ;; float32 _motorcycle
    4
    ;; float32 _bicycle
    4
    ;; float32 _pedestrian
    4
    ;; float32 _stacionary
    4
    ;; float32 _other
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _car
       (sys::poke _car (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _truck
       (sys::poke _truck (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motorcycle
       (sys::poke _motorcycle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bicycle
       (sys::poke _bicycle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pedestrian
       (sys::poke _pedestrian (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _stacionary
       (sys::poke _stacionary (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _other
       (sys::poke _other (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _car
     (setq _car (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _truck
     (setq _truck (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motorcycle
     (setq _motorcycle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bicycle
     (setq _bicycle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pedestrian
     (setq _pedestrian (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _stacionary
     (setq _stacionary (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _other
     (setq _other (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get object_list::Classification :md5sum-) "37b53ddc70d71a526ada035ab3f28e33")
(setf (get object_list::Classification :datatype-) "object_list/Classification")
(setf (get object_list::Classification :definition-)
      "float32 car
float32 truck
float32 motorcycle
float32 bicycle
float32 pedestrian
float32 stacionary
float32 other

")



(provide :object_list/Classification "37b53ddc70d71a526ada035ab3f28e33")


