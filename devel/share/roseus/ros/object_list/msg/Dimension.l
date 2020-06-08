;; Auto-generated. Do not edit!


(when (boundp 'object_list::Dimension)
  (if (not (find-package "OBJECT_LIST"))
    (make-package "OBJECT_LIST"))
  (shadow 'Dimension (find-package "OBJECT_LIST")))
(unless (find-package "OBJECT_LIST::DIMENSION")
  (make-package "OBJECT_LIST::DIMENSION"))

(in-package "ROS")
;;//! \htmlinclude Dimension.msg.html


(defclass object_list::Dimension
  :super ros::object
  :slots (_length _width _height ))

(defmethod object_list::Dimension
  (:init
   (&key
    ((:length __length) 0.0)
    ((:width __width) 0.0)
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _length (float __length))
   (setq _width (float __width))
   (setq _height (float __height))
   self)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; float64 _length
    8
    ;; float64 _width
    8
    ;; float64 _height
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _length
       (sys::poke _length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _length
     (setq _length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get object_list::Dimension :md5sum-) "95f1c31b94fb56f33a669e4bf805939a")
(setf (get object_list::Dimension :datatype-) "object_list/Dimension")
(setf (get object_list::Dimension :definition-)
      "float64 length
float64 width
float64 height

")



(provide :object_list/Dimension "95f1c31b94fb56f33a669e4bf805939a")


