;; Auto-generated. Do not edit!


(when (boundp 'object_list::Features)
  (if (not (find-package "OBJECT_LIST"))
    (make-package "OBJECT_LIST"))
  (shadow 'Features (find-package "OBJECT_LIST")))
(unless (find-package "OBJECT_LIST::FEATURES")
  (make-package "OBJECT_LIST::FEATURES"))

(in-package "ROS")
;;//! \htmlinclude Features.msg.html


(defclass object_list::Features
  :super ros::object
  :slots (_FL _FM _FR _MR _RR _RM _RL _ML ))

(defmethod object_list::Features
  (:init
   (&key
    ((:FL __FL) 0)
    ((:FM __FM) 0)
    ((:FR __FR) 0)
    ((:MR __MR) 0)
    ((:RR __RR) 0)
    ((:RM __RM) 0)
    ((:RL __RL) 0)
    ((:ML __ML) 0)
    )
   (send-super :init)
   (setq _FL (round __FL))
   (setq _FM (round __FM))
   (setq _FR (round __FR))
   (setq _MR (round __MR))
   (setq _RR (round __RR))
   (setq _RM (round __RM))
   (setq _RL (round __RL))
   (setq _ML (round __ML))
   self)
  (:FL
   (&optional __FL)
   (if __FL (setq _FL __FL)) _FL)
  (:FM
   (&optional __FM)
   (if __FM (setq _FM __FM)) _FM)
  (:FR
   (&optional __FR)
   (if __FR (setq _FR __FR)) _FR)
  (:MR
   (&optional __MR)
   (if __MR (setq _MR __MR)) _MR)
  (:RR
   (&optional __RR)
   (if __RR (setq _RR __RR)) _RR)
  (:RM
   (&optional __RM)
   (if __RM (setq _RM __RM)) _RM)
  (:RL
   (&optional __RL)
   (if __RL (setq _RL __RL)) _RL)
  (:ML
   (&optional __ML)
   (if __ML (setq _ML __ML)) _ML)
  (:serialization-length
   ()
   (+
    ;; uint8 _FL
    1
    ;; uint8 _FM
    1
    ;; uint8 _FR
    1
    ;; uint8 _MR
    1
    ;; uint8 _RR
    1
    ;; uint8 _RM
    1
    ;; uint8 _RL
    1
    ;; uint8 _ML
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _FL
       (write-byte _FL s)
     ;; uint8 _FM
       (write-byte _FM s)
     ;; uint8 _FR
       (write-byte _FR s)
     ;; uint8 _MR
       (write-byte _MR s)
     ;; uint8 _RR
       (write-byte _RR s)
     ;; uint8 _RM
       (write-byte _RM s)
     ;; uint8 _RL
       (write-byte _RL s)
     ;; uint8 _ML
       (write-byte _ML s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _FL
     (setq _FL (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _FM
     (setq _FM (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _FR
     (setq _FR (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _MR
     (setq _MR (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _RR
     (setq _RR (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _RM
     (setq _RM (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _RL
     (setq _RL (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _ML
     (setq _ML (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get object_list::Features :md5sum-) "acfb5ca82687e271a6722833317ebf1a")
(setf (get object_list::Features :datatype-) "object_list/Features")
(setf (get object_list::Features :definition-)
      "uint8 FL
uint8 FM
uint8 FR
uint8 MR
uint8 RR
uint8 RM
uint8 RL
uint8 ML

")



(provide :object_list/Features "acfb5ca82687e271a6722833317ebf1a")


