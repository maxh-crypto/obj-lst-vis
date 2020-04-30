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
    ((:FL __FL) nil)
    ((:FM __FM) nil)
    ((:FR __FR) nil)
    ((:MR __MR) nil)
    ((:RR __RR) nil)
    ((:RM __RM) nil)
    ((:RL __RL) nil)
    ((:ML __ML) nil)
    )
   (send-super :init)
   (setq _FL __FL)
   (setq _FM __FM)
   (setq _FR __FR)
   (setq _MR __MR)
   (setq _RR __RR)
   (setq _RM __RM)
   (setq _RL __RL)
   (setq _ML __ML)
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
    ;; bool _FL
    1
    ;; bool _FM
    1
    ;; bool _FR
    1
    ;; bool _MR
    1
    ;; bool _RR
    1
    ;; bool _RM
    1
    ;; bool _RL
    1
    ;; bool _ML
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _FL
       (if _FL (write-byte -1 s) (write-byte 0 s))
     ;; bool _FM
       (if _FM (write-byte -1 s) (write-byte 0 s))
     ;; bool _FR
       (if _FR (write-byte -1 s) (write-byte 0 s))
     ;; bool _MR
       (if _MR (write-byte -1 s) (write-byte 0 s))
     ;; bool _RR
       (if _RR (write-byte -1 s) (write-byte 0 s))
     ;; bool _RM
       (if _RM (write-byte -1 s) (write-byte 0 s))
     ;; bool _RL
       (if _RL (write-byte -1 s) (write-byte 0 s))
     ;; bool _ML
       (if _ML (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _FL
     (setq _FL (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _FM
     (setq _FM (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _FR
     (setq _FR (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _MR
     (setq _MR (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _RR
     (setq _RR (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _RM
     (setq _RM (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _RL
     (setq _RL (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ML
     (setq _ML (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get object_list::Features :md5sum-) "a00bfafb469c58244d0cd8e717c0b7f8")
(setf (get object_list::Features :datatype-) "object_list/Features")
(setf (get object_list::Features :definition-)
      "bool FL
bool FM
bool FR
bool MR
bool RR
bool RM
bool RL
bool ML

")



(provide :object_list/Features "a00bfafb469c58244d0cd8e717c0b7f8")


