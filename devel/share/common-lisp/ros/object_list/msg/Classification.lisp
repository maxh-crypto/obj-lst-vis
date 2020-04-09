; Auto-generated. Do not edit!


(cl:in-package object_list-msg)


;//! \htmlinclude Classification.msg.html

(cl:defclass <Classification> (roslisp-msg-protocol:ros-message)
  ((car
    :reader car
    :initarg :car
    :type cl:float
    :initform 0.0)
   (truck
    :reader truck
    :initarg :truck
    :type cl:float
    :initform 0.0)
   (motorcycle
    :reader motorcycle
    :initarg :motorcycle
    :type cl:float
    :initform 0.0)
   (bicycle
    :reader bicycle
    :initarg :bicycle
    :type cl:float
    :initform 0.0)
   (pedestrian
    :reader pedestrian
    :initarg :pedestrian
    :type cl:float
    :initform 0.0)
   (stacionary
    :reader stacionary
    :initarg :stacionary
    :type cl:float
    :initform 0.0)
   (other
    :reader other
    :initarg :other
    :type cl:float
    :initform 0.0))
)

(cl:defclass Classification (<Classification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Classification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Classification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_list-msg:<Classification> is deprecated: use object_list-msg:Classification instead.")))

(cl:ensure-generic-function 'car-val :lambda-list '(m))
(cl:defmethod car-val ((m <Classification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:car-val is deprecated.  Use object_list-msg:car instead.")
  (car m))

(cl:ensure-generic-function 'truck-val :lambda-list '(m))
(cl:defmethod truck-val ((m <Classification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:truck-val is deprecated.  Use object_list-msg:truck instead.")
  (truck m))

(cl:ensure-generic-function 'motorcycle-val :lambda-list '(m))
(cl:defmethod motorcycle-val ((m <Classification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:motorcycle-val is deprecated.  Use object_list-msg:motorcycle instead.")
  (motorcycle m))

(cl:ensure-generic-function 'bicycle-val :lambda-list '(m))
(cl:defmethod bicycle-val ((m <Classification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:bicycle-val is deprecated.  Use object_list-msg:bicycle instead.")
  (bicycle m))

(cl:ensure-generic-function 'pedestrian-val :lambda-list '(m))
(cl:defmethod pedestrian-val ((m <Classification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:pedestrian-val is deprecated.  Use object_list-msg:pedestrian instead.")
  (pedestrian m))

(cl:ensure-generic-function 'stacionary-val :lambda-list '(m))
(cl:defmethod stacionary-val ((m <Classification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:stacionary-val is deprecated.  Use object_list-msg:stacionary instead.")
  (stacionary m))

(cl:ensure-generic-function 'other-val :lambda-list '(m))
(cl:defmethod other-val ((m <Classification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:other-val is deprecated.  Use object_list-msg:other instead.")
  (other m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Classification>) ostream)
  "Serializes a message object of type '<Classification>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'car))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'truck))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motorcycle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bicycle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pedestrian))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'stacionary))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'other))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Classification>) istream)
  "Deserializes a message object of type '<Classification>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'car) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'truck) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motorcycle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bicycle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pedestrian) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stacionary) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'other) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Classification>)))
  "Returns string type for a message object of type '<Classification>"
  "object_list/Classification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Classification)))
  "Returns string type for a message object of type 'Classification"
  "object_list/Classification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Classification>)))
  "Returns md5sum for a message object of type '<Classification>"
  "37b53ddc70d71a526ada035ab3f28e33")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Classification)))
  "Returns md5sum for a message object of type 'Classification"
  "37b53ddc70d71a526ada035ab3f28e33")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Classification>)))
  "Returns full string definition for message of type '<Classification>"
  (cl:format cl:nil "float32 car~%float32 truck~%float32 motorcycle~%float32 bicycle~%float32 pedestrian~%float32 stacionary~%float32 other~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Classification)))
  "Returns full string definition for message of type 'Classification"
  (cl:format cl:nil "float32 car~%float32 truck~%float32 motorcycle~%float32 bicycle~%float32 pedestrian~%float32 stacionary~%float32 other~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Classification>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Classification>))
  "Converts a ROS message object to a list"
  (cl:list 'Classification
    (cl:cons ':car (car msg))
    (cl:cons ':truck (truck msg))
    (cl:cons ':motorcycle (motorcycle msg))
    (cl:cons ':bicycle (bicycle msg))
    (cl:cons ':pedestrian (pedestrian msg))
    (cl:cons ':stacionary (stacionary msg))
    (cl:cons ':other (other msg))
))
