; Auto-generated. Do not edit!


(cl:in-package object_list-msg)


;//! \htmlinclude Features.msg.html

(cl:defclass <Features> (roslisp-msg-protocol:ros-message)
  ((FL
    :reader FL
    :initarg :FL
    :type cl:fixnum
    :initform 0)
   (FM
    :reader FM
    :initarg :FM
    :type cl:fixnum
    :initform 0)
   (FR
    :reader FR
    :initarg :FR
    :type cl:fixnum
    :initform 0)
   (MR
    :reader MR
    :initarg :MR
    :type cl:fixnum
    :initform 0)
   (RR
    :reader RR
    :initarg :RR
    :type cl:fixnum
    :initform 0)
   (RM
    :reader RM
    :initarg :RM
    :type cl:fixnum
    :initform 0)
   (RL
    :reader RL
    :initarg :RL
    :type cl:fixnum
    :initform 0)
   (ML
    :reader ML
    :initarg :ML
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Features (<Features>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Features>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Features)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_list-msg:<Features> is deprecated: use object_list-msg:Features instead.")))

(cl:ensure-generic-function 'FL-val :lambda-list '(m))
(cl:defmethod FL-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:FL-val is deprecated.  Use object_list-msg:FL instead.")
  (FL m))

(cl:ensure-generic-function 'FM-val :lambda-list '(m))
(cl:defmethod FM-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:FM-val is deprecated.  Use object_list-msg:FM instead.")
  (FM m))

(cl:ensure-generic-function 'FR-val :lambda-list '(m))
(cl:defmethod FR-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:FR-val is deprecated.  Use object_list-msg:FR instead.")
  (FR m))

(cl:ensure-generic-function 'MR-val :lambda-list '(m))
(cl:defmethod MR-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:MR-val is deprecated.  Use object_list-msg:MR instead.")
  (MR m))

(cl:ensure-generic-function 'RR-val :lambda-list '(m))
(cl:defmethod RR-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:RR-val is deprecated.  Use object_list-msg:RR instead.")
  (RR m))

(cl:ensure-generic-function 'RM-val :lambda-list '(m))
(cl:defmethod RM-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:RM-val is deprecated.  Use object_list-msg:RM instead.")
  (RM m))

(cl:ensure-generic-function 'RL-val :lambda-list '(m))
(cl:defmethod RL-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:RL-val is deprecated.  Use object_list-msg:RL instead.")
  (RL m))

(cl:ensure-generic-function 'ML-val :lambda-list '(m))
(cl:defmethod ML-val ((m <Features>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_list-msg:ML-val is deprecated.  Use object_list-msg:ML instead.")
  (ML m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Features>) ostream)
  "Serializes a message object of type '<Features>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'FL)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'FM)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'FR)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'MR)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'RR)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'RM)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'RL)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ML)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Features>) istream)
  "Deserializes a message object of type '<Features>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'FL)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'FM)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'FR)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'MR)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'RR)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'RM)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'RL)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ML)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Features>)))
  "Returns string type for a message object of type '<Features>"
  "object_list/Features")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Features)))
  "Returns string type for a message object of type 'Features"
  "object_list/Features")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Features>)))
  "Returns md5sum for a message object of type '<Features>"
  "acfb5ca82687e271a6722833317ebf1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Features)))
  "Returns md5sum for a message object of type 'Features"
  "acfb5ca82687e271a6722833317ebf1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Features>)))
  "Returns full string definition for message of type '<Features>"
  (cl:format cl:nil "uint8 FL~%uint8 FM~%uint8 FR~%uint8 MR~%uint8 RR~%uint8 RM~%uint8 RL~%uint8 ML~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Features)))
  "Returns full string definition for message of type 'Features"
  (cl:format cl:nil "uint8 FL~%uint8 FM~%uint8 FR~%uint8 MR~%uint8 RR~%uint8 RM~%uint8 RL~%uint8 ML~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Features>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Features>))
  "Converts a ROS message object to a list"
  (cl:list 'Features
    (cl:cons ':FL (FL msg))
    (cl:cons ':FM (FM msg))
    (cl:cons ':FR (FR msg))
    (cl:cons ':MR (MR msg))
    (cl:cons ':RR (RR msg))
    (cl:cons ':RM (RM msg))
    (cl:cons ':RL (RL msg))
    (cl:cons ':ML (ML msg))
))
