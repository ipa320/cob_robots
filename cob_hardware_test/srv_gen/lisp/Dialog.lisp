; Auto-generated. Do not edit!


(cl:in-package cob_hardware_test-srv)


;//! \htmlinclude Dialog-request.msg.html

(cl:defclass <Dialog-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass Dialog-request (<Dialog-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dialog-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dialog-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_hardware_test-srv:<Dialog-request> is deprecated: use cob_hardware_test-srv:Dialog-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Dialog-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hardware_test-srv:type-val is deprecated.  Use cob_hardware_test-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <Dialog-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hardware_test-srv:message-val is deprecated.  Use cob_hardware_test-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dialog-request>) ostream)
  "Serializes a message object of type '<Dialog-request>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dialog-request>) istream)
  "Deserializes a message object of type '<Dialog-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dialog-request>)))
  "Returns string type for a service object of type '<Dialog-request>"
  "cob_hardware_test/DialogRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dialog-request)))
  "Returns string type for a service object of type 'Dialog-request"
  "cob_hardware_test/DialogRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dialog-request>)))
  "Returns md5sum for a message object of type '<Dialog-request>"
  "a01bcfd0fcb73d1ce9468534690c8d34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dialog-request)))
  "Returns md5sum for a message object of type 'Dialog-request"
  "a01bcfd0fcb73d1ce9468534690c8d34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dialog-request>)))
  "Returns full string definition for message of type '<Dialog-request>"
  (cl:format cl:nil "int8 type~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dialog-request)))
  "Returns full string definition for message of type 'Dialog-request"
  (cl:format cl:nil "int8 type~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dialog-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dialog-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Dialog-request
    (cl:cons ':type (type msg))
    (cl:cons ':message (message msg))
))
;//! \htmlinclude Dialog-response.msg.html

(cl:defclass <Dialog-response> (roslisp-msg-protocol:ros-message)
  ((answer
    :reader answer
    :initarg :answer
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Dialog-response (<Dialog-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dialog-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dialog-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_hardware_test-srv:<Dialog-response> is deprecated: use cob_hardware_test-srv:Dialog-response instead.")))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <Dialog-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_hardware_test-srv:answer-val is deprecated.  Use cob_hardware_test-srv:answer instead.")
  (answer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dialog-response>) ostream)
  "Serializes a message object of type '<Dialog-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'answer) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dialog-response>) istream)
  "Deserializes a message object of type '<Dialog-response>"
    (cl:setf (cl:slot-value msg 'answer) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dialog-response>)))
  "Returns string type for a service object of type '<Dialog-response>"
  "cob_hardware_test/DialogResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dialog-response)))
  "Returns string type for a service object of type 'Dialog-response"
  "cob_hardware_test/DialogResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dialog-response>)))
  "Returns md5sum for a message object of type '<Dialog-response>"
  "a01bcfd0fcb73d1ce9468534690c8d34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dialog-response)))
  "Returns md5sum for a message object of type 'Dialog-response"
  "a01bcfd0fcb73d1ce9468534690c8d34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dialog-response>)))
  "Returns full string definition for message of type '<Dialog-response>"
  (cl:format cl:nil "bool answer~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dialog-response)))
  "Returns full string definition for message of type 'Dialog-response"
  (cl:format cl:nil "bool answer~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dialog-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dialog-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Dialog-response
    (cl:cons ':answer (answer msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Dialog)))
  'Dialog-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Dialog)))
  'Dialog-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dialog)))
  "Returns string type for a service object of type '<Dialog>"
  "cob_hardware_test/Dialog")