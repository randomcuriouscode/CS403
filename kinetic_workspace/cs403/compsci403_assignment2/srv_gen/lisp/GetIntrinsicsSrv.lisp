; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment2-srv)


;//! \htmlinclude GetIntrinsicsSrv-request.msg.html

(cl:defclass <GetIntrinsicsSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetIntrinsicsSrv-request (<GetIntrinsicsSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntrinsicsSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntrinsicsSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment2-srv:<GetIntrinsicsSrv-request> is deprecated: use compsci403_assignment2-srv:GetIntrinsicsSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntrinsicsSrv-request>) ostream)
  "Serializes a message object of type '<GetIntrinsicsSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntrinsicsSrv-request>) istream)
  "Deserializes a message object of type '<GetIntrinsicsSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntrinsicsSrv-request>)))
  "Returns string type for a service object of type '<GetIntrinsicsSrv-request>"
  "compsci403_assignment2/GetIntrinsicsSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicsSrv-request)))
  "Returns string type for a service object of type 'GetIntrinsicsSrv-request"
  "compsci403_assignment2/GetIntrinsicsSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntrinsicsSrv-request>)))
  "Returns md5sum for a message object of type '<GetIntrinsicsSrv-request>"
  "41c1a913ab0768dc23cb054cfa8156c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntrinsicsSrv-request)))
  "Returns md5sum for a message object of type 'GetIntrinsicsSrv-request"
  "41c1a913ab0768dc23cb054cfa8156c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntrinsicsSrv-request>)))
  "Returns full string definition for message of type '<GetIntrinsicsSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntrinsicsSrv-request)))
  "Returns full string definition for message of type 'GetIntrinsicsSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntrinsicsSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntrinsicsSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntrinsicsSrv-request
))
;//! \htmlinclude GetIntrinsicsSrv-response.msg.html

(cl:defclass <GetIntrinsicsSrv-response> (roslisp-msg-protocol:ros-message)
  ((fx
    :reader fx
    :initarg :fx
    :type cl:float
    :initform 0.0)
   (fy
    :reader fy
    :initarg :fy
    :type cl:float
    :initform 0.0)
   (px
    :reader px
    :initarg :px
    :type cl:float
    :initform 0.0)
   (py
    :reader py
    :initarg :py
    :type cl:float
    :initform 0.0)
   (a
    :reader a
    :initarg :a
    :type cl:float
    :initform 0.0)
   (b
    :reader b
    :initarg :b
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetIntrinsicsSrv-response (<GetIntrinsicsSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntrinsicsSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntrinsicsSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment2-srv:<GetIntrinsicsSrv-response> is deprecated: use compsci403_assignment2-srv:GetIntrinsicsSrv-response instead.")))

(cl:ensure-generic-function 'fx-val :lambda-list '(m))
(cl:defmethod fx-val ((m <GetIntrinsicsSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:fx-val is deprecated.  Use compsci403_assignment2-srv:fx instead.")
  (fx m))

(cl:ensure-generic-function 'fy-val :lambda-list '(m))
(cl:defmethod fy-val ((m <GetIntrinsicsSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:fy-val is deprecated.  Use compsci403_assignment2-srv:fy instead.")
  (fy m))

(cl:ensure-generic-function 'px-val :lambda-list '(m))
(cl:defmethod px-val ((m <GetIntrinsicsSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:px-val is deprecated.  Use compsci403_assignment2-srv:px instead.")
  (px m))

(cl:ensure-generic-function 'py-val :lambda-list '(m))
(cl:defmethod py-val ((m <GetIntrinsicsSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:py-val is deprecated.  Use compsci403_assignment2-srv:py instead.")
  (py m))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <GetIntrinsicsSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:a-val is deprecated.  Use compsci403_assignment2-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <GetIntrinsicsSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:b-val is deprecated.  Use compsci403_assignment2-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntrinsicsSrv-response>) ostream)
  "Serializes a message object of type '<GetIntrinsicsSrv-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'py))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntrinsicsSrv-response>) istream)
  "Deserializes a message object of type '<GetIntrinsicsSrv-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'py) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntrinsicsSrv-response>)))
  "Returns string type for a service object of type '<GetIntrinsicsSrv-response>"
  "compsci403_assignment2/GetIntrinsicsSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicsSrv-response)))
  "Returns string type for a service object of type 'GetIntrinsicsSrv-response"
  "compsci403_assignment2/GetIntrinsicsSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntrinsicsSrv-response>)))
  "Returns md5sum for a message object of type '<GetIntrinsicsSrv-response>"
  "41c1a913ab0768dc23cb054cfa8156c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntrinsicsSrv-response)))
  "Returns md5sum for a message object of type 'GetIntrinsicsSrv-response"
  "41c1a913ab0768dc23cb054cfa8156c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntrinsicsSrv-response>)))
  "Returns full string definition for message of type '<GetIntrinsicsSrv-response>"
  (cl:format cl:nil "float32 fx~%float32 fy~%float32 px~%float32 py~%float32 a~%float32 b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntrinsicsSrv-response)))
  "Returns full string definition for message of type 'GetIntrinsicsSrv-response"
  (cl:format cl:nil "float32 fx~%float32 fy~%float32 px~%float32 py~%float32 a~%float32 b~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntrinsicsSrv-response>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntrinsicsSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntrinsicsSrv-response
    (cl:cons ':fx (fx msg))
    (cl:cons ':fy (fy msg))
    (cl:cons ':px (px msg))
    (cl:cons ':py (py msg))
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIntrinsicsSrv)))
  'GetIntrinsicsSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIntrinsicsSrv)))
  'GetIntrinsicsSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntrinsicsSrv)))
  "Returns string type for a service object of type '<GetIntrinsicsSrv>"
  "compsci403_assignment2/GetIntrinsicsSrv")