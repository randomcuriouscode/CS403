; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment2-srv)


;//! \htmlinclude GetDepthFromDisparitySrv-request.msg.html

(cl:defclass <GetDepthFromDisparitySrv-request> (roslisp-msg-protocol:ros-message)
  ((disparity
    :reader disparity
    :initarg :disparity
    :type cl:integer
    :initform 0)
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

(cl:defclass GetDepthFromDisparitySrv-request (<GetDepthFromDisparitySrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDepthFromDisparitySrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDepthFromDisparitySrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment2-srv:<GetDepthFromDisparitySrv-request> is deprecated: use compsci403_assignment2-srv:GetDepthFromDisparitySrv-request instead.")))

(cl:ensure-generic-function 'disparity-val :lambda-list '(m))
(cl:defmethod disparity-val ((m <GetDepthFromDisparitySrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:disparity-val is deprecated.  Use compsci403_assignment2-srv:disparity instead.")
  (disparity m))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <GetDepthFromDisparitySrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:a-val is deprecated.  Use compsci403_assignment2-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <GetDepthFromDisparitySrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:b-val is deprecated.  Use compsci403_assignment2-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDepthFromDisparitySrv-request>) ostream)
  "Serializes a message object of type '<GetDepthFromDisparitySrv-request>"
  (cl:let* ((signed (cl:slot-value msg 'disparity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDepthFromDisparitySrv-request>) istream)
  "Deserializes a message object of type '<GetDepthFromDisparitySrv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'disparity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDepthFromDisparitySrv-request>)))
  "Returns string type for a service object of type '<GetDepthFromDisparitySrv-request>"
  "compsci403_assignment2/GetDepthFromDisparitySrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDepthFromDisparitySrv-request)))
  "Returns string type for a service object of type 'GetDepthFromDisparitySrv-request"
  "compsci403_assignment2/GetDepthFromDisparitySrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDepthFromDisparitySrv-request>)))
  "Returns md5sum for a message object of type '<GetDepthFromDisparitySrv-request>"
  "4eb644ec455b55a777cc3c887868dbf9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDepthFromDisparitySrv-request)))
  "Returns md5sum for a message object of type 'GetDepthFromDisparitySrv-request"
  "4eb644ec455b55a777cc3c887868dbf9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDepthFromDisparitySrv-request>)))
  "Returns full string definition for message of type '<GetDepthFromDisparitySrv-request>"
  (cl:format cl:nil "int32 disparity~%float32 a~%float32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDepthFromDisparitySrv-request)))
  "Returns full string definition for message of type 'GetDepthFromDisparitySrv-request"
  (cl:format cl:nil "int32 disparity~%float32 a~%float32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDepthFromDisparitySrv-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDepthFromDisparitySrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDepthFromDisparitySrv-request
    (cl:cons ':disparity (disparity msg))
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude GetDepthFromDisparitySrv-response.msg.html

(cl:defclass <GetDepthFromDisparitySrv-response> (roslisp-msg-protocol:ros-message)
  ((depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetDepthFromDisparitySrv-response (<GetDepthFromDisparitySrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDepthFromDisparitySrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDepthFromDisparitySrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment2-srv:<GetDepthFromDisparitySrv-response> is deprecated: use compsci403_assignment2-srv:GetDepthFromDisparitySrv-response instead.")))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <GetDepthFromDisparitySrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:depth-val is deprecated.  Use compsci403_assignment2-srv:depth instead.")
  (depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDepthFromDisparitySrv-response>) ostream)
  "Serializes a message object of type '<GetDepthFromDisparitySrv-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDepthFromDisparitySrv-response>) istream)
  "Deserializes a message object of type '<GetDepthFromDisparitySrv-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDepthFromDisparitySrv-response>)))
  "Returns string type for a service object of type '<GetDepthFromDisparitySrv-response>"
  "compsci403_assignment2/GetDepthFromDisparitySrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDepthFromDisparitySrv-response)))
  "Returns string type for a service object of type 'GetDepthFromDisparitySrv-response"
  "compsci403_assignment2/GetDepthFromDisparitySrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDepthFromDisparitySrv-response>)))
  "Returns md5sum for a message object of type '<GetDepthFromDisparitySrv-response>"
  "4eb644ec455b55a777cc3c887868dbf9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDepthFromDisparitySrv-response)))
  "Returns md5sum for a message object of type 'GetDepthFromDisparitySrv-response"
  "4eb644ec455b55a777cc3c887868dbf9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDepthFromDisparitySrv-response>)))
  "Returns full string definition for message of type '<GetDepthFromDisparitySrv-response>"
  (cl:format cl:nil "float32 depth~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDepthFromDisparitySrv-response)))
  "Returns full string definition for message of type 'GetDepthFromDisparitySrv-response"
  (cl:format cl:nil "float32 depth~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDepthFromDisparitySrv-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDepthFromDisparitySrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDepthFromDisparitySrv-response
    (cl:cons ':depth (depth msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDepthFromDisparitySrv)))
  'GetDepthFromDisparitySrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDepthFromDisparitySrv)))
  'GetDepthFromDisparitySrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDepthFromDisparitySrv)))
  "Returns string type for a service object of type '<GetDepthFromDisparitySrv>"
  "compsci403_assignment2/GetDepthFromDisparitySrv")