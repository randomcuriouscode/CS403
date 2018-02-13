; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment2-srv)


;//! \htmlinclude Get3DPointFromDepthSrv-request.msg.html

(cl:defclass <Get3DPointFromDepthSrv-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0)
   (fx
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
    :initform 0.0))
)

(cl:defclass Get3DPointFromDepthSrv-request (<Get3DPointFromDepthSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get3DPointFromDepthSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get3DPointFromDepthSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment2-srv:<Get3DPointFromDepthSrv-request> is deprecated: use compsci403_assignment2-srv:Get3DPointFromDepthSrv-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Get3DPointFromDepthSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:x-val is deprecated.  Use compsci403_assignment2-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Get3DPointFromDepthSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:y-val is deprecated.  Use compsci403_assignment2-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <Get3DPointFromDepthSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:depth-val is deprecated.  Use compsci403_assignment2-srv:depth instead.")
  (depth m))

(cl:ensure-generic-function 'fx-val :lambda-list '(m))
(cl:defmethod fx-val ((m <Get3DPointFromDepthSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:fx-val is deprecated.  Use compsci403_assignment2-srv:fx instead.")
  (fx m))

(cl:ensure-generic-function 'fy-val :lambda-list '(m))
(cl:defmethod fy-val ((m <Get3DPointFromDepthSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:fy-val is deprecated.  Use compsci403_assignment2-srv:fy instead.")
  (fy m))

(cl:ensure-generic-function 'px-val :lambda-list '(m))
(cl:defmethod px-val ((m <Get3DPointFromDepthSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:px-val is deprecated.  Use compsci403_assignment2-srv:px instead.")
  (px m))

(cl:ensure-generic-function 'py-val :lambda-list '(m))
(cl:defmethod py-val ((m <Get3DPointFromDepthSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:py-val is deprecated.  Use compsci403_assignment2-srv:py instead.")
  (py m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get3DPointFromDepthSrv-request>) ostream)
  "Serializes a message object of type '<Get3DPointFromDepthSrv-request>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get3DPointFromDepthSrv-request>) istream)
  "Deserializes a message object of type '<Get3DPointFromDepthSrv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-single-float-bits bits)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get3DPointFromDepthSrv-request>)))
  "Returns string type for a service object of type '<Get3DPointFromDepthSrv-request>"
  "compsci403_assignment2/Get3DPointFromDepthSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get3DPointFromDepthSrv-request)))
  "Returns string type for a service object of type 'Get3DPointFromDepthSrv-request"
  "compsci403_assignment2/Get3DPointFromDepthSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get3DPointFromDepthSrv-request>)))
  "Returns md5sum for a message object of type '<Get3DPointFromDepthSrv-request>"
  "fa92684c4d3e22b7bf75527d69a8eb73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get3DPointFromDepthSrv-request)))
  "Returns md5sum for a message object of type 'Get3DPointFromDepthSrv-request"
  "fa92684c4d3e22b7bf75527d69a8eb73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get3DPointFromDepthSrv-request>)))
  "Returns full string definition for message of type '<Get3DPointFromDepthSrv-request>"
  (cl:format cl:nil "int32 x~%int32 y~%float32 depth~%float32 fx~%float32 fy~%float32 px~%float32 py~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get3DPointFromDepthSrv-request)))
  "Returns full string definition for message of type 'Get3DPointFromDepthSrv-request"
  (cl:format cl:nil "int32 x~%int32 y~%float32 depth~%float32 fx~%float32 fy~%float32 px~%float32 py~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get3DPointFromDepthSrv-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get3DPointFromDepthSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Get3DPointFromDepthSrv-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':depth (depth msg))
    (cl:cons ':fx (fx msg))
    (cl:cons ':fy (fy msg))
    (cl:cons ':px (px msg))
    (cl:cons ':py (py msg))
))
;//! \htmlinclude Get3DPointFromDepthSrv-response.msg.html

(cl:defclass <Get3DPointFromDepthSrv-response> (roslisp-msg-protocol:ros-message)
  ((X
    :reader X
    :initarg :X
    :type cl:float
    :initform 0.0)
   (Y
    :reader Y
    :initarg :Y
    :type cl:float
    :initform 0.0)
   (Z
    :reader Z
    :initarg :Z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Get3DPointFromDepthSrv-response (<Get3DPointFromDepthSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get3DPointFromDepthSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get3DPointFromDepthSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment2-srv:<Get3DPointFromDepthSrv-response> is deprecated: use compsci403_assignment2-srv:Get3DPointFromDepthSrv-response instead.")))

(cl:ensure-generic-function 'X-val :lambda-list '(m))
(cl:defmethod X-val ((m <Get3DPointFromDepthSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:X-val is deprecated.  Use compsci403_assignment2-srv:X instead.")
  (X m))

(cl:ensure-generic-function 'Y-val :lambda-list '(m))
(cl:defmethod Y-val ((m <Get3DPointFromDepthSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:Y-val is deprecated.  Use compsci403_assignment2-srv:Y instead.")
  (Y m))

(cl:ensure-generic-function 'Z-val :lambda-list '(m))
(cl:defmethod Z-val ((m <Get3DPointFromDepthSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment2-srv:Z-val is deprecated.  Use compsci403_assignment2-srv:Z instead.")
  (Z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get3DPointFromDepthSrv-response>) ostream)
  "Serializes a message object of type '<Get3DPointFromDepthSrv-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'X))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get3DPointFromDepthSrv-response>) istream)
  "Deserializes a message object of type '<Get3DPointFromDepthSrv-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get3DPointFromDepthSrv-response>)))
  "Returns string type for a service object of type '<Get3DPointFromDepthSrv-response>"
  "compsci403_assignment2/Get3DPointFromDepthSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get3DPointFromDepthSrv-response)))
  "Returns string type for a service object of type 'Get3DPointFromDepthSrv-response"
  "compsci403_assignment2/Get3DPointFromDepthSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get3DPointFromDepthSrv-response>)))
  "Returns md5sum for a message object of type '<Get3DPointFromDepthSrv-response>"
  "fa92684c4d3e22b7bf75527d69a8eb73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get3DPointFromDepthSrv-response)))
  "Returns md5sum for a message object of type 'Get3DPointFromDepthSrv-response"
  "fa92684c4d3e22b7bf75527d69a8eb73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get3DPointFromDepthSrv-response>)))
  "Returns full string definition for message of type '<Get3DPointFromDepthSrv-response>"
  (cl:format cl:nil "float32 X~%float32 Y~%float32 Z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get3DPointFromDepthSrv-response)))
  "Returns full string definition for message of type 'Get3DPointFromDepthSrv-response"
  (cl:format cl:nil "float32 X~%float32 Y~%float32 Z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get3DPointFromDepthSrv-response>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get3DPointFromDepthSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Get3DPointFromDepthSrv-response
    (cl:cons ':X (X msg))
    (cl:cons ':Y (Y msg))
    (cl:cons ':Z (Z msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Get3DPointFromDepthSrv)))
  'Get3DPointFromDepthSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Get3DPointFromDepthSrv)))
  'Get3DPointFromDepthSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get3DPointFromDepthSrv)))
  "Returns string type for a service object of type '<Get3DPointFromDepthSrv>"
  "compsci403_assignment2/Get3DPointFromDepthSrv")