; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment1-srv)


;//! \htmlinclude Assignment1ProviderSrv-request.msg.html

(cl:defclass <Assignment1ProviderSrv-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Assignment1ProviderSrv-request (<Assignment1ProviderSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Assignment1ProviderSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Assignment1ProviderSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment1-srv:<Assignment1ProviderSrv-request> is deprecated: use compsci403_assignment1-srv:Assignment1ProviderSrv-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Assignment1ProviderSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment1-srv:x-val is deprecated.  Use compsci403_assignment1-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Assignment1ProviderSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment1-srv:y-val is deprecated.  Use compsci403_assignment1-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Assignment1ProviderSrv-request>) ostream)
  "Serializes a message object of type '<Assignment1ProviderSrv-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Assignment1ProviderSrv-request>) istream)
  "Deserializes a message object of type '<Assignment1ProviderSrv-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Assignment1ProviderSrv-request>)))
  "Returns string type for a service object of type '<Assignment1ProviderSrv-request>"
  "compsci403_assignment1/Assignment1ProviderSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Assignment1ProviderSrv-request)))
  "Returns string type for a service object of type 'Assignment1ProviderSrv-request"
  "compsci403_assignment1/Assignment1ProviderSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Assignment1ProviderSrv-request>)))
  "Returns md5sum for a message object of type '<Assignment1ProviderSrv-request>"
  "5e4d1ad141c5f278e394a7614ddc18cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Assignment1ProviderSrv-request)))
  "Returns md5sum for a message object of type 'Assignment1ProviderSrv-request"
  "5e4d1ad141c5f278e394a7614ddc18cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Assignment1ProviderSrv-request>)))
  "Returns full string definition for message of type '<Assignment1ProviderSrv-request>"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Assignment1ProviderSrv-request)))
  "Returns full string definition for message of type 'Assignment1ProviderSrv-request"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Assignment1ProviderSrv-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Assignment1ProviderSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Assignment1ProviderSrv-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
;//! \htmlinclude Assignment1ProviderSrv-response.msg.html

(cl:defclass <Assignment1ProviderSrv-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:float
    :initform 0.0))
)

(cl:defclass Assignment1ProviderSrv-response (<Assignment1ProviderSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Assignment1ProviderSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Assignment1ProviderSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment1-srv:<Assignment1ProviderSrv-response> is deprecated: use compsci403_assignment1-srv:Assignment1ProviderSrv-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Assignment1ProviderSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment1-srv:result-val is deprecated.  Use compsci403_assignment1-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Assignment1ProviderSrv-response>) ostream)
  "Serializes a message object of type '<Assignment1ProviderSrv-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Assignment1ProviderSrv-response>) istream)
  "Deserializes a message object of type '<Assignment1ProviderSrv-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'result) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Assignment1ProviderSrv-response>)))
  "Returns string type for a service object of type '<Assignment1ProviderSrv-response>"
  "compsci403_assignment1/Assignment1ProviderSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Assignment1ProviderSrv-response)))
  "Returns string type for a service object of type 'Assignment1ProviderSrv-response"
  "compsci403_assignment1/Assignment1ProviderSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Assignment1ProviderSrv-response>)))
  "Returns md5sum for a message object of type '<Assignment1ProviderSrv-response>"
  "5e4d1ad141c5f278e394a7614ddc18cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Assignment1ProviderSrv-response)))
  "Returns md5sum for a message object of type 'Assignment1ProviderSrv-response"
  "5e4d1ad141c5f278e394a7614ddc18cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Assignment1ProviderSrv-response>)))
  "Returns full string definition for message of type '<Assignment1ProviderSrv-response>"
  (cl:format cl:nil "float32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Assignment1ProviderSrv-response)))
  "Returns full string definition for message of type 'Assignment1ProviderSrv-response"
  (cl:format cl:nil "float32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Assignment1ProviderSrv-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Assignment1ProviderSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Assignment1ProviderSrv-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Assignment1ProviderSrv)))
  'Assignment1ProviderSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Assignment1ProviderSrv)))
  'Assignment1ProviderSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Assignment1ProviderSrv)))
  "Returns string type for a service object of type '<Assignment1ProviderSrv>"
  "compsci403_assignment1/Assignment1ProviderSrv")