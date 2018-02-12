; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment1-srv)


;//! \htmlinclude Assignment1CallerSrv-request.msg.html

(cl:defclass <Assignment1CallerSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Assignment1CallerSrv-request (<Assignment1CallerSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Assignment1CallerSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Assignment1CallerSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment1-srv:<Assignment1CallerSrv-request> is deprecated: use compsci403_assignment1-srv:Assignment1CallerSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Assignment1CallerSrv-request>) ostream)
  "Serializes a message object of type '<Assignment1CallerSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Assignment1CallerSrv-request>) istream)
  "Deserializes a message object of type '<Assignment1CallerSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Assignment1CallerSrv-request>)))
  "Returns string type for a service object of type '<Assignment1CallerSrv-request>"
  "compsci403_assignment1/Assignment1CallerSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Assignment1CallerSrv-request)))
  "Returns string type for a service object of type 'Assignment1CallerSrv-request"
  "compsci403_assignment1/Assignment1CallerSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Assignment1CallerSrv-request>)))
  "Returns md5sum for a message object of type '<Assignment1CallerSrv-request>"
  "994972b6e03928b2476860ce6c4c8e17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Assignment1CallerSrv-request)))
  "Returns md5sum for a message object of type 'Assignment1CallerSrv-request"
  "994972b6e03928b2476860ce6c4c8e17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Assignment1CallerSrv-request>)))
  "Returns full string definition for message of type '<Assignment1CallerSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Assignment1CallerSrv-request)))
  "Returns full string definition for message of type 'Assignment1CallerSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Assignment1CallerSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Assignment1CallerSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Assignment1CallerSrv-request
))
;//! \htmlinclude Assignment1CallerSrv-response.msg.html

(cl:defclass <Assignment1CallerSrv-response> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform ""))
)

(cl:defclass Assignment1CallerSrv-response (<Assignment1CallerSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Assignment1CallerSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Assignment1CallerSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment1-srv:<Assignment1CallerSrv-response> is deprecated: use compsci403_assignment1-srv:Assignment1CallerSrv-response instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <Assignment1CallerSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment1-srv:str-val is deprecated.  Use compsci403_assignment1-srv:str instead.")
  (str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Assignment1CallerSrv-response>) ostream)
  "Serializes a message object of type '<Assignment1CallerSrv-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Assignment1CallerSrv-response>) istream)
  "Deserializes a message object of type '<Assignment1CallerSrv-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Assignment1CallerSrv-response>)))
  "Returns string type for a service object of type '<Assignment1CallerSrv-response>"
  "compsci403_assignment1/Assignment1CallerSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Assignment1CallerSrv-response)))
  "Returns string type for a service object of type 'Assignment1CallerSrv-response"
  "compsci403_assignment1/Assignment1CallerSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Assignment1CallerSrv-response>)))
  "Returns md5sum for a message object of type '<Assignment1CallerSrv-response>"
  "994972b6e03928b2476860ce6c4c8e17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Assignment1CallerSrv-response)))
  "Returns md5sum for a message object of type 'Assignment1CallerSrv-response"
  "994972b6e03928b2476860ce6c4c8e17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Assignment1CallerSrv-response>)))
  "Returns full string definition for message of type '<Assignment1CallerSrv-response>"
  (cl:format cl:nil "string str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Assignment1CallerSrv-response)))
  "Returns full string definition for message of type 'Assignment1CallerSrv-response"
  (cl:format cl:nil "string str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Assignment1CallerSrv-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Assignment1CallerSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Assignment1CallerSrv-response
    (cl:cons ':str (str msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Assignment1CallerSrv)))
  'Assignment1CallerSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Assignment1CallerSrv)))
  'Assignment1CallerSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Assignment1CallerSrv)))
  "Returns string type for a service object of type '<Assignment1CallerSrv>"
  "compsci403_assignment1/Assignment1CallerSrv")