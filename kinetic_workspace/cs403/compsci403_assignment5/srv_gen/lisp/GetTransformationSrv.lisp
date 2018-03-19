; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment5-srv)


;//! \htmlinclude GetTransformationSrv-request.msg.html

(cl:defclass <GetTransformationSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetTransformationSrv-request (<GetTransformationSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTransformationSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTransformationSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment5-srv:<GetTransformationSrv-request> is deprecated: use compsci403_assignment5-srv:GetTransformationSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTransformationSrv-request>) ostream)
  "Serializes a message object of type '<GetTransformationSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTransformationSrv-request>) istream)
  "Deserializes a message object of type '<GetTransformationSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTransformationSrv-request>)))
  "Returns string type for a service object of type '<GetTransformationSrv-request>"
  "compsci403_assignment5/GetTransformationSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTransformationSrv-request)))
  "Returns string type for a service object of type 'GetTransformationSrv-request"
  "compsci403_assignment5/GetTransformationSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTransformationSrv-request>)))
  "Returns md5sum for a message object of type '<GetTransformationSrv-request>"
  "62d96f1a31dbc311ad4b59c90d485f6e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTransformationSrv-request)))
  "Returns md5sum for a message object of type 'GetTransformationSrv-request"
  "62d96f1a31dbc311ad4b59c90d485f6e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTransformationSrv-request>)))
  "Returns full string definition for message of type '<GetTransformationSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTransformationSrv-request)))
  "Returns full string definition for message of type 'GetTransformationSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTransformationSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTransformationSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTransformationSrv-request
))
;//! \htmlinclude GetTransformationSrv-response.msg.html

(cl:defclass <GetTransformationSrv-response> (roslisp-msg-protocol:ros-message)
  ((R
    :reader R
    :initarg :R
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (T
    :reader T
    :initarg :T
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass GetTransformationSrv-response (<GetTransformationSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetTransformationSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetTransformationSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment5-srv:<GetTransformationSrv-response> is deprecated: use compsci403_assignment5-srv:GetTransformationSrv-response instead.")))

(cl:ensure-generic-function 'R-val :lambda-list '(m))
(cl:defmethod R-val ((m <GetTransformationSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment5-srv:R-val is deprecated.  Use compsci403_assignment5-srv:R instead.")
  (R m))

(cl:ensure-generic-function 'T-val :lambda-list '(m))
(cl:defmethod T-val ((m <GetTransformationSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment5-srv:T-val is deprecated.  Use compsci403_assignment5-srv:T instead.")
  (T m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetTransformationSrv-response>) ostream)
  "Serializes a message object of type '<GetTransformationSrv-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'R))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'T) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetTransformationSrv-response>) istream)
  "Deserializes a message object of type '<GetTransformationSrv-response>"
  (cl:setf (cl:slot-value msg 'R) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'R)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'T) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetTransformationSrv-response>)))
  "Returns string type for a service object of type '<GetTransformationSrv-response>"
  "compsci403_assignment5/GetTransformationSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTransformationSrv-response)))
  "Returns string type for a service object of type 'GetTransformationSrv-response"
  "compsci403_assignment5/GetTransformationSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetTransformationSrv-response>)))
  "Returns md5sum for a message object of type '<GetTransformationSrv-response>"
  "62d96f1a31dbc311ad4b59c90d485f6e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetTransformationSrv-response)))
  "Returns md5sum for a message object of type 'GetTransformationSrv-response"
  "62d96f1a31dbc311ad4b59c90d485f6e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetTransformationSrv-response>)))
  "Returns full string definition for message of type '<GetTransformationSrv-response>"
  (cl:format cl:nil "float32[9] R~%geometry_msgs/Point32 T~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetTransformationSrv-response)))
  "Returns full string definition for message of type 'GetTransformationSrv-response"
  (cl:format cl:nil "float32[9] R~%geometry_msgs/Point32 T~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetTransformationSrv-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'R) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'T))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetTransformationSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetTransformationSrv-response
    (cl:cons ':R (R msg))
    (cl:cons ':T (T msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetTransformationSrv)))
  'GetTransformationSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetTransformationSrv)))
  'GetTransformationSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetTransformationSrv)))
  "Returns string type for a service object of type '<GetTransformationSrv>"
  "compsci403_assignment5/GetTransformationSrv")