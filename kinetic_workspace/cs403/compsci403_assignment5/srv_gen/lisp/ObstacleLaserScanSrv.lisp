; Auto-generated. Do not edit!


(cl:in-package compsci403_assignment5-srv)


;//! \htmlinclude ObstacleLaserScanSrv-request.msg.html

(cl:defclass <ObstacleLaserScanSrv-request> (roslisp-msg-protocol:ros-message)
  ((S
    :reader S
    :initarg :S
    :type sensor_msgs-msg:LaserScan
    :initform (cl:make-instance 'sensor_msgs-msg:LaserScan))
   (R
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

(cl:defclass ObstacleLaserScanSrv-request (<ObstacleLaserScanSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleLaserScanSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleLaserScanSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment5-srv:<ObstacleLaserScanSrv-request> is deprecated: use compsci403_assignment5-srv:ObstacleLaserScanSrv-request instead.")))

(cl:ensure-generic-function 'S-val :lambda-list '(m))
(cl:defmethod S-val ((m <ObstacleLaserScanSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment5-srv:S-val is deprecated.  Use compsci403_assignment5-srv:S instead.")
  (S m))

(cl:ensure-generic-function 'R-val :lambda-list '(m))
(cl:defmethod R-val ((m <ObstacleLaserScanSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment5-srv:R-val is deprecated.  Use compsci403_assignment5-srv:R instead.")
  (R m))

(cl:ensure-generic-function 'T-val :lambda-list '(m))
(cl:defmethod T-val ((m <ObstacleLaserScanSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment5-srv:T-val is deprecated.  Use compsci403_assignment5-srv:T instead.")
  (T m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleLaserScanSrv-request>) ostream)
  "Serializes a message object of type '<ObstacleLaserScanSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'S) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'R))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'T) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleLaserScanSrv-request>) istream)
  "Deserializes a message object of type '<ObstacleLaserScanSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'S) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleLaserScanSrv-request>)))
  "Returns string type for a service object of type '<ObstacleLaserScanSrv-request>"
  "compsci403_assignment5/ObstacleLaserScanSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleLaserScanSrv-request)))
  "Returns string type for a service object of type 'ObstacleLaserScanSrv-request"
  "compsci403_assignment5/ObstacleLaserScanSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleLaserScanSrv-request>)))
  "Returns md5sum for a message object of type '<ObstacleLaserScanSrv-request>"
  "a44379f4c79ce76e70a13a28ae6b48f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleLaserScanSrv-request)))
  "Returns md5sum for a message object of type 'ObstacleLaserScanSrv-request"
  "a44379f4c79ce76e70a13a28ae6b48f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleLaserScanSrv-request>)))
  "Returns full string definition for message of type '<ObstacleLaserScanSrv-request>"
  (cl:format cl:nil "sensor_msgs/LaserScan S~%float32[9] R~%geometry_msgs/Point32 T~%~%================================================================================~%MSG: sensor_msgs/LaserScan~%# Single scan from a planar laser range-finder~%#~%# If you have another ranging device with different behavior (e.g. a sonar~%# array), please find or create a different message, since applications~%# will make fairly laser-specific assumptions about this data~%~%Header header            # timestamp in the header is the acquisition time of ~%                         # the first ray in the scan.~%                         #~%                         # in frame frame_id, angles are measured around ~%                         # the positive Z axis (counterclockwise, if Z is up)~%                         # with zero angle being forward along the x axis~%                         ~%float32 angle_min        # start angle of the scan [rad]~%float32 angle_max        # end angle of the scan [rad]~%float32 angle_increment  # angular distance between measurements [rad]~%~%float32 time_increment   # time between measurements [seconds] - if your scanner~%                         # is moving, this will be used in interpolating position~%                         # of 3d points~%float32 scan_time        # time between scans [seconds]~%~%float32 range_min        # minimum range value [m]~%float32 range_max        # maximum range value [m]~%~%float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)~%float32[] intensities    # intensity data [device-specific units].  If your~%                         # device does not provide intensities, please leave~%                         # the array empty.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleLaserScanSrv-request)))
  "Returns full string definition for message of type 'ObstacleLaserScanSrv-request"
  (cl:format cl:nil "sensor_msgs/LaserScan S~%float32[9] R~%geometry_msgs/Point32 T~%~%================================================================================~%MSG: sensor_msgs/LaserScan~%# Single scan from a planar laser range-finder~%#~%# If you have another ranging device with different behavior (e.g. a sonar~%# array), please find or create a different message, since applications~%# will make fairly laser-specific assumptions about this data~%~%Header header            # timestamp in the header is the acquisition time of ~%                         # the first ray in the scan.~%                         #~%                         # in frame frame_id, angles are measured around ~%                         # the positive Z axis (counterclockwise, if Z is up)~%                         # with zero angle being forward along the x axis~%                         ~%float32 angle_min        # start angle of the scan [rad]~%float32 angle_max        # end angle of the scan [rad]~%float32 angle_increment  # angular distance between measurements [rad]~%~%float32 time_increment   # time between measurements [seconds] - if your scanner~%                         # is moving, this will be used in interpolating position~%                         # of 3d points~%float32 scan_time        # time between scans [seconds]~%~%float32 range_min        # minimum range value [m]~%float32 range_max        # maximum range value [m]~%~%float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)~%float32[] intensities    # intensity data [device-specific units].  If your~%                         # device does not provide intensities, please leave~%                         # the array empty.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleLaserScanSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'S))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'R) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'T))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleLaserScanSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleLaserScanSrv-request
    (cl:cons ':S (S msg))
    (cl:cons ':R (R msg))
    (cl:cons ':T (T msg))
))
;//! \htmlinclude ObstacleLaserScanSrv-response.msg.html

(cl:defclass <ObstacleLaserScanSrv-response> (roslisp-msg-protocol:ros-message)
  ((S_prime
    :reader S_prime
    :initarg :S_prime
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass ObstacleLaserScanSrv-response (<ObstacleLaserScanSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleLaserScanSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleLaserScanSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name compsci403_assignment5-srv:<ObstacleLaserScanSrv-response> is deprecated: use compsci403_assignment5-srv:ObstacleLaserScanSrv-response instead.")))

(cl:ensure-generic-function 'S_prime-val :lambda-list '(m))
(cl:defmethod S_prime-val ((m <ObstacleLaserScanSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader compsci403_assignment5-srv:S_prime-val is deprecated.  Use compsci403_assignment5-srv:S_prime instead.")
  (S_prime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleLaserScanSrv-response>) ostream)
  "Serializes a message object of type '<ObstacleLaserScanSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'S_prime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'S_prime))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleLaserScanSrv-response>) istream)
  "Deserializes a message object of type '<ObstacleLaserScanSrv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'S_prime) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'S_prime)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleLaserScanSrv-response>)))
  "Returns string type for a service object of type '<ObstacleLaserScanSrv-response>"
  "compsci403_assignment5/ObstacleLaserScanSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleLaserScanSrv-response)))
  "Returns string type for a service object of type 'ObstacleLaserScanSrv-response"
  "compsci403_assignment5/ObstacleLaserScanSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleLaserScanSrv-response>)))
  "Returns md5sum for a message object of type '<ObstacleLaserScanSrv-response>"
  "a44379f4c79ce76e70a13a28ae6b48f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleLaserScanSrv-response)))
  "Returns md5sum for a message object of type 'ObstacleLaserScanSrv-response"
  "a44379f4c79ce76e70a13a28ae6b48f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleLaserScanSrv-response>)))
  "Returns full string definition for message of type '<ObstacleLaserScanSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point32[] S_prime~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleLaserScanSrv-response)))
  "Returns full string definition for message of type 'ObstacleLaserScanSrv-response"
  (cl:format cl:nil "geometry_msgs/Point32[] S_prime~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleLaserScanSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'S_prime) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleLaserScanSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleLaserScanSrv-response
    (cl:cons ':S_prime (S_prime msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ObstacleLaserScanSrv)))
  'ObstacleLaserScanSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ObstacleLaserScanSrv)))
  'ObstacleLaserScanSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleLaserScanSrv)))
  "Returns string type for a service object of type '<ObstacleLaserScanSrv>"
  "compsci403_assignment5/ObstacleLaserScanSrv")