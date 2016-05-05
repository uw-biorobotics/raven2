; Auto-generated. Do not edit!


(cl:in-package raven_2-msg)


;//! \htmlinclude raven_automove.msg.html

(cl:defclass <raven_automove> (roslisp-msg-protocol:ros-message)
  ((hdr
    :reader hdr
    :initarg :hdr
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (del_pos
    :reader del_pos
    :initarg :del_pos
    :type (cl:vector cl:integer)
   :initform (cl:make-array 6 :element-type 'cl:integer :initial-element 0))
   (tf_incr
    :reader tf_incr
    :initarg :tf_incr
    :type (cl:vector geometry_msgs-msg:Transform)
   :initform (cl:make-array 2 :element-type 'geometry_msgs-msg:Transform :initial-element (cl:make-instance 'geometry_msgs-msg:Transform))))
)

(cl:defclass raven_automove (<raven_automove>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <raven_automove>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'raven_automove)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raven_2-msg:<raven_automove> is deprecated: use raven_2-msg:raven_automove instead.")))

(cl:ensure-generic-function 'hdr-val :lambda-list '(m))
(cl:defmethod hdr-val ((m <raven_automove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raven_2-msg:hdr-val is deprecated.  Use raven_2-msg:hdr instead.")
  (hdr m))

(cl:ensure-generic-function 'del_pos-val :lambda-list '(m))
(cl:defmethod del_pos-val ((m <raven_automove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raven_2-msg:del_pos-val is deprecated.  Use raven_2-msg:del_pos instead.")
  (del_pos m))

(cl:ensure-generic-function 'tf_incr-val :lambda-list '(m))
(cl:defmethod tf_incr-val ((m <raven_automove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raven_2-msg:tf_incr-val is deprecated.  Use raven_2-msg:tf_incr instead.")
  (tf_incr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <raven_automove>) ostream)
  "Serializes a message object of type '<raven_automove>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hdr) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'del_pos))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tf_incr))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <raven_automove>) istream)
  "Deserializes a message object of type '<raven_automove>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hdr) istream)
  (cl:setf (cl:slot-value msg 'del_pos) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'del_pos)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'tf_incr) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'tf_incr)))
    (cl:dotimes (i 2)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Transform))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<raven_automove>)))
  "Returns string type for a message object of type '<raven_automove>"
  "raven_2/raven_automove")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'raven_automove)))
  "Returns string type for a message object of type 'raven_automove"
  "raven_2/raven_automove")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<raven_automove>)))
  "Returns md5sum for a message object of type '<raven_automove>"
  "409f695c901a20d6a1f90a4e55ae6f63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'raven_automove)))
  "Returns md5sum for a message object of type 'raven_automove"
  "409f695c901a20d6a1f90a4e55ae6f63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<raven_automove>)))
  "Returns full string definition for message of type '<raven_automove>"
  (cl:format cl:nil "Header      hdr~%int32[6]    del_pos~%geometry_msgs/Transform[2] tf_incr~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'raven_automove)))
  "Returns full string definition for message of type 'raven_automove"
  (cl:format cl:nil "Header      hdr~%int32[6]    del_pos~%geometry_msgs/Transform[2] tf_incr~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <raven_automove>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hdr))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'del_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'tf_incr) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <raven_automove>))
  "Converts a ROS message object to a list"
  (cl:list 'raven_automove
    (cl:cons ':hdr (hdr msg))
    (cl:cons ':del_pos (del_pos msg))
    (cl:cons ':tf_incr (tf_incr msg))
))
