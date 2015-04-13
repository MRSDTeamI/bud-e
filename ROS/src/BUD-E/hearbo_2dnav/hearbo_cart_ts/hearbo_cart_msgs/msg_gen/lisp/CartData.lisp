; Auto-generated. Do not edit!


(cl:in-package hearbo_cart_msgs-msg)


;//! \htmlinclude CartData.msg.html

(cl:defclass <CartData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ch
    :reader ch
    :initarg :ch
    :type (cl:vector hearbo_cart_msgs-msg:EncVal)
   :initform (cl:make-array 2 :element-type 'hearbo_cart_msgs-msg:EncVal :initial-element (cl:make-instance 'hearbo_cart_msgs-msg:EncVal))))
)

(cl:defclass CartData (<CartData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hearbo_cart_msgs-msg:<CartData> is deprecated: use hearbo_cart_msgs-msg:CartData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CartData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:header-val is deprecated.  Use hearbo_cart_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ch-val :lambda-list '(m))
(cl:defmethod ch-val ((m <CartData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:ch-val is deprecated.  Use hearbo_cart_msgs-msg:ch instead.")
  (ch m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartData>) ostream)
  "Serializes a message object of type '<CartData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ch))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartData>) istream)
  "Deserializes a message object of type '<CartData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'ch) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'ch)))
    (cl:dotimes (i 2)
    (cl:setf (cl:aref vals i) (cl:make-instance 'hearbo_cart_msgs-msg:EncVal))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartData>)))
  "Returns string type for a message object of type '<CartData>"
  "hearbo_cart_msgs/CartData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartData)))
  "Returns string type for a message object of type 'CartData"
  "hearbo_cart_msgs/CartData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartData>)))
  "Returns md5sum for a message object of type '<CartData>"
  "321b938e799bcee6695873279fe47f5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartData)))
  "Returns md5sum for a message object of type 'CartData"
  "321b938e799bcee6695873279fe47f5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartData>)))
  "Returns full string definition for message of type '<CartData>"
  (cl:format cl:nil "#~%# Received Data from the iXs cart~%#~%# header : time stamp and frame id~%# ch[0] : Encoder values of the driving wheels~%# ch[1] : Encoder values of the steering motor~%~%Header header~%EncVal[2] ch~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hearbo_cart_msgs/EncVal~%# Encoder value for all the wheels of the iXs cart~%# data[0] : ID = 0 : Left  forward wheel~%# data[1] : ID = 1 : Left  back    wheel~%# data[2] : ID = 2 : Right back    wheel~%# data[3] : ID = 3 : Right forward wheel~%~%uint32[4] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartData)))
  "Returns full string definition for message of type 'CartData"
  (cl:format cl:nil "#~%# Received Data from the iXs cart~%#~%# header : time stamp and frame id~%# ch[0] : Encoder values of the driving wheels~%# ch[1] : Encoder values of the steering motor~%~%Header header~%EncVal[2] ch~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: hearbo_cart_msgs/EncVal~%# Encoder value for all the wheels of the iXs cart~%# data[0] : ID = 0 : Left  forward wheel~%# data[1] : ID = 1 : Left  back    wheel~%# data[2] : ID = 2 : Right back    wheel~%# data[3] : ID = 3 : Right forward wheel~%~%uint32[4] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'ch) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartData>))
  "Converts a ROS message object to a list"
  (cl:list 'CartData
    (cl:cons ':header (header msg))
    (cl:cons ':ch (ch msg))
))
