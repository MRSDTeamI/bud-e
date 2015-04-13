; Auto-generated. Do not edit!


(cl:in-package hearbo_cart_msgs-msg)


;//! \htmlinclude EncVal.msg.html

(cl:defclass <EncVal> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:integer)
   :initform (cl:make-array 4 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass EncVal (<EncVal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EncVal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EncVal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hearbo_cart_msgs-msg:<EncVal> is deprecated: use hearbo_cart_msgs-msg:EncVal instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <EncVal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:data-val is deprecated.  Use hearbo_cart_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EncVal>) ostream)
  "Serializes a message object of type '<EncVal>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EncVal>) istream)
  "Deserializes a message object of type '<EncVal>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EncVal>)))
  "Returns string type for a message object of type '<EncVal>"
  "hearbo_cart_msgs/EncVal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EncVal)))
  "Returns string type for a message object of type 'EncVal"
  "hearbo_cart_msgs/EncVal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EncVal>)))
  "Returns md5sum for a message object of type '<EncVal>"
  "f628a1313d75e7c0f4d18e856bac80c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EncVal)))
  "Returns md5sum for a message object of type 'EncVal"
  "f628a1313d75e7c0f4d18e856bac80c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EncVal>)))
  "Returns full string definition for message of type '<EncVal>"
  (cl:format cl:nil "# Encoder value for all the wheels of the iXs cart~%# data[0] : ID = 0 : Left  forward wheel~%# data[1] : ID = 1 : Left  back    wheel~%# data[2] : ID = 2 : Right back    wheel~%# data[3] : ID = 3 : Right forward wheel~%~%uint32[4] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EncVal)))
  "Returns full string definition for message of type 'EncVal"
  (cl:format cl:nil "# Encoder value for all the wheels of the iXs cart~%# data[0] : ID = 0 : Left  forward wheel~%# data[1] : ID = 1 : Left  back    wheel~%# data[2] : ID = 2 : Right back    wheel~%# data[3] : ID = 3 : Right forward wheel~%~%uint32[4] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EncVal>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EncVal>))
  "Converts a ROS message object to a list"
  (cl:list 'EncVal
    (cl:cons ':data (data msg))
))
