; Auto-generated. Do not edit!


(cl:in-package hearbo_cart_msgs-msg)


;//! \htmlinclude CartVelocity.msg.html

(cl:defclass <CartVelocity> (roslisp-msg-protocol:ros-message)
  ((velocity_x
    :reader velocity_x
    :initarg :velocity_x
    :type cl:float
    :initform 0.0)
   (velocity_y
    :reader velocity_y
    :initarg :velocity_y
    :type cl:float
    :initform 0.0)
   (velocity_rot
    :reader velocity_rot
    :initarg :velocity_rot
    :type cl:float
    :initform 0.0))
)

(cl:defclass CartVelocity (<CartVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hearbo_cart_msgs-msg:<CartVelocity> is deprecated: use hearbo_cart_msgs-msg:CartVelocity instead.")))

(cl:ensure-generic-function 'velocity_x-val :lambda-list '(m))
(cl:defmethod velocity_x-val ((m <CartVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:velocity_x-val is deprecated.  Use hearbo_cart_msgs-msg:velocity_x instead.")
  (velocity_x m))

(cl:ensure-generic-function 'velocity_y-val :lambda-list '(m))
(cl:defmethod velocity_y-val ((m <CartVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:velocity_y-val is deprecated.  Use hearbo_cart_msgs-msg:velocity_y instead.")
  (velocity_y m))

(cl:ensure-generic-function 'velocity_rot-val :lambda-list '(m))
(cl:defmethod velocity_rot-val ((m <CartVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:velocity_rot-val is deprecated.  Use hearbo_cart_msgs-msg:velocity_rot instead.")
  (velocity_rot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartVelocity>) ostream)
  "Serializes a message object of type '<CartVelocity>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity_rot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartVelocity>) istream)
  "Deserializes a message object of type '<CartVelocity>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_rot) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartVelocity>)))
  "Returns string type for a message object of type '<CartVelocity>"
  "hearbo_cart_msgs/CartVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartVelocity)))
  "Returns string type for a message object of type 'CartVelocity"
  "hearbo_cart_msgs/CartVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartVelocity>)))
  "Returns md5sum for a message object of type '<CartVelocity>"
  "94229bd71d5361c95436a6dfadea58b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartVelocity)))
  "Returns md5sum for a message object of type 'CartVelocity"
  "94229bd71d5361c95436a6dfadea58b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartVelocity>)))
  "Returns full string definition for message of type '<CartVelocity>"
  (cl:format cl:nil "float64 velocity_x    #positive is forward~%float64 velocity_y    #positive is towards the left~%float64 velocity_rot  #positive is counter_clockwise~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartVelocity)))
  "Returns full string definition for message of type 'CartVelocity"
  (cl:format cl:nil "float64 velocity_x    #positive is forward~%float64 velocity_y    #positive is towards the left~%float64 velocity_rot  #positive is counter_clockwise~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartVelocity>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'CartVelocity
    (cl:cons ':velocity_x (velocity_x msg))
    (cl:cons ':velocity_y (velocity_y msg))
    (cl:cons ':velocity_rot (velocity_rot msg))
))
