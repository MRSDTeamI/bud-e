; Auto-generated. Do not edit!


(cl:in-package hearbo_cart_msgs-msg)


;//! \htmlinclude CartCommand.msg.html

(cl:defclass <CartCommand> (roslisp-msg-protocol:ros-message)
  ((drive_flag
    :reader drive_flag
    :initarg :drive_flag
    :type cl:fixnum
    :initform 0)
   (gain_flag
    :reader gain_flag
    :initarg :gain_flag
    :type cl:fixnum
    :initform 0)
   (encoder_flag
    :reader encoder_flag
    :initarg :encoder_flag
    :type cl:fixnum
    :initform 0)
   (drive_mode
    :reader drive_mode
    :initarg :drive_mode
    :type cl:fixnum
    :initform 0)
   (drive_id
    :reader drive_id
    :initarg :drive_id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (each_vel
    :reader each_vel
    :initarg :each_vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (each_ang
    :reader each_ang
    :initarg :each_ang
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (all_vel
    :reader all_vel
    :initarg :all_vel
    :type cl:float
    :initform 0.0)
   (all_ang
    :reader all_ang
    :initarg :all_ang
    :type cl:float
    :initform 0.0)
   (gain_id_ch0
    :reader gain_id_ch0
    :initarg :gain_id_ch0
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (gain_id_ch1
    :reader gain_id_ch1
    :initarg :gain_id_ch1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (gain_ch0
    :reader gain_ch0
    :initarg :gain_ch0
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (gain_ch1
    :reader gain_ch1
    :initarg :gain_ch1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (encoder_id_ch0
    :reader encoder_id_ch0
    :initarg :encoder_id_ch0
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (encoder_id_ch1
    :reader encoder_id_ch1
    :initarg :encoder_id_ch1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (encoder_ch0
    :reader encoder_ch0
    :initarg :encoder_ch0
    :type (cl:vector cl:integer)
   :initform (cl:make-array 4 :element-type 'cl:integer :initial-element 0))
   (encoder_ch1
    :reader encoder_ch1
    :initarg :encoder_ch1
    :type (cl:vector cl:integer)
   :initform (cl:make-array 4 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass CartCommand (<CartCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hearbo_cart_msgs-msg:<CartCommand> is deprecated: use hearbo_cart_msgs-msg:CartCommand instead.")))

(cl:ensure-generic-function 'drive_flag-val :lambda-list '(m))
(cl:defmethod drive_flag-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:drive_flag-val is deprecated.  Use hearbo_cart_msgs-msg:drive_flag instead.")
  (drive_flag m))

(cl:ensure-generic-function 'gain_flag-val :lambda-list '(m))
(cl:defmethod gain_flag-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:gain_flag-val is deprecated.  Use hearbo_cart_msgs-msg:gain_flag instead.")
  (gain_flag m))

(cl:ensure-generic-function 'encoder_flag-val :lambda-list '(m))
(cl:defmethod encoder_flag-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:encoder_flag-val is deprecated.  Use hearbo_cart_msgs-msg:encoder_flag instead.")
  (encoder_flag m))

(cl:ensure-generic-function 'drive_mode-val :lambda-list '(m))
(cl:defmethod drive_mode-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:drive_mode-val is deprecated.  Use hearbo_cart_msgs-msg:drive_mode instead.")
  (drive_mode m))

(cl:ensure-generic-function 'drive_id-val :lambda-list '(m))
(cl:defmethod drive_id-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:drive_id-val is deprecated.  Use hearbo_cart_msgs-msg:drive_id instead.")
  (drive_id m))

(cl:ensure-generic-function 'each_vel-val :lambda-list '(m))
(cl:defmethod each_vel-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:each_vel-val is deprecated.  Use hearbo_cart_msgs-msg:each_vel instead.")
  (each_vel m))

(cl:ensure-generic-function 'each_ang-val :lambda-list '(m))
(cl:defmethod each_ang-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:each_ang-val is deprecated.  Use hearbo_cart_msgs-msg:each_ang instead.")
  (each_ang m))

(cl:ensure-generic-function 'all_vel-val :lambda-list '(m))
(cl:defmethod all_vel-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:all_vel-val is deprecated.  Use hearbo_cart_msgs-msg:all_vel instead.")
  (all_vel m))

(cl:ensure-generic-function 'all_ang-val :lambda-list '(m))
(cl:defmethod all_ang-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:all_ang-val is deprecated.  Use hearbo_cart_msgs-msg:all_ang instead.")
  (all_ang m))

(cl:ensure-generic-function 'gain_id_ch0-val :lambda-list '(m))
(cl:defmethod gain_id_ch0-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:gain_id_ch0-val is deprecated.  Use hearbo_cart_msgs-msg:gain_id_ch0 instead.")
  (gain_id_ch0 m))

(cl:ensure-generic-function 'gain_id_ch1-val :lambda-list '(m))
(cl:defmethod gain_id_ch1-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:gain_id_ch1-val is deprecated.  Use hearbo_cart_msgs-msg:gain_id_ch1 instead.")
  (gain_id_ch1 m))

(cl:ensure-generic-function 'gain_ch0-val :lambda-list '(m))
(cl:defmethod gain_ch0-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:gain_ch0-val is deprecated.  Use hearbo_cart_msgs-msg:gain_ch0 instead.")
  (gain_ch0 m))

(cl:ensure-generic-function 'gain_ch1-val :lambda-list '(m))
(cl:defmethod gain_ch1-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:gain_ch1-val is deprecated.  Use hearbo_cart_msgs-msg:gain_ch1 instead.")
  (gain_ch1 m))

(cl:ensure-generic-function 'encoder_id_ch0-val :lambda-list '(m))
(cl:defmethod encoder_id_ch0-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:encoder_id_ch0-val is deprecated.  Use hearbo_cart_msgs-msg:encoder_id_ch0 instead.")
  (encoder_id_ch0 m))

(cl:ensure-generic-function 'encoder_id_ch1-val :lambda-list '(m))
(cl:defmethod encoder_id_ch1-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:encoder_id_ch1-val is deprecated.  Use hearbo_cart_msgs-msg:encoder_id_ch1 instead.")
  (encoder_id_ch1 m))

(cl:ensure-generic-function 'encoder_ch0-val :lambda-list '(m))
(cl:defmethod encoder_ch0-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:encoder_ch0-val is deprecated.  Use hearbo_cart_msgs-msg:encoder_ch0 instead.")
  (encoder_ch0 m))

(cl:ensure-generic-function 'encoder_ch1-val :lambda-list '(m))
(cl:defmethod encoder_ch1-val ((m <CartCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hearbo_cart_msgs-msg:encoder_ch1-val is deprecated.  Use hearbo_cart_msgs-msg:encoder_ch1 instead.")
  (encoder_ch1 m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CartCommand>)))
    "Constants for message type '<CartCommand>"
  '((:FALSE . 0)
    (:TRUE . 1)
    (:STOP_MODE . 0)
    (:TURN_MODE . 1)
    (:DRIVE_MODE . 2)
    (:MANUAL_MODE . 3)
    (:RESTART_MODE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CartCommand)))
    "Constants for message type 'CartCommand"
  '((:FALSE . 0)
    (:TRUE . 1)
    (:STOP_MODE . 0)
    (:TURN_MODE . 1)
    (:DRIVE_MODE . 2)
    (:MANUAL_MODE . 3)
    (:RESTART_MODE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartCommand>) ostream)
  "Serializes a message object of type '<CartCommand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drive_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gain_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'encoder_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drive_mode)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'drive_id))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'each_vel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'each_ang))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'all_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'all_ang))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'gain_id_ch0))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'gain_id_ch1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'gain_ch0))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'gain_ch1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'encoder_id_ch0))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'encoder_id_ch1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'encoder_ch0))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'encoder_ch1))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartCommand>) istream)
  "Deserializes a message object of type '<CartCommand>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drive_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gain_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'encoder_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'drive_mode)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'drive_id) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'drive_id)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'each_vel) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'each_vel)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'each_ang) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'each_ang)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'all_vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'all_ang) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'gain_id_ch0) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'gain_id_ch0)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'gain_id_ch1) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'gain_id_ch1)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'gain_ch0) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'gain_ch0)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'gain_ch1) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'gain_ch1)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'encoder_id_ch0) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'encoder_id_ch0)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'encoder_id_ch1) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'encoder_id_ch1)))
    (cl:dotimes (i 4)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'encoder_ch0) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'encoder_ch0)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  (cl:setf (cl:slot-value msg 'encoder_ch1) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'encoder_ch1)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartCommand>)))
  "Returns string type for a message object of type '<CartCommand>"
  "hearbo_cart_msgs/CartCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartCommand)))
  "Returns string type for a message object of type 'CartCommand"
  "hearbo_cart_msgs/CartCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartCommand>)))
  "Returns md5sum for a message object of type '<CartCommand>"
  "59937212c7ecdba57a473a75da70fa95")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartCommand)))
  "Returns md5sum for a message object of type 'CartCommand"
  "59937212c7ecdba57a473a75da70fa95")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartCommand>)))
  "Returns full string definition for message of type '<CartCommand>"
  (cl:format cl:nil "#~%# iXs cart control message~%#~%~%#~%# Some pre-defined parameters~%#~%~%uint8 FALSE=0~%uint8 TRUE=1~%~%# ====================================================================================~%# Message structure setting~%# ====================================================================================~%~%#~%# valid : ~%# ~%# Major selection of message structure sent to the robot.~%# You can set the following three commands valid/invalid.~%# (1) Commands for drive (steering angle, velocity of driving wheels, selection of free/driven motors)~%# (2) Gain of each motor~%# (3) Motor selection for encoder value reset~%# Those three settings do not have to be sent to the robot simultaneously.~%#~%# Example1~%# If you want to only change the gain, set \"gain_flag\" TRUE.~%#~%# Example2~%# If you want to change both velocity and gain of the motors, set \"gain_flag\" and \"drive_flag\" TRUE. ~%# ~%~%uint8 drive_flag~%uint8 gain_flag~%uint8 encoder_flag~%~%# ====================================================================================~%# Message when drive_flag is TRUE~%# ====================================================================================~%~%#~%# drive_mode : cart control mode~%#~%# STOP_MODE : ~%# This set all the gain zero (Encoder is not reset)~%#~%# TURN_MODE : ~%# Spin rotation mode (Desired encoder values of steering motors is fixed for spinning)~%#~%# DRIVE_MODE : ~%# Steering syncronization mode (Desired encoder values of steering motors are set to be the same.)~%#~%# MANUAL_MODE :~%# You can set the following things freely for each motor.~%# (1) Angle of steering motors, (3) Velocity of driving wheels~%#~%# RESTART_MODE : ~%# This set all the gain default values (Encoder is not reset)~%#~%~%uint8 drive_mode~%uint8 STOP_MODE=0~%uint8 TURN_MODE=1~%uint8 DRIVE_MODE=2~%uint8 MANUAL_MODE=3~%uint8 RESTART_MODE=4~%~%#~%# drive_id : selection of motor id (This decides which motor is processed/unprocessed.)~%# ID0 : Left  front wheel ~%# ID1 : Left  back  wheel ~%# ID2 : Right back wheel ~%# ID3 : Right front wheel ~%# This setting is valid when the \"drive_mode\" is TURN_MODE/DRIVE_MODE/MANUAL_MODE.~%# ~%# Set each element of \"drive_id\" TRUE for activation and FALSE for deactivation.~%# drive_id[0] : Left  front wheel ~%# drive_id[1] : Left  back  wheel ~%# drive_id[2] : Right back wheel ~%# drive_id[3] : Right front wheel ~%# If you set FALSE for some axes, the motors of the axes will be deactivated (free motors). ~%#~%~%uint8[4] drive_id~%~%#~%# each_vel : desired velocity of each drive wheel~%# This setting is valid when the \"drive_mode\" is MANUAL_MODE.~%# ~%# Set each element of \"each_vel\" as the desired velocity [mm/s].~%# each_vel[0] : Left  front wheel ~%# each_vel[1] : Left  back  wheel ~%# each_vel[2] : Right back wheel ~%# each_vel[3] : Right front wheel ~%#~%~%float32[4] each_vel~%~%#~%# each_ang : desired angle of each steering motor~%# This setting is valid when the \"drive_mode\" is MANUAL_MODE.~%# ~%# Set each element of \"each_ang\" as the desired angle [deg].~%# each_ang[0] : Left  front wheel ~%# each_ang[1] : Left  back  wheel ~%# each_ang[2] : Right back wheel ~%# each_ang[3] : Right front wheel ~%#~%~%float32[4] each_ang~%~%#~%# all_vel : desired velocity of all drive wheels~%# This setting is valid when the \"drive_mode\" is TURN_MODE/DRIVE_MODE.~%# ~%# Set \"all_vel\" as the desired velocity [mm/s].~%# Then the desired velocities of all driving wheels are set. ~%#~%~%float32 all_vel~%~%#~%# all_ang : desired angle of all steering motors~%# This setting is valid when the \"drive_mode\" is DRIVE_MODE.~%# ~%# Set \"all_ang\" as the desired angle [deg].~%# Then the desired angles of all steering motors are set. ~%#~%~%float32 all_ang~%~%# ====================================================================================~%# Message when gain_flag is TRUE~%# ====================================================================================~%~%#~%# gain_id_ch : selection of motor id (This decides which gain is set/ignored.)~%# ~%# Set each element of \"gain_id_ch0\" and \"gain_id_ch1\" TRUE for activation and FALSE for deactivation.~%# gain_id_ch0[0] : Left  front driving wheel ~%# gain_id_ch0[1] : Left  back  driving wheel ~%# gain_id_ch0[2] : Right back  driving wheel ~%# gain_id_ch0[3] : Right front driving wheel ~%# gain_id_ch1[0] : Left  front steering motor ~%# gain_id_ch1[1] : Left  back  steering motor ~%# gain_id_ch1[2] : Right back  steering motor ~%# gain_id_ch1[3] : Right front steering motor ~%#~%~%uint8[4] gain_id_ch0~%uint8[4] gain_id_ch1~%~%#~%# gain_ch : propotional gain setting for each motor (P control)~%# ~%# Set each element of \"gain_ch\" as the desired propotional gain.~%# gain_ch0[0] : Left  front driving wheel ~%# gain_ch0[1] : Left  back  driving wheel ~%# gain_ch0[2] : Right back  driving wheel ~%# gain_ch0[3] : Right front driving wheel ~%# gain_ch1[0] : Left  front steering motor ~%# gain_ch1[1] : Left  back  steering motor ~%# gain_ch1[2] : Right back  steering motor ~%# gain_ch1[3] : Right front steering motor ~%#~%~%int16[4] gain_ch0~%int16[4] gain_ch1~%~%# ====================================================================================~%# Message when encoder_flag is TRUE~%# ====================================================================================~%~%#~%# encoder_id_ch : selection of motor id (This decides which motor encoder is reset.)~%# ~%# Set each element of \"encoder_id_ch0\" and \"encoder_id_ch1\" TRUE for activation and FALSE for deactivation.~%# encoder_id_ch0[0] : Left  front driving wheel ~%# encoder_id_ch0[1] : Left  back  driving wheel ~%# encoder_id_ch0[2] : Right back  driving wheel ~%# encoder_id_ch0[3] : Right front driving wheel ~%# encoder_id_ch1[0] : Left  front steering motor ~%# encoder_id_ch1[1] : Left  back  steering motor ~%# encoder_id_ch1[2] : Right back  steering motor ~%# encoder_id_ch1[3] : Right front steering motor ~%#~%~%uint8[4] encoder_id_ch0~%uint8[4] encoder_id_ch1~%~%#~%# encoder_ch : encoder reset value setting for each motor~%# ~%# Set each element of \"encoder_ch\" as the encoder value after reset.~%# encoder_ch0[0] : Left  front driving wheel ~%# encoder_ch0[1] : Left  back  driving wheel ~%# encoder_ch0[2] : Right back  driving wheel ~%# encoder_ch0[3] : Right front driving wheel ~%# encoder_ch1[0] : Left  front steering motor ~%# encoder_ch1[1] : Left  back  steering motor ~%# encoder_ch1[2] : Right back  steering motor ~%# encoder_ch1[3] : Right front steering motor ~%#~%~%int32[4] encoder_ch0~%int32[4] encoder_ch1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartCommand)))
  "Returns full string definition for message of type 'CartCommand"
  (cl:format cl:nil "#~%# iXs cart control message~%#~%~%#~%# Some pre-defined parameters~%#~%~%uint8 FALSE=0~%uint8 TRUE=1~%~%# ====================================================================================~%# Message structure setting~%# ====================================================================================~%~%#~%# valid : ~%# ~%# Major selection of message structure sent to the robot.~%# You can set the following three commands valid/invalid.~%# (1) Commands for drive (steering angle, velocity of driving wheels, selection of free/driven motors)~%# (2) Gain of each motor~%# (3) Motor selection for encoder value reset~%# Those three settings do not have to be sent to the robot simultaneously.~%#~%# Example1~%# If you want to only change the gain, set \"gain_flag\" TRUE.~%#~%# Example2~%# If you want to change both velocity and gain of the motors, set \"gain_flag\" and \"drive_flag\" TRUE. ~%# ~%~%uint8 drive_flag~%uint8 gain_flag~%uint8 encoder_flag~%~%# ====================================================================================~%# Message when drive_flag is TRUE~%# ====================================================================================~%~%#~%# drive_mode : cart control mode~%#~%# STOP_MODE : ~%# This set all the gain zero (Encoder is not reset)~%#~%# TURN_MODE : ~%# Spin rotation mode (Desired encoder values of steering motors is fixed for spinning)~%#~%# DRIVE_MODE : ~%# Steering syncronization mode (Desired encoder values of steering motors are set to be the same.)~%#~%# MANUAL_MODE :~%# You can set the following things freely for each motor.~%# (1) Angle of steering motors, (3) Velocity of driving wheels~%#~%# RESTART_MODE : ~%# This set all the gain default values (Encoder is not reset)~%#~%~%uint8 drive_mode~%uint8 STOP_MODE=0~%uint8 TURN_MODE=1~%uint8 DRIVE_MODE=2~%uint8 MANUAL_MODE=3~%uint8 RESTART_MODE=4~%~%#~%# drive_id : selection of motor id (This decides which motor is processed/unprocessed.)~%# ID0 : Left  front wheel ~%# ID1 : Left  back  wheel ~%# ID2 : Right back wheel ~%# ID3 : Right front wheel ~%# This setting is valid when the \"drive_mode\" is TURN_MODE/DRIVE_MODE/MANUAL_MODE.~%# ~%# Set each element of \"drive_id\" TRUE for activation and FALSE for deactivation.~%# drive_id[0] : Left  front wheel ~%# drive_id[1] : Left  back  wheel ~%# drive_id[2] : Right back wheel ~%# drive_id[3] : Right front wheel ~%# If you set FALSE for some axes, the motors of the axes will be deactivated (free motors). ~%#~%~%uint8[4] drive_id~%~%#~%# each_vel : desired velocity of each drive wheel~%# This setting is valid when the \"drive_mode\" is MANUAL_MODE.~%# ~%# Set each element of \"each_vel\" as the desired velocity [mm/s].~%# each_vel[0] : Left  front wheel ~%# each_vel[1] : Left  back  wheel ~%# each_vel[2] : Right back wheel ~%# each_vel[3] : Right front wheel ~%#~%~%float32[4] each_vel~%~%#~%# each_ang : desired angle of each steering motor~%# This setting is valid when the \"drive_mode\" is MANUAL_MODE.~%# ~%# Set each element of \"each_ang\" as the desired angle [deg].~%# each_ang[0] : Left  front wheel ~%# each_ang[1] : Left  back  wheel ~%# each_ang[2] : Right back wheel ~%# each_ang[3] : Right front wheel ~%#~%~%float32[4] each_ang~%~%#~%# all_vel : desired velocity of all drive wheels~%# This setting is valid when the \"drive_mode\" is TURN_MODE/DRIVE_MODE.~%# ~%# Set \"all_vel\" as the desired velocity [mm/s].~%# Then the desired velocities of all driving wheels are set. ~%#~%~%float32 all_vel~%~%#~%# all_ang : desired angle of all steering motors~%# This setting is valid when the \"drive_mode\" is DRIVE_MODE.~%# ~%# Set \"all_ang\" as the desired angle [deg].~%# Then the desired angles of all steering motors are set. ~%#~%~%float32 all_ang~%~%# ====================================================================================~%# Message when gain_flag is TRUE~%# ====================================================================================~%~%#~%# gain_id_ch : selection of motor id (This decides which gain is set/ignored.)~%# ~%# Set each element of \"gain_id_ch0\" and \"gain_id_ch1\" TRUE for activation and FALSE for deactivation.~%# gain_id_ch0[0] : Left  front driving wheel ~%# gain_id_ch0[1] : Left  back  driving wheel ~%# gain_id_ch0[2] : Right back  driving wheel ~%# gain_id_ch0[3] : Right front driving wheel ~%# gain_id_ch1[0] : Left  front steering motor ~%# gain_id_ch1[1] : Left  back  steering motor ~%# gain_id_ch1[2] : Right back  steering motor ~%# gain_id_ch1[3] : Right front steering motor ~%#~%~%uint8[4] gain_id_ch0~%uint8[4] gain_id_ch1~%~%#~%# gain_ch : propotional gain setting for each motor (P control)~%# ~%# Set each element of \"gain_ch\" as the desired propotional gain.~%# gain_ch0[0] : Left  front driving wheel ~%# gain_ch0[1] : Left  back  driving wheel ~%# gain_ch0[2] : Right back  driving wheel ~%# gain_ch0[3] : Right front driving wheel ~%# gain_ch1[0] : Left  front steering motor ~%# gain_ch1[1] : Left  back  steering motor ~%# gain_ch1[2] : Right back  steering motor ~%# gain_ch1[3] : Right front steering motor ~%#~%~%int16[4] gain_ch0~%int16[4] gain_ch1~%~%# ====================================================================================~%# Message when encoder_flag is TRUE~%# ====================================================================================~%~%#~%# encoder_id_ch : selection of motor id (This decides which motor encoder is reset.)~%# ~%# Set each element of \"encoder_id_ch0\" and \"encoder_id_ch1\" TRUE for activation and FALSE for deactivation.~%# encoder_id_ch0[0] : Left  front driving wheel ~%# encoder_id_ch0[1] : Left  back  driving wheel ~%# encoder_id_ch0[2] : Right back  driving wheel ~%# encoder_id_ch0[3] : Right front driving wheel ~%# encoder_id_ch1[0] : Left  front steering motor ~%# encoder_id_ch1[1] : Left  back  steering motor ~%# encoder_id_ch1[2] : Right back  steering motor ~%# encoder_id_ch1[3] : Right front steering motor ~%#~%~%uint8[4] encoder_id_ch0~%uint8[4] encoder_id_ch1~%~%#~%# encoder_ch : encoder reset value setting for each motor~%# ~%# Set each element of \"encoder_ch\" as the encoder value after reset.~%# encoder_ch0[0] : Left  front driving wheel ~%# encoder_ch0[1] : Left  back  driving wheel ~%# encoder_ch0[2] : Right back  driving wheel ~%# encoder_ch0[3] : Right front driving wheel ~%# encoder_ch1[0] : Left  front steering motor ~%# encoder_ch1[1] : Left  back  steering motor ~%# encoder_ch1[2] : Right back  steering motor ~%# encoder_ch1[3] : Right front steering motor ~%#~%~%int32[4] encoder_ch0~%int32[4] encoder_ch1~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartCommand>))
  (cl:+ 0
     1
     1
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'drive_id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'each_vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'each_ang) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gain_id_ch0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gain_id_ch1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gain_ch0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gain_ch1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'encoder_id_ch0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'encoder_id_ch1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'encoder_ch0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'encoder_ch1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'CartCommand
    (cl:cons ':drive_flag (drive_flag msg))
    (cl:cons ':gain_flag (gain_flag msg))
    (cl:cons ':encoder_flag (encoder_flag msg))
    (cl:cons ':drive_mode (drive_mode msg))
    (cl:cons ':drive_id (drive_id msg))
    (cl:cons ':each_vel (each_vel msg))
    (cl:cons ':each_ang (each_ang msg))
    (cl:cons ':all_vel (all_vel msg))
    (cl:cons ':all_ang (all_ang msg))
    (cl:cons ':gain_id_ch0 (gain_id_ch0 msg))
    (cl:cons ':gain_id_ch1 (gain_id_ch1 msg))
    (cl:cons ':gain_ch0 (gain_ch0 msg))
    (cl:cons ':gain_ch1 (gain_ch1 msg))
    (cl:cons ':encoder_id_ch0 (encoder_id_ch0 msg))
    (cl:cons ':encoder_id_ch1 (encoder_id_ch1 msg))
    (cl:cons ':encoder_ch0 (encoder_ch0 msg))
    (cl:cons ':encoder_ch1 (encoder_ch1 msg))
))
