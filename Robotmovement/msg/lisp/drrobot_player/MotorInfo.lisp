; Auto-generated. Do not edit!


(in-package drrobot_player-msg)


;//! \htmlinclude MotorInfo.msg.html

(defclass <MotorInfo> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (robot_type
    :reader robot_type-val
    :initarg :robot_type
    :type string
    :initform "")
   (encoder_pos
    :reader encoder_pos-val
    :initarg :encoder_pos
    :type integer
    :initform 0)
   (encoder_vel
    :reader encoder_vel-val
    :initarg :encoder_vel
    :type integer
    :initform 0)
   (encoder_dir
    :reader encoder_dir-val
    :initarg :encoder_dir
    :type integer
    :initform 0)
   (motor_current
    :reader motor_current-val
    :initarg :motor_current
    :type float
    :initform 0.0)
   (motor_pwm
    :reader motor_pwm-val
    :initarg :motor_pwm
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <MotorInfo>) ostream)
  "Serializes a message object of type '<MotorInfo>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'robot_type))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'robot_type))
    (write-byte (ldb (byte 8 0) (slot-value msg 'encoder_pos)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'encoder_pos)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'encoder_pos)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'encoder_pos)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'encoder_vel)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'encoder_vel)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'encoder_vel)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'encoder_vel)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'encoder_dir)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'encoder_dir)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'encoder_dir)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'encoder_dir)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'motor_current))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'motor_pwm)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'motor_pwm)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'motor_pwm)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'motor_pwm)) ostream)
)
(defmethod deserialize ((msg <MotorInfo>) istream)
  "Deserializes a message object of type '<MotorInfo>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'robot_type) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'robot_type) __ros_str_idx) (code-char (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'encoder_pos)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'encoder_pos)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'encoder_pos)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'encoder_pos)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'encoder_vel)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'encoder_vel)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'encoder_vel)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'encoder_vel)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'encoder_dir)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'encoder_dir)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'encoder_dir)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'encoder_dir)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'motor_current) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'motor_pwm)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'motor_pwm)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'motor_pwm)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'motor_pwm)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<MotorInfo>)))
  "Returns string type for a message object of type '<MotorInfo>"
  "drrobot_player/MotorInfo")
(defmethod md5sum ((type (eql '<MotorInfo>)))
  "Returns md5sum for a message object of type '<MotorInfo>"
  "9e31f4f22948e8b2ee140c8cc701e042")
(defmethod message-definition ((type (eql '<MotorInfo>)))
  "Returns full string definition for message of type '<MotorInfo>"
  (format nil "# motor sensor data message from DrRobot Robot.~%~%Header header    	# timestamp in the header is the time the driver~%		 	# returned the battery/power reading~%string robot_type	# robot type, I90 series, sentinel3 or Jaguar Power/Motion~%~%uint32 encoder_pos	# encoder positon count~%uint32 encoder_vel	# encoder velocity value~%uint32 encoder_dir	# encoder direction~%~%float32 motor_current	# motor current~%uint32 motor_pwm	# output PWM value, only for Jaguar series robot~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <MotorInfo>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'robot_type))
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <MotorInfo>))
  "Converts a ROS message object to a list"
  (list '<MotorInfo>
    (cons ':header (header-val msg))
    (cons ':robot_type (robot_type-val msg))
    (cons ':encoder_pos (encoder_pos-val msg))
    (cons ':encoder_vel (encoder_vel-val msg))
    (cons ':encoder_dir (encoder_dir-val msg))
    (cons ':motor_current (motor_current-val msg))
    (cons ':motor_pwm (motor_pwm-val msg))
))
