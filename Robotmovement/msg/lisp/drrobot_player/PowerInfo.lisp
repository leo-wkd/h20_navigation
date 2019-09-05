; Auto-generated. Do not edit!


(in-package drrobot_player-msg)


;//! \htmlinclude PowerInfo.msg.html

(defclass <PowerInfo> (ros-message)
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
   (bat1_vol
    :reader bat1_vol-val
    :initarg :bat1_vol
    :type float
    :initform 0.0)
   (bat2_vol
    :reader bat2_vol-val
    :initarg :bat2_vol
    :type float
    :initform 0.0)
   (bat1_temp
    :reader bat1_temp-val
    :initarg :bat1_temp
    :type float
    :initform 0.0)
   (bat2_temp
    :reader bat2_temp-val
    :initarg :bat2_temp
    :type float
    :initform 0.0)
   (dcin_vol
    :reader dcin_vol-val
    :initarg :dcin_vol
    :type float
    :initform 0.0)
   (ref_vol
    :reader ref_vol-val
    :initarg :ref_vol
    :type float
    :initform 0.0)
   (power_status
    :reader power_status-val
    :initarg :power_status
    :type fixnum
    :initform 0)
   (power_path
    :reader power_path-val
    :initarg :power_path
    :type fixnum
    :initform 0)
   (charge_path
    :reader charge_path-val
    :initarg :charge_path
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <PowerInfo>) ostream)
  "Serializes a message object of type '<PowerInfo>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'robot_type))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'robot_type))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'bat1_vol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'bat2_vol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'bat1_temp))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'bat2_temp))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'dcin_vol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'ref_vol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'power_status)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'power_path)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'charge_path)) ostream)
)
(defmethod deserialize ((msg <PowerInfo>) istream)
  "Deserializes a message object of type '<PowerInfo>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'robot_type) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'robot_type) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'bat1_vol) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'bat2_vol) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'bat1_temp) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'bat2_temp) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'dcin_vol) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'ref_vol) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'power_status)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'power_path)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'charge_path)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PowerInfo>)))
  "Returns string type for a message object of type '<PowerInfo>"
  "drrobot_player/PowerInfo")
(defmethod md5sum ((type (eql '<PowerInfo>)))
  "Returns md5sum for a message object of type '<PowerInfo>"
  "13107d877ae888e7541f720d1432d852")
(defmethod message-definition ((type (eql '<PowerInfo>)))
  "Returns full string definition for message of type '<PowerInfo>"
  (format nil "# battery /power information message from DrRobot Robot.~%~%Header header    	# timestamp in the header is the time the driver~%		 	# returned the battery/power reading~%string robot_type	# robot type, I90 series, sentinel3 or Hawk/H20 Power/Motion~%~%#below message is only I90 series with Power control system on robot, otherwise reserved~%float32 bat1_vol	# battery1 voltage~%float32 bat2_vol	# battery2 voltage~%float32 bat1_temp	# battery1 temperature reading, now only is the AD value~%float32 bat2_temp	# battery2 temperature reading, now only is the AD value~%float32 dcin_vol	# dcin power voltage reading~%float32 ref_vol		# board AD reference voltage reading~%uint8 power_status	# power status, referee document to get detailed info for every bit~%uint8 power_path	# power selected path, please referee DrRobot document~%uint8 charge_path	# charger selected path, please referee DrRobot document~%~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <PowerInfo>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'robot_type))
     4
     4
     4
     4
     4
     4
     1
     1
     1
))
(defmethod ros-message-to-list ((msg <PowerInfo>))
  "Converts a ROS message object to a list"
  (list '<PowerInfo>
    (cons ':header (header-val msg))
    (cons ':robot_type (robot_type-val msg))
    (cons ':bat1_vol (bat1_vol-val msg))
    (cons ':bat2_vol (bat2_vol-val msg))
    (cons ':bat1_temp (bat1_temp-val msg))
    (cons ':bat2_temp (bat2_temp-val msg))
    (cons ':dcin_vol (dcin_vol-val msg))
    (cons ':ref_vol (ref_vol-val msg))
    (cons ':power_status (power_status-val msg))
    (cons ':power_path (power_path-val msg))
    (cons ':charge_path (charge_path-val msg))
))
