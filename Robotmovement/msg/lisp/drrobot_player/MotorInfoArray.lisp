; Auto-generated. Do not edit!


(in-package drrobot_player-msg)


;//! \htmlinclude MotorInfoArray.msg.html

(defclass <MotorInfoArray> (ros-message)
  ((motorInfos
    :reader motorInfos-val
    :initarg :motorInfos
    :type (vector <MotorInfo>)
   :initform (make-array 0 :element-type '<MotorInfo> :initial-element (make-instance '<MotorInfo>))))
)
(defmethod serialize ((msg <MotorInfoArray>) ostream)
  "Serializes a message object of type '<MotorInfoArray>"
  (let ((__ros_arr_len (length (slot-value msg 'motorInfos))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'motorInfos))
)
(defmethod deserialize ((msg <MotorInfoArray>) istream)
  "Deserializes a message object of type '<MotorInfoArray>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'motorInfos) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'motorInfos)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance '<MotorInfo>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<MotorInfoArray>)))
  "Returns string type for a message object of type '<MotorInfoArray>"
  "drrobot_player/MotorInfoArray")
(defmethod md5sum ((type (eql '<MotorInfoArray>)))
  "Returns md5sum for a message object of type '<MotorInfoArray>"
  "64d8eb9826ec2f78779f54df29bcc931")
(defmethod message-definition ((type (eql '<MotorInfoArray>)))
  "Returns full string definition for message of type '<MotorInfoArray>"
  (format nil "#this message will be used for motor sensor~%MotorInfo[] motorInfos~%~%================================================================================~%MSG: drrobot_player/MotorInfo~%# motor sensor data message from DrRobot Robot.~%~%Header header    	# timestamp in the header is the time the driver~%		 	# returned the battery/power reading~%string robot_type	# robot type, I90 series, sentinel3 or Jaguar Power/Motion~%~%uint32 encoder_pos	# encoder positon count~%uint32 encoder_vel	# encoder velocity value~%uint32 encoder_dir	# encoder direction~%~%float32 motor_current	# motor current~%uint32 motor_pwm	# output PWM value, only for Jaguar series robot~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <MotorInfoArray>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'motorInfos) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <MotorInfoArray>))
  "Converts a ROS message object to a list"
  (list '<MotorInfoArray>
    (cons ':motorInfos (motorInfos-val msg))
))
