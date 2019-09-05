; Auto-generated. Do not edit!


(in-package drrobot_player-msg)


;//! \htmlinclude CustomSensor.msg.html

(defclass <CustomSensor> (ros-message)
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
   (customADData
    :reader customADData-val
    :initarg :customADData
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (customIO
    :reader customIO-val
    :initarg :customIO
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <CustomSensor>) ostream)
  "Serializes a message object of type '<CustomSensor>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'robot_type))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'robot_type))
  (let ((__ros_arr_len (length (slot-value msg 'customADData))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'customADData))
    (write-byte (ldb (byte 8 0) (slot-value msg 'customIO)) ostream)
)
(defmethod deserialize ((msg <CustomSensor>) istream)
  "Deserializes a message object of type '<CustomSensor>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'robot_type) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'robot_type) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'customADData) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'customADData)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'customIO)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<CustomSensor>)))
  "Returns string type for a message object of type '<CustomSensor>"
  "drrobot_player/CustomSensor")
(defmethod md5sum ((type (eql '<CustomSensor>)))
  "Returns md5sum for a message object of type '<CustomSensor>"
  "99688dde61b19bf7f23d2791779ffdd3")
(defmethod message-definition ((type (eql '<CustomSensor>)))
  "Returns full string definition for message of type '<CustomSensor>"
  (format nil "# custom Sensor information message from DrRobot Robot.~%~%Header header    	# timestamp in the header is the time the driver~%		 	# returned the battery/power reading~%string robot_type	# robot type, I90 series, sentinel3 or Hawk/H20 Power/Motion~%~%#make sure what sensor is on your expanded sensor port~%uint32[] customADData		# custom AD channel 8 channel on the board, which channel is avaiable please contact Dr Robot~%uint8 customIO    #expanded IO~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <CustomSensor>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'robot_type))
     4 (reduce #'+ (slot-value msg 'customADData) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     1
))
(defmethod ros-message-to-list ((msg <CustomSensor>))
  "Converts a ROS message object to a list"
  (list '<CustomSensor>
    (cons ':header (header-val msg))
    (cons ':robot_type (robot_type-val msg))
    (cons ':customADData (customADData-val msg))
    (cons ':customIO (customIO-val msg))
))
