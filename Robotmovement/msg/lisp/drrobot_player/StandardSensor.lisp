; Auto-generated. Do not edit!


(in-package drrobot_player-msg)


;//! \htmlinclude StandardSensor.msg.html

(defclass <StandardSensor> (ros-message)
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
   (humanSensorData
    :reader humanSensorData-val
    :initarg :humanSensorData
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (tiltingSensorData
    :reader tiltingSensorData-val
    :initarg :tiltingSensorData
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (overHeatSensorData
    :reader overHeatSensorData-val
    :initarg :overHeatSensorData
    :type (vector integer)
   :initform (make-array 0 :element-type 'integer :initial-element 0))
   (thermoSensorData
    :reader thermoSensorData-val
    :initarg :thermoSensorData
    :type integer
    :initform 0)
   (boardPowerVol
    :reader boardPowerVol-val
    :initarg :boardPowerVol
    :type float
    :initform 0.0)
   (motorPowerVol
    :reader motorPowerVol-val
    :initarg :motorPowerVol
    :type float
    :initform 0.0)
   (servoPowerVol
    :reader servoPowerVol-val
    :initarg :servoPowerVol
    :type float
    :initform 0.0)
   (refVol
    :reader refVol-val
    :initarg :refVol
    :type float
    :initform 0.0)
   (potVol
    :reader potVol-val
    :initarg :potVol
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <StandardSensor>) ostream)
  "Serializes a message object of type '<StandardSensor>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'robot_type))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'robot_type))
  (let ((__ros_arr_len (length (slot-value msg 'humanSensorData))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'humanSensorData))
  (let ((__ros_arr_len (length (slot-value msg 'tiltingSensorData))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'tiltingSensorData))
  (let ((__ros_arr_len (length (slot-value msg 'overHeatSensorData))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream)
  (write-byte (ldb (byte 8 8) ele) ostream)
  (write-byte (ldb (byte 8 16) ele) ostream)
  (write-byte (ldb (byte 8 24) ele) ostream))
    (slot-value msg 'overHeatSensorData))
    (write-byte (ldb (byte 8 0) (slot-value msg 'thermoSensorData)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'thermoSensorData)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'thermoSensorData)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'thermoSensorData)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'boardPowerVol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'motorPowerVol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'servoPowerVol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'refVol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'potVol))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <StandardSensor>) istream)
  "Deserializes a message object of type '<StandardSensor>"
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
    (setf (slot-value msg 'humanSensorData) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'humanSensorData)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'tiltingSensorData) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'tiltingSensorData)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'overHeatSensorData) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'overHeatSensorData)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 8) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 16) (aref vals i)) (read-byte istream))
  (setf (ldb (byte 8 24) (aref vals i)) (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'thermoSensorData)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'thermoSensorData)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'thermoSensorData)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'thermoSensorData)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'boardPowerVol) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'motorPowerVol) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'servoPowerVol) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'refVol) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'potVol) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<StandardSensor>)))
  "Returns string type for a message object of type '<StandardSensor>"
  "drrobot_player/StandardSensor")
(defmethod md5sum ((type (eql '<StandardSensor>)))
  "Returns md5sum for a message object of type '<StandardSensor>"
  "e867c61af4cf779989629edb31d6fbd8")
(defmethod message-definition ((type (eql '<StandardSensor>)))
  "Returns full string definition for message of type '<StandardSensor>"
  (format nil "# standard Sensor information message from DrRobot Robot.~%~%Header header    	# timestamp in the header is the time the driver~%		 	# returned the battery/power reading~%string robot_type	# robot type, I90 series, sentinel3 or Hawk/H20 Power/Motion~%~%#make sure below sensors on your robot or not~%uint32[] humanSensorData		#human sensor~%uint32[] tiltingSensorData    #tilting sensor X Y~%uint32[] overHeatSensorData   # over heat sensor on the board~%uint32 thermoSensorData	    # enviroment temperature~%float32 boardPowerVol 			# control board voltage~%float32 motorPowerVol			# motor power voltage~%float32 servoPowerVol			# servo power voltage~%float32 refVol					# AD reference voltage~%float32 potVol					# potentialmeter power voltage , not used now~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <StandardSensor>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'robot_type))
     4 (reduce #'+ (slot-value msg 'humanSensorData) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'tiltingSensorData) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4 (reduce #'+ (slot-value msg 'overHeatSensorData) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <StandardSensor>))
  "Converts a ROS message object to a list"
  (list '<StandardSensor>
    (cons ':header (header-val msg))
    (cons ':robot_type (robot_type-val msg))
    (cons ':humanSensorData (humanSensorData-val msg))
    (cons ':tiltingSensorData (tiltingSensorData-val msg))
    (cons ':overHeatSensorData (overHeatSensorData-val msg))
    (cons ':thermoSensorData (thermoSensorData-val msg))
    (cons ':boardPowerVol (boardPowerVol-val msg))
    (cons ':motorPowerVol (motorPowerVol-val msg))
    (cons ':servoPowerVol (servoPowerVol-val msg))
    (cons ':refVol (refVol-val msg))
    (cons ':potVol (potVol-val msg))
))
