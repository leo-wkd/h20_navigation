; Auto-generated. Do not edit!


(in-package drrobot_player-msg)


;//! \htmlinclude RangeArray.msg.html

(defclass <RangeArray> (ros-message)
  ((ranges
    :reader ranges-val
    :initarg :ranges
    :type (vector <Range>)
   :initform (make-array 0 :element-type '<Range> :initial-element (make-instance '<Range>))))
)
(defmethod serialize ((msg <RangeArray>) ostream)
  "Serializes a message object of type '<RangeArray>"
  (let ((__ros_arr_len (length (slot-value msg 'ranges))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (serialize ele ostream))
    (slot-value msg 'ranges))
)
(defmethod deserialize ((msg <RangeArray>) istream)
  "Deserializes a message object of type '<RangeArray>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'ranges) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'ranges)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (make-instance '<Range>))
(deserialize (aref vals i) istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<RangeArray>)))
  "Returns string type for a message object of type '<RangeArray>"
  "drrobot_player/RangeArray")
(defmethod md5sum ((type (eql '<RangeArray>)))
  "Returns md5sum for a message object of type '<RangeArray>"
  "b72db098d4ed346ce682a1d5e70d327c")
(defmethod message-definition ((type (eql '<RangeArray>)))
  "Returns full string definition for message of type '<RangeArray>"
  (format nil "#this message will be used for IR ranger sensor and ultrasonic range sensor~%Range[] ranges~%~%================================================================================~%MSG: drrobot_player/Range~%# Single range reading from an active ranger that emits energy and reports~%# one range reading that is valid along an arc at the distance measured. ~%# This message is not appropriate for fixed-range obstacle detectors, ~%# such as the Sharp GP2D15. This message is also not appropriate for laser ~%# scanners. See the LaserScan message if you are working with a laser scanner.~%~%Header header    	# timestamp in the header is the time the ranger~%		 	# returned the distance reading~%~%# Radiation type enums~%# If you want a value added to this list, send an email to the ros-users list~%uint8 ULTRASOUND=0~%uint8 INFRARED=1~%~%uint8 radiation_type    # the type of radiation used by the sensor~%		 	# (sound, IR, etc) [enum]~%~%float32 field_of_view   # the size of the arc that the distance reading is~%		 	# valid for [rad]~%		 	# the object causing the range reading may have~%		 	# been anywhere within -field_of_view/2 and~%		 	# field_of_view/2 at the measured range. ~%                        # 0 angle corresponds to the x-axis of the sensor.~%~%float32 min_range       # minimum range value [m]~%float32 max_range       # maximum range value [m]~%~%float32 range           # range data [m]~%		 	# (Note: values < range_min or > range_max~%		 	# should be discarded)~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <RangeArray>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'ranges) :key #'(lambda (ele) (declare (ignorable ele)) (+ (serialization-length ele))))
))
(defmethod ros-message-to-list ((msg <RangeArray>))
  "Converts a ROS message object to a list"
  (list '<RangeArray>
    (cons ':ranges (ranges-val msg))
))
