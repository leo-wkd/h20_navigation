; Auto-generated. Do not edit!


(in-package drrobot_player-msg)


;//! \htmlinclude Range.msg.html

(defclass <Range> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (radiation_type
    :reader radiation_type-val
    :initarg :radiation_type
    :type fixnum
    :initform 0)
   (field_of_view
    :reader field_of_view-val
    :initarg :field_of_view
    :type float
    :initform 0.0)
   (min_range
    :reader min_range-val
    :initarg :min_range
    :type float
    :initform 0.0)
   (max_range
    :reader max_range-val
    :initarg :max_range
    :type float
    :initform 0.0)
   (range
    :reader range-val
    :initarg :range
    :type float
    :initform 0.0))
)
(defmethod symbol-codes ((msg-type (eql '<Range>)))
  "Constants for message type '<Range>"
  '((:ULTRASOUND . 0)
    (:INFRARED . 1))
)
(defmethod serialize ((msg <Range>) ostream)
  "Serializes a message object of type '<Range>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'radiation_type)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'field_of_view))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'min_range))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'max_range))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'range))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <Range>) istream)
  "Deserializes a message object of type '<Range>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'radiation_type)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'field_of_view) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'min_range) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'max_range) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<Range>)))
  "Returns string type for a message object of type '<Range>"
  "drrobot_player/Range")
(defmethod md5sum ((type (eql '<Range>)))
  "Returns md5sum for a message object of type '<Range>"
  "c005c34273dc426c67a020a87bc24148")
(defmethod message-definition ((type (eql '<Range>)))
  "Returns full string definition for message of type '<Range>"
  (format nil "# Single range reading from an active ranger that emits energy and reports~%# one range reading that is valid along an arc at the distance measured. ~%# This message is not appropriate for fixed-range obstacle detectors, ~%# such as the Sharp GP2D15. This message is also not appropriate for laser ~%# scanners. See the LaserScan message if you are working with a laser scanner.~%~%Header header    	# timestamp in the header is the time the ranger~%		 	# returned the distance reading~%~%# Radiation type enums~%# If you want a value added to this list, send an email to the ros-users list~%uint8 ULTRASOUND=0~%uint8 INFRARED=1~%~%uint8 radiation_type    # the type of radiation used by the sensor~%		 	# (sound, IR, etc) [enum]~%~%float32 field_of_view   # the size of the arc that the distance reading is~%		 	# valid for [rad]~%		 	# the object causing the range reading may have~%		 	# been anywhere within -field_of_view/2 and~%		 	# field_of_view/2 at the measured range. ~%                        # 0 angle corresponds to the x-axis of the sensor.~%~%float32 min_range       # minimum range value [m]~%float32 max_range       # maximum range value [m]~%~%float32 range           # range data [m]~%		 	# (Note: values < range_min or > range_max~%		 	# should be discarded)~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <Range>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     1
     4
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <Range>))
  "Converts a ROS message object to a list"
  (list '<Range>
    (cons ':header (header-val msg))
    (cons ':radiation_type (radiation_type-val msg))
    (cons ':field_of_view (field_of_view-val msg))
    (cons ':min_range (min_range-val msg))
    (cons ':max_range (max_range-val msg))
    (cons ':range (range-val msg))
))
