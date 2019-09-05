
(in-package :asdf)

(defsystem "drrobot_jaguar4x4_player-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "CustomSensor" :depends-on ("_package"))
    (:file "_package_CustomSensor" :depends-on ("_package"))
    (:file "RangeArray" :depends-on ("_package"))
    (:file "_package_RangeArray" :depends-on ("_package"))
    (:file "Range" :depends-on ("_package"))
    (:file "_package_Range" :depends-on ("_package"))
    (:file "MotorInfoArray" :depends-on ("_package"))
    (:file "_package_MotorInfoArray" :depends-on ("_package"))
    (:file "StandardSensor" :depends-on ("_package"))
    (:file "_package_StandardSensor" :depends-on ("_package"))
    (:file "PowerInfo" :depends-on ("_package"))
    (:file "_package_PowerInfo" :depends-on ("_package"))
    (:file "MotorInfo" :depends-on ("_package"))
    (:file "_package_MotorInfo" :depends-on ("_package"))
    ))
