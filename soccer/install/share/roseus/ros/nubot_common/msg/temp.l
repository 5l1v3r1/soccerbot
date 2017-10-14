;; Auto-generated. Do not edit!


(when (boundp 'nubot_common::temp)
  (if (not (find-package "NUBOT_COMMON"))
    (make-package "NUBOT_COMMON"))
  (shadow 'temp (find-package "NUBOT_COMMON")))
(unless (find-package "NUBOT_COMMON::TEMP")
  (make-package "NUBOT_COMMON::TEMP"))

(in-package "ROS")
;;//! \htmlinclude temp.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass nubot_common::temp
  :super ros::object
  :slots (_body_name _reference_frame _reference_point _wrench _start_time _duration ))

(defmethod nubot_common::temp
  (:init
   (&key
    ((:body_name __body_name) "")
    ((:reference_frame __reference_frame) "")
    ((:reference_point __reference_point) (instance geometry_msgs::Point :init))
    ((:wrench __wrench) (instance geometry_msgs::Wrench :init))
    ((:start_time __start_time) (instance ros::time :init))
    ((:duration __duration) (instance ros::time :init))
    )
   (send-super :init)
   (setq _body_name (string __body_name))
   (setq _reference_frame (string __reference_frame))
   (setq _reference_point __reference_point)
   (setq _wrench __wrench)
   (setq _start_time __start_time)
   (setq _duration __duration)
   self)
  (:body_name
   (&optional __body_name)
   (if __body_name (setq _body_name __body_name)) _body_name)
  (:reference_frame
   (&optional __reference_frame)
   (if __reference_frame (setq _reference_frame __reference_frame)) _reference_frame)
  (:reference_point
   (&rest __reference_point)
   (if (keywordp (car __reference_point))
       (send* _reference_point __reference_point)
     (progn
       (if __reference_point (setq _reference_point (car __reference_point)))
       _reference_point)))
  (:wrench
   (&rest __wrench)
   (if (keywordp (car __wrench))
       (send* _wrench __wrench)
     (progn
       (if __wrench (setq _wrench (car __wrench)))
       _wrench)))
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:serialization-length
   ()
   (+
    ;; string _body_name
    4 (length _body_name)
    ;; string _reference_frame
    4 (length _reference_frame)
    ;; geometry_msgs/Point _reference_point
    (send _reference_point :serialization-length)
    ;; geometry_msgs/Wrench _wrench
    (send _wrench :serialization-length)
    ;; time _start_time
    8
    ;; duration _duration
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _body_name
       (write-long (length _body_name) s) (princ _body_name s)
     ;; string _reference_frame
       (write-long (length _reference_frame) s) (princ _reference_frame s)
     ;; geometry_msgs/Point _reference_point
       (send _reference_point :serialize s)
     ;; geometry_msgs/Wrench _wrench
       (send _wrench :serialize s)
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; duration _duration
       (write-long (send _duration :sec) s) (write-long (send _duration :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _body_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _body_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _reference_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reference_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _reference_point
     (send _reference_point :deserialize buf ptr-) (incf ptr- (send _reference_point :serialization-length))
   ;; geometry_msgs/Wrench _wrench
     (send _wrench :deserialize buf ptr-) (incf ptr- (send _wrench :serialization-length))
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; duration _duration
     (send _duration :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _duration :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get nubot_common::temp :md5sum-) "e37e6adf97eba5095baa77dffb71e5bd")
(setf (get nubot_common::temp :datatype-) "nubot_common/temp")
(setf (get nubot_common::temp :definition-)
      "string body_name                          # Gazebo body to apply wrench (linear force and torque)
                                          # wrench is applied in the gazebo world by default
                                          # body names are prefixed by model name, e.g. pr2::base_link
string reference_frame                    # wrench is defined in the reference frame of this entity
                                          # use inertial frame if left empty
                                          # frame names are bodies prefixed by model name, e.g. pr2::base_link
geometry_msgs/Point  reference_point      # wrench is defined at this location in the reference frame
geometry_msgs/Wrench wrench               # wrench applied to the origin of the body
time start_time                           # (optional) wrench application start time (seconds)
                                          # if start_time is not specified, or
                                          # start_time < current time, start as soon as possible
duration duration                         # optional duration of wrench application time (seconds)
                                          # if duration < 0, apply wrench continuously without end
                                          # if duration = 0, do nothing
                                          # if duration < step size, apply wrench
                                          # for one step size

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :nubot_common/temp "e37e6adf97eba5095baa77dffb71e5bd")


