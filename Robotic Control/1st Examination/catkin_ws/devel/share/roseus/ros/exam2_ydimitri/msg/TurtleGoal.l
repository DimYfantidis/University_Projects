;; Auto-generated. Do not edit!


(when (boundp 'exam2_ydimitri::TurtleGoal)
  (if (not (find-package "EXAM2_YDIMITRI"))
    (make-package "EXAM2_YDIMITRI"))
  (shadow 'TurtleGoal (find-package "EXAM2_YDIMITRI")))
(unless (find-package "EXAM2_YDIMITRI::TURTLEGOAL")
  (make-package "EXAM2_YDIMITRI::TURTLEGOAL"))

(in-package "ROS")
;;//! \htmlinclude TurtleGoal.msg.html


(defclass exam2_ydimitri::TurtleGoal
  :super ros::object
  :slots (_goal_x _goal_y ))

(defmethod exam2_ydimitri::TurtleGoal
  (:init
   (&key
    ((:goal_x __goal_x) 0.0)
    ((:goal_y __goal_y) 0.0)
    )
   (send-super :init)
   (setq _goal_x (float __goal_x))
   (setq _goal_y (float __goal_y))
   self)
  (:goal_x
   (&optional __goal_x)
   (if __goal_x (setq _goal_x __goal_x)) _goal_x)
  (:goal_y
   (&optional __goal_y)
   (if __goal_y (setq _goal_y __goal_y)) _goal_y)
  (:serialization-length
   ()
   (+
    ;; float32 _goal_x
    4
    ;; float32 _goal_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _goal_x
       (sys::poke _goal_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _goal_y
       (sys::poke _goal_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _goal_x
     (setq _goal_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _goal_y
     (setq _goal_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get exam2_ydimitri::TurtleGoal :md5sum-) "9aa8c2e2b21fc8544d27640eeac21d97")
(setf (get exam2_ydimitri::TurtleGoal :datatype-) "exam2_ydimitri/TurtleGoal")
(setf (get exam2_ydimitri::TurtleGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
float32 goal_x
float32 goal_y

")



(provide :exam2_ydimitri/TurtleGoal "9aa8c2e2b21fc8544d27640eeac21d97")


