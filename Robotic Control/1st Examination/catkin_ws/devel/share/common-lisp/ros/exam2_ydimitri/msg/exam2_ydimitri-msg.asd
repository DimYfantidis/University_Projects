
(cl:in-package :asdf)

(defsystem "exam2_ydimitri-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TurtleAction" :depends-on ("_package_TurtleAction"))
    (:file "_package_TurtleAction" :depends-on ("_package"))
    (:file "TurtleActionFeedback" :depends-on ("_package_TurtleActionFeedback"))
    (:file "_package_TurtleActionFeedback" :depends-on ("_package"))
    (:file "TurtleActionGoal" :depends-on ("_package_TurtleActionGoal"))
    (:file "_package_TurtleActionGoal" :depends-on ("_package"))
    (:file "TurtleActionResult" :depends-on ("_package_TurtleActionResult"))
    (:file "_package_TurtleActionResult" :depends-on ("_package"))
    (:file "TurtleFeedback" :depends-on ("_package_TurtleFeedback"))
    (:file "_package_TurtleFeedback" :depends-on ("_package"))
    (:file "TurtleGoal" :depends-on ("_package_TurtleGoal"))
    (:file "_package_TurtleGoal" :depends-on ("_package"))
    (:file "TurtleResult" :depends-on ("_package_TurtleResult"))
    (:file "_package_TurtleResult" :depends-on ("_package"))
  ))