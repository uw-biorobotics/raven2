
(cl:in-package :asdf)

(defsystem "raven_2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "raven_automove" :depends-on ("_package_raven_automove"))
    (:file "_package_raven_automove" :depends-on ("_package"))
    (:file "raven_state" :depends-on ("_package_raven_state"))
    (:file "_package_raven_state" :depends-on ("_package"))
  ))