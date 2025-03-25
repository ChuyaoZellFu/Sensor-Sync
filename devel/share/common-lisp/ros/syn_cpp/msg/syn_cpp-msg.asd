
(cl:in-package :asdf)

(defsystem "syn_cpp-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Bbox" :depends-on ("_package_Bbox"))
    (:file "_package_Bbox" :depends-on ("_package"))
    (:file "BboxData" :depends-on ("_package_BboxData"))
    (:file "_package_BboxData" :depends-on ("_package"))
    (:file "Track" :depends-on ("_package_Track"))
    (:file "_package_Track" :depends-on ("_package"))
    (:file "Trackbox" :depends-on ("_package_Trackbox"))
    (:file "_package_Trackbox" :depends-on ("_package"))
  ))