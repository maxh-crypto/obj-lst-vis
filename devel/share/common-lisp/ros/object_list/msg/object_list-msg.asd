
(cl:in-package :asdf)

(defsystem "object_list-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Classification" :depends-on ("_package_Classification"))
    (:file "_package_Classification" :depends-on ("_package"))
    (:file "Dimension" :depends-on ("_package_Dimension"))
    (:file "_package_Dimension" :depends-on ("_package"))
    (:file "Features" :depends-on ("_package_Features"))
    (:file "_package_Features" :depends-on ("_package"))
    (:file "Geometric" :depends-on ("_package_Geometric"))
    (:file "_package_Geometric" :depends-on ("_package"))
    (:file "ObjectList" :depends-on ("_package_ObjectList"))
    (:file "_package_ObjectList" :depends-on ("_package"))
    (:file "ObjectsList" :depends-on ("_package_ObjectsList"))
    (:file "_package_ObjectsList" :depends-on ("_package"))
  ))