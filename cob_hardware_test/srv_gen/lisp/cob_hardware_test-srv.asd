
(cl:in-package :asdf)

(defsystem "cob_hardware_test-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Dialog" :depends-on ("_package_Dialog"))
    (:file "_package_Dialog" :depends-on ("_package"))
  ))