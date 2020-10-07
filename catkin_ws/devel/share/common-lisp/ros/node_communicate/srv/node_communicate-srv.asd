
(cl:in-package :asdf)

(defsystem "node_communicate-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "info" :depends-on ("_package_info"))
    (:file "_package_info" :depends-on ("_package"))
  ))