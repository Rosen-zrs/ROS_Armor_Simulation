
(cl:in-package :asdf)

(defsystem "node_communicate-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Person" :depends-on ("_package_Person"))
    (:file "_package_Person" :depends-on ("_package"))
    (:file "message" :depends-on ("_package_message"))
    (:file "_package_message" :depends-on ("_package"))
  ))