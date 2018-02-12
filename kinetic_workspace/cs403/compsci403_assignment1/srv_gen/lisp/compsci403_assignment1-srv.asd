
(cl:in-package :asdf)

(defsystem "compsci403_assignment1-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Assignment1ProviderSrv" :depends-on ("_package_Assignment1ProviderSrv"))
    (:file "_package_Assignment1ProviderSrv" :depends-on ("_package"))
    (:file "Assignment1CallerSrv" :depends-on ("_package_Assignment1CallerSrv"))
    (:file "_package_Assignment1CallerSrv" :depends-on ("_package"))
  ))