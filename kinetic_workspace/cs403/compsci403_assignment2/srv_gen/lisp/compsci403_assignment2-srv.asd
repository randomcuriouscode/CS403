
(cl:in-package :asdf)

(defsystem "compsci403_assignment2-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Get3DPointFromDisparitySrv" :depends-on ("_package_Get3DPointFromDisparitySrv"))
    (:file "_package_Get3DPointFromDisparitySrv" :depends-on ("_package"))
    (:file "GetPixelFrom3DPointSrv" :depends-on ("_package_GetPixelFrom3DPointSrv"))
    (:file "_package_GetPixelFrom3DPointSrv" :depends-on ("_package"))
    (:file "GetDepthFromDisparitySrv" :depends-on ("_package_GetDepthFromDisparitySrv"))
    (:file "_package_GetDepthFromDisparitySrv" :depends-on ("_package"))
    (:file "GetIntrinsicsSrv" :depends-on ("_package_GetIntrinsicsSrv"))
    (:file "_package_GetIntrinsicsSrv" :depends-on ("_package"))
    (:file "Get3DPointFromDepthSrv" :depends-on ("_package_Get3DPointFromDepthSrv"))
    (:file "_package_Get3DPointFromDepthSrv" :depends-on ("_package"))
  ))