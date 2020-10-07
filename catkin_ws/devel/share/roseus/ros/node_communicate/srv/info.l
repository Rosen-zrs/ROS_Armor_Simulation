;; Auto-generated. Do not edit!


(when (boundp 'node_communicate::info)
  (if (not (find-package "NODE_COMMUNICATE"))
    (make-package "NODE_COMMUNICATE"))
  (shadow 'info (find-package "NODE_COMMUNICATE")))
(unless (find-package "NODE_COMMUNICATE::INFO")
  (make-package "NODE_COMMUNICATE::INFO"))
(unless (find-package "NODE_COMMUNICATE::INFOREQUEST")
  (make-package "NODE_COMMUNICATE::INFOREQUEST"))
(unless (find-package "NODE_COMMUNICATE::INFORESPONSE")
  (make-package "NODE_COMMUNICATE::INFORESPONSE"))

(in-package "ROS")





(defclass node_communicate::infoRequest
  :super ros::object
  :slots (_request ))

(defmethod node_communicate::infoRequest
  (:init
   (&key
    ((:request __request) "")
    )
   (send-super :init)
   (setq _request (string __request))
   self)
  (:request
   (&optional __request)
   (if __request (setq _request __request)) _request)
  (:serialization-length
   ()
   (+
    ;; string _request
    4 (length _request)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _request
       (write-long (length _request) s) (princ _request s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _request
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _request (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass node_communicate::infoResponse
  :super ros::object
  :slots (_response ))

(defmethod node_communicate::infoResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass node_communicate::info
  :super ros::object
  :slots ())

(setf (get node_communicate::info :md5sum-) "33ea4e5aeb30f5913da681ca459d55f3")
(setf (get node_communicate::info :datatype-) "node_communicate/info")
(setf (get node_communicate::info :request) node_communicate::infoRequest)
(setf (get node_communicate::info :response) node_communicate::infoResponse)

(defmethod node_communicate::infoRequest
  (:response () (instance node_communicate::infoResponse :init)))

(setf (get node_communicate::infoRequest :md5sum-) "33ea4e5aeb30f5913da681ca459d55f3")
(setf (get node_communicate::infoRequest :datatype-) "node_communicate/infoRequest")
(setf (get node_communicate::infoRequest :definition-)
      "string request
---
string response

")

(setf (get node_communicate::infoResponse :md5sum-) "33ea4e5aeb30f5913da681ca459d55f3")
(setf (get node_communicate::infoResponse :datatype-) "node_communicate/infoResponse")
(setf (get node_communicate::infoResponse :definition-)
      "string request
---
string response

")



(provide :node_communicate/info "33ea4e5aeb30f5913da681ca459d55f3")


