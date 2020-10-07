;; Auto-generated. Do not edit!


(when (boundp 'node_communicate::message)
  (if (not (find-package "NODE_COMMUNICATE"))
    (make-package "NODE_COMMUNICATE"))
  (shadow 'message (find-package "NODE_COMMUNICATE")))
(unless (find-package "NODE_COMMUNICATE::MESSAGE")
  (make-package "NODE_COMMUNICATE::MESSAGE"))

(in-package "ROS")
;;//! \htmlinclude message.msg.html


(defclass node_communicate::message
  :super ros::object
  :slots (_info ))

(defmethod node_communicate::message
  (:init
   (&key
    ((:info __info) "")
    )
   (send-super :init)
   (setq _info (string __info))
   self)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:serialization-length
   ()
   (+
    ;; string _info
    4 (length _info)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _info
       (write-long (length _info) s) (princ _info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get node_communicate::message :md5sum-) "c10fc26d5cca9a4b9114f5fc5dea9570")
(setf (get node_communicate::message :datatype-) "node_communicate/message")
(setf (get node_communicate::message :definition-)
      "string info

")



(provide :node_communicate/message "c10fc26d5cca9a4b9114f5fc5dea9570")


