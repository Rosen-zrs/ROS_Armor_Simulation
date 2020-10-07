; Auto-generated. Do not edit!


(cl:in-package node_communicate-srv)


;//! \htmlinclude info-request.msg.html

(cl:defclass <info-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:string
    :initform ""))
)

(cl:defclass info-request (<info-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <info-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'info-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name node_communicate-srv:<info-request> is deprecated: use node_communicate-srv:info-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <info-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader node_communicate-srv:request-val is deprecated.  Use node_communicate-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <info-request>) ostream)
  "Serializes a message object of type '<info-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <info-request>) istream)
  "Deserializes a message object of type '<info-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<info-request>)))
  "Returns string type for a service object of type '<info-request>"
  "node_communicate/infoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'info-request)))
  "Returns string type for a service object of type 'info-request"
  "node_communicate/infoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<info-request>)))
  "Returns md5sum for a message object of type '<info-request>"
  "33ea4e5aeb30f5913da681ca459d55f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'info-request)))
  "Returns md5sum for a message object of type 'info-request"
  "33ea4e5aeb30f5913da681ca459d55f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<info-request>)))
  "Returns full string definition for message of type '<info-request>"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'info-request)))
  "Returns full string definition for message of type 'info-request"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <info-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'request))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <info-request>))
  "Converts a ROS message object to a list"
  (cl:list 'info-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude info-response.msg.html

(cl:defclass <info-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass info-response (<info-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <info-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'info-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name node_communicate-srv:<info-response> is deprecated: use node_communicate-srv:info-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <info-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader node_communicate-srv:response-val is deprecated.  Use node_communicate-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <info-response>) ostream)
  "Serializes a message object of type '<info-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <info-response>) istream)
  "Deserializes a message object of type '<info-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<info-response>)))
  "Returns string type for a service object of type '<info-response>"
  "node_communicate/infoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'info-response)))
  "Returns string type for a service object of type 'info-response"
  "node_communicate/infoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<info-response>)))
  "Returns md5sum for a message object of type '<info-response>"
  "33ea4e5aeb30f5913da681ca459d55f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'info-response)))
  "Returns md5sum for a message object of type 'info-response"
  "33ea4e5aeb30f5913da681ca459d55f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<info-response>)))
  "Returns full string definition for message of type '<info-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'info-response)))
  "Returns full string definition for message of type 'info-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <info-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <info-response>))
  "Converts a ROS message object to a list"
  (cl:list 'info-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'info)))
  'info-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'info)))
  'info-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'info)))
  "Returns string type for a service object of type '<info>"
  "node_communicate/info")