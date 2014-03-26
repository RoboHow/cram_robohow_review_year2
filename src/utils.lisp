;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-robohow-review-year2)

(defun move-arms-up (&key allowed-collision-objects)
  (pr2-manip-pm::execute-move-arm-pose
   :left
   (tf:make-pose-stamped
    "base_link" (roslisp:ros-time)
    (tf:make-3d-vector 0.3 0.5 1.3)
    (tf:euler->quaternion :ax pi))
   :allowed-collision-objects allowed-collision-objects)
  (pr2-manip-pm::execute-move-arm-pose
   :right
   (tf:make-pose-stamped
    "base_link" (roslisp:ros-time)
    (tf:make-3d-vector 0.3 -0.5 1.3)
    (tf:euler->quaternion :ax pi))
   :allowed-collision-objects allowed-collision-objects))

(defun make-handles (distance-from-center
                     &key
                       (segments 1)
                       (starting-angle 0.0)
                       (ax 0.0) (ay 0.0) (az 0.0)
                       (offset-angle 0.0)
                       grasp-type
                       (center-offset
                        (tf:make-identity-vector)))
  (loop for i from 0 below segments
        as current-angle = (+ (* 2 pi (float (+ (/ i segments) starting-angle)))
                              offset-angle)
        as handle-pose = (tf:make-pose
                          (tf:make-3d-vector
                           (+ (* distance-from-center (cos current-angle))
                              (tf:x center-offset))
                           (+ (* distance-from-center (sin current-angle))
                              (tf:y center-offset))
                           (+ 0.0
                              (tf:z center-offset)))
                          (tf:euler->quaternion
                           :ax ax :ay ay :az (+ az current-angle)))
        as handle-object = (make-designator
                            'cram-designators:object
                            (append
                             `((desig-props:type desig-props:handle)
                               (desig-props:at
                                ,(a location `((desig-props:pose
                                                ,handle-pose)))))
                             (when grasp-type
                               `((desig-props:grasp-type ,grasp-type)))))
        collect handle-object))

(defgeneric init-ms-belief-state (&key debug-window objects))

(defmethod init-ms-belief-state (&key debug-window objects)
  (crs:prolog `(btr:clear-bullet-world))
  (let* ((robot-pose (get-robot-pose))
         (urdf-robot
           (cl-urdf:parse-urdf
            (roslisp:get-param "robot_description_lowres")))
         (urdf-kitchen
           (cl-urdf:parse-urdf
            (roslisp:get-param "kitchen_description")))
         (scene-rot-quaternion (tf:euler->quaternion :az pi))
         (scene-rot `(,(tf:x scene-rot-quaternion)
                      ,(tf:y scene-rot-quaternion)
                      ,(tf:z scene-rot-quaternion)
                      ,(tf:w scene-rot-quaternion)))
         (scene-trans `(-3.45 -4.35 0))
         (robot-pose robot-pose)
         (robot-rot `(,(tf:x (tf:orientation robot-pose))
                      ,(tf:y (tf:orientation robot-pose))
                      ,(tf:z (tf:orientation robot-pose))
                      ,(tf:w (tf:orientation robot-pose))))
         (robot-trans `(,(tf:x (tf:origin robot-pose))
                        ,(tf:y (tf:origin robot-pose))
                        ,(tf:z (tf:origin robot-pose))))
         (bdgs
           (car
            (force-ll
             (crs:prolog
              `(and (btr:clear-bullet-world)
                    (btr:bullet-world ?w)
                    (btr:assert (btr:object
                                 ?w btr:static-plane floor
                                 ((0 0 0) (0 0 0 1))
                                 :normal (0 0 1) :constant 0))
                    ,@(when debug-window
                       `((btr:debug-window ?w)))
                    (btr:robot ?robot)
                    ,@(loop for object in objects
                            for obj-name = (car object)
                            for obj-pose = (cdr object)
                            collect `(btr:assert
                                      (btr:object
                                       ?w btr:box ,obj-name
                                       ((,(tf:x (tf:origin obj-pose))
                                         ,(tf:y (tf:origin obj-pose))
                                         ,(tf:z (tf:origin obj-pose)))
                                        (,(tf:x (tf:orientation obj-pose))
                                         ,(tf:y (tf:orientation obj-pose))
                                         ,(tf:z (tf:orientation obj-pose))
                                         ,(tf:w (tf:orientation obj-pose))))
                                       :mass 0.0 :size (0.1 0.1 0.1))))
                    (assert (btr:object
                             ?w btr:urdf ?robot
                             (,robot-trans ,robot-rot)
                             :urdf ,urdf-robot))
                    (assert (btr:object
                             ?w btr:semantic-map sem-map-kitchen
                             (,scene-trans ,scene-rot)
                             :urdf ,urdf-kitchen))))))))
    (var-value
     '?pr2
     (lazy-car
      (crs:prolog
       `(and (btr:robot ?robot)
             (btr:%object ?w ?robot ?pr2)) bdgs)))
    (var-value
     '?sem-map
     (lazy-car
      (crs:prolog
       `(btr:%object ?w sem-map-kitchen ?sem-map) bdgs)))))

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        robosherlock-process-module:robosherlock-process-module)
     ,@body))

(defun get-robot-pose ()
  (let ((time (roslisp:ros-time)))
    (tf:wait-for-transform
     *tf* :time time
          :source-frame "/map"
          :target-frame "/base_link")
    (tf:transform-pose
     *tf* :pose (tf:make-pose-stamped
                 "/base_link"
                 time
                 (tf:make-identity-vector)
                 (tf:make-identity-rotation))
     :target-frame "/map")))

(defun drive-to-pose (pose-stamped)
  (block nav
    (with-designators ((loc (location `((desig-props:pose ,pose-stamped))))
                       (act (action `((desig-props:type desig-props:navigation)
                                      (desig-props:goal ,loc)))))
      (cpl:with-failure-handling ((cram-plan-failures:location-not-reached-failure (f)
                                (declare (ignore f))
                                (return-from nav)))
        (perform act)))))
