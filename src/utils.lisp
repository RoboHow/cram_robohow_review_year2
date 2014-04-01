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

(defparameter *loc-on-sink-counter* nil)
(defparameter *loc-on-kitchen-island* nil)
(defparameter *loc-on-pancake-table* nil)
(defparameter *loc-putdown-spatula-left* nil)
(defparameter *loc-putdown-spatula-right* nil)
(defparameter *loc-putdown-pancake-mix* nil)
(defparameter *pancake-maker* nil)
(defparameter *spatula-left* nil)
(defparameter *spatula-right* nil)
(defparameter *pancake-mix* nil)
(defparameter *pancake* nil)

(defmacro perceive-a (object &key stationary)
  `(cpl:with-failure-handling
       ((cram-plan-failures:object-not-found (f)
          (declare (ignore f))
          (ros-warn (pick-and-place) "Object not found. Retrying.")
          (cpl:retry)))
     (cond (,stationary
            (let ((at (desig-prop-value ,object 'desig-props:at)))
              (achieve `(cram-plan-library:looking-at ,(reference at)))
              (first (perceive-object
                      'cram-plan-library:currently-visible
                      ,object))))
           (t (perceive-object 'cram-plan-library:a ,object)))))

(defmacro pick-object (object &key stationary)
  `(cpl:with-failure-handling
       ((cram-plan-failures:manipulation-failure (f)
          (declare (ignore f))
          (ensure-arms-up)
          (cpl:retry))
        (cram-plan-failures:location-not-reached-failure (f)
          (declare (ignore f))
          (ros-warn (pick-and-place) "Cannot reach location. Retrying.")
          (cpl:retry))
        (cram-plan-failures:object-not-found (f)
          (declare (ignore f))
          (ros-warn (pick-and-place) "Object not found. Retrying.")
          (cpl:retry)))
     ,(cond (stationary
             `(achieve `(cram-plan-library:object-picked ,,object)))
            (t
             `(achieve `(cram-plan-library:object-in-hand ,,object))))))

(defmacro place-object (object location &key stationary)
  `(cpl:with-failure-handling
       ((cram-plan-failures:manipulation-pose-unreachable (f)
          (declare (ignore f))
          (cram-plan-library::retry-with-updated-location
           ,location (next-solution ,location)))
        (cram-plan-failures:location-not-reached-failure (f)
          (declare (ignore f))
          (ros-warn (pick-and-place) "Cannot reach location. Retrying.")
          (cpl:retry)))
     (let ((side (var-value
                  '?side
                  (lazy-car (crs:prolog `(cram-plan-library:object-in-hand ,,object ?side))))))
       ,(cond (stationary
               `(achieve `(cram-plan-library::object-put ,,object ,,location)))
              (t
               `(achieve `(cram-plan-library::object-placed-at ,,object ,,location))))
       (ensure-arms-up side))))

(defun prepare-global-designators (&key pancake-manipulation-only)
  (setf *loc-on-sink-counter*
        (make-designator
         'location
         `((on Cupboard)
           (name "kitchen_sink_block"))))
  (setf *loc-on-kitchen-island*
        (make-designator
         'location
         `((on Cupboard)
           (name "kitchen_island"))))
  (setf *loc-on-pancake-table*
        (make-designator
         'location
         `((on Cupboard)
           (name "pancake_table"))))
  (setf *pancake-maker*
        (make-designator
         'object
         `((at ,*loc-on-pancake-table*)
           (type pancakemaker))))
  (setf *pancake*
        (make-designator
         'object
         `((at ,(make-designator 'location
                                 `((on ,*pancake-maker*))))
           (type pancake))))
  (setf *loc-putdown-spatula-left*
        (make-designator
         'location
         (append (description *loc-on-pancake-table*)
                 `((desig-props:near desig-props::pancakemaker0)
                   (desig-props:left-of desig-props::pancakemaker0)
                   (desig-props:for desig-props::spatula0)
                   (desig-props:on Cupboard)
                   (desig-props:name "pancake_table")))))
  (setf *loc-putdown-spatula-right*
        (make-designator
         'location
         (append (description *loc-on-pancake-table*)
                 `((desig-props:near desig-props::pancakemaker0)
                   (desig-props:right-of desig-props::pancakemaker0)
                   (desig-props:for desig-props::spatula0)
                   (desig-props:on Cupboard)
                   (desig-props:name "pancake_table")))))
  (setf *loc-putdown-pancake-mix*
        (make-designator
         'location
         (append (description *loc-on-kitchen-island*))))
  (setf *pancake-mix*
        (make-designator
         'object
         `((at ,(cond (pancake-manipulation-only *loc-putdown-pancake-mix*)
                      (t *loc-on-sink-counter*)))
           (type pancakemix)
           (max-handles 1)
           ,@(mapcar
              (lambda (handle-object)
                `(handle ,handle-object))
              (make-handles
               0.04
               :segments 2
               :offset-angle (/ pi 2)
               :ax (/ pi 2)
               :center-offset
               (tf:make-3d-vector 0.02 0.0 0.0))))))
  (setf *spatula-left*
        (make-designator
         'object
         `((at ,(cond (pancake-manipulation-only *loc-putdown-spatula-left*)
                      (t *loc-on-kitchen-island*)))
           (type spatula)
           (desig-props:grasp-type desig-props:top-slide-down)
           (max-handles 1)
           ,@(mapcar
              (lambda (handle-object)
                `(handle ,handle-object))
              (make-handles
               0.04
               :ax (/ pi 2)
               :grasp-type 'desig-props:top-slide-down
               :center-offset (tf:make-3d-vector -0.23 0 0.02)))
           ,@(mapcar
              (lambda (handle-object)
                `(handle ,handle-object))
              (make-handles
               0.04
               :ax (/ pi 2)
               :offset-angle pi
               :grasp-type 'desig-props:top-slide-down
               :center-offset (tf:make-3d-vector 0.23 0 0.02))))))
  (setf *spatula-right*
        (make-designator
         'object
         `((at ,(cond (pancake-manipulation-only *loc-putdown-spatula-right*)
                      (t *loc-on-sink-counter*)))
           (type spatula)
           (desig-props:grasp-type desig-props:top-slide-down)
           (max-handles 1)
           ,@(mapcar
              (lambda (handle-object)
                `(handle ,handle-object))
              (make-handles
               0.04
               :ax (/ pi 2)
               :grasp-type 'desig-props:top-slide-down
               :center-offset (tf:make-3d-vector -0.23 0 0.02)))
           ,@(mapcar
              (lambda (handle-object)
                `(handle ,handle-object))
              (make-handles
               0.04
               :ax (/ pi 2)
               :offset-angle pi
               :grasp-type 'desig-props:top-slide-down
               :center-offset (tf:make-3d-vector 0.23 0 0.02)))))))

(defun drive-to-pancake-pose-far ()
  (drive-to-pose (tf:make-pose-stamped
                  "/map" (roslisp:ros-time)
                  (tf:make-3d-vector -0.3 -0.2 0.0)
                  (tf:euler->quaternion :az pi))))

(defun drive-to-pancake-pose-close ()
  (drive-to-pose (tf:make-pose-stamped
                  "/map" (roslisp:ros-time)
                  (tf:make-3d-vector -0.5 -0.2 0.0)
                  (tf:euler->quaternion :az pi))))

(defun face-location (location)
  (let* ((current-pose (get-robot-pose))
         (curr-origin (tf:origin current-pose))
         (loc-pose (reference location))
         (loc-origin (tf:origin loc-pose)))
    (let* ((dx (- (tf:x loc-origin) (tf:x curr-origin)))
           (dy (- (tf:y loc-origin) (tf:y curr-origin)))
           (l (sqrt (+ (* dx dx) (* dy dy))))
           (angle (acos (/ dx l))))
      (drive-to-pose (tf:copy-pose-stamped
                      current-pose
                      :orientation (tf:euler->quaternion :az (- angle))
                      :stamp (roslisp:ros-time))))))

(defun move-arms-up (&key allowed-collision-objects side)
  (when (or (eql side :left) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :left
     (tf:make-pose-stamped
      "base_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.3 0.5 1.3)
      (tf:euler->quaternion :ax pi))
     :allowed-collision-objects allowed-collision-objects))
  (when (or (eql side :right) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :right
     (tf:make-pose-stamped
      "base_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.3 -0.5 1.3)
      (tf:euler->quaternion :ax pi))
     :allowed-collision-objects allowed-collision-objects)))

(defun make-handles (distance-from-center
                     &key
                       (segments 1)
                       (ax 0.0) (ay 0.0) (az 0.0)
                       (offset-angle 0.0)
                       grasp-type
                       (center-offset
                        (tf:make-identity-vector)))
  (loop for i from 0 below segments
        as current-angle = (+ (* 2 pi (float (/ i segments)))
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
        (cpl:pursue
          (cpl:sleep 5)
          (perform act))))))

(def-top-level-cram-function see-object (description)
  (with-process-modules
    (with-designators ((obj (object description)))
      (cram-plan-library:perceive-object
       'cram-plan-library:currently-visible obj))))

(defun ensure-arms-up (&optional side)
  (cpl:with-failure-handling
      ((cram-plan-failures:manipulation-failure (f)
         (declare (ignore f))
         (cpl:retry)))
    (move-arms-up :side side)))

(defun renew-collision-environment ()
  (moveit:clear-collision-environment)
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (sem-map-coll-env:publish-semantic-map-collision-objects))

(defun make-spatula-designator (location &key forced-side)
  (with-designators
      ((spatula (object (append
                         `((desig-props:grasp-type desig-props:top-slide-down)
                           (desig-props:type desig-props:spatula)
                           (desig-props:at ,location)
                           (desig-props:max-handles 1)
                           ,@(mapcar
                              (lambda (handle-object)
                                `(desig-props:handle ,handle-object))
                              (make-handles
                               0.04
                               :ax (/ pi 2)
                               :grasp-type 'desig-props:top-slide-down
                               :center-offset (tf:make-3d-vector -0.23 0 0.02)))
                           ,@(mapcar
                              (lambda (handle-object)
                                `(desig-props:handle ,handle-object))
                              (make-handles
                               0.04
                               :ax (/ pi 2)
                               :offset-angle pi
                               :grasp-type 'desig-props:top-slide-down
                               :center-offset (tf:make-3d-vector 0.23 0 0.02))))
                         (when forced-side
                           `((desig-props:side ,forced-side)))))))
    spatula))

(defun prepare-settings ()
  ;; NOTE(winkler): This validator breaks IK based `to reach' and `to
  ;; see' location resolution. Disabling it, since everything works
  ;; just nicely without it. Gotta look into this later.
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::check-ik-solution)
  (cram-designators:disable-location-validation-function
   'spatial-relations-costmap::potential-field-costmap-pose-function)
  (cram-designators:disable-location-validation-function
   'spatial-relations-costmap::collision-pose-validator)
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::validate-designator-solution)
  (cram-uima::config-uima)
  ;; Setting the timeout for action server responses to a high
  ;; value. Otherwise, the (very long, > 2.0 seconds) motion planning
  ;; process will just drop the connection and never execute.
  (setf actionlib::*action-server-timeout* 20)
  (beliefstate::enable-logging t)
  (init-ms-belief-state :debug-window t)
  (setf btr::*bb-comparison-validity-threshold* 0.03)
  (moveit:clear-collision-environment)
  ;; Twice, because sometimes a ROS message for an object gets lost.
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (sem-map-coll-env:publish-semantic-map-collision-objects))

(defun set-initial-joint-values-pouring ()
  (moveit::move-link-joint-states
   "right_arm" `(,(cons "r_upper_arm_roll_joint" -1.392565491097796)
                 ,(cons "r_shoulder_pan_joint" -1.0650093105988152)
                 ,(cons "r_shoulder_lift_joint" 0.26376743371555295)
                 ,(cons "r_forearm_roll_joint" -0.524)
                 ,(cons "r_elbow_flex_joint" -1.629946646305397)
                 ,(cons "r_wrist_flex_joint" -0.9668414952685922)
                 ,(cons "r_wrist_roll_joint" 1.8614))))

(defun set-initial-joint-values-flipping ()
  (moveit::move-link-joint-states
   "right_arm" `(,(cons "r_upper_arm_roll_joint" -1.32)
                 ,(cons "r_shoulder_pan_joint" -1.08)
                 ,(cons "r_shoulder_lift_joint" 0.16)
                 ,(cons "r_forearm_roll_joint" 0.0)
                 ,(cons "r_elbow_flex_joint" -1.14)
                 ,(cons "r_wrist_flex_joint" -1.05)
                 ,(cons "r_wrist_roll_joint" 1.57)))
  (moveit::move-link-joint-states
   "left_arm" `(,(cons "l_upper_arm_roll_joint" 1.32)
                ,(cons "l_shoulder_pan_joint" 1.08)
                ,(cons "l_shoulder_lift_joint" 0.16)
                ,(cons "l_forearm_roll_joint" 0.0)
                ,(cons "l_elbow_flex_joint" -1.14)
                ,(cons "l_wrist_flex_joint" -1.05)
                ,(cons "l_wrist_roll_joint" 1.57))))
