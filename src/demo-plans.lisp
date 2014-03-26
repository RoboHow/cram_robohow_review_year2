;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(def-top-level-cram-function robohow-demo-plan (&key (put-down t))
  ;; NOTE(winkler): This validator breaks IK based `to reach' and `to
  ;; see' location resolution. Disabling it, since everything works
  ;; just nicely without it. Gotta look into this later.
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::check-ik-solution)
  ;; (cram-designators:disable-location-validation-function
  ;;  'bullet-reasoning-designators::validate-designator-solution)
  (cram-uima::config-uima)
  ;; Setting the timeout for action server responses to a high
  ;; value. Otherwise, the (very long, > 2.0 seconds) motion planning
  ;; process will just drop the connection and never execute.
  (setf actionlib::*action-server-timeout* 20)
  (beliefstate::enable-logging t)
  (with-process-modules
    (init-ms-belief-state :debug-window t)
    (let ((object-source "kitchen_sink_block")
          (object-destination "kitchen_island"))
      (with-designators
          ((loc (location
                 `((desig-props:on Cupboard)
                   (desig-props:name ,object-source))))
           (spatula-1 (object `((desig-props:grasp-type desig-props:top-slide-down)
                                (desig-props:type desig-props:spatula)
                                (desig-props:at ,loc)
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
                                    :center-offset (tf:make-3d-vector 0.23 0 0.02))))))
           (spatula-2 (object (append (remove 'name (description spatula-1)
                                              :test (lambda (x y) (eql x (car y))))
                                      `())))
           (pancake-mix (object `((desig-props:at ,loc)
                                  (type desig-props:object)
                                  (desig-props:max-handles 1)
                                  ,@(mapcar
                                     (lambda (handle-object)
                                       `(desig-props:handle ,handle-object))
                                     (make-handles
                                      0.04
                                      :segments 8
                                      :ax (/ pi 2)
                                      :center-offset
                                      (tf:make-3d-vector 0.02 0.0 0.0))))))
           (milk-box (object `((desig-props:at ,loc)
                               (desig-props:max-handles 1)
                               ,@(mapcar
                                  (lambda (handle-object)
                                    `(desig-props:handle ,handle-object))
                                  (make-handles
                                   0.04
                                   :segments 8
                                   :ax (/ pi 2)
                                   :center-offset
                                   (tf:make-3d-vector 0.02 0.0 0.0)))))))
        (let ((objects `(,spatula-1))); ,pancake-mix))); ,spatula-1 ,spatula-2 ,milk-box ,pancake-mix)))
          (dolist (object-prototype objects)
            (moveit:clear-collision-environment)
            (sem-map-coll-env:publish-semantic-map-collision-objects)
            (sem-map-coll-env:publish-semantic-map-collision-objects)
            (cpl:with-failure-handling
                ((cram-plan-failures:manipulation-failure (f)
                   (declare (ignore f))
                   (cpl:retry)))
              (move-arms-up))
            (cpl:with-failure-handling
                ((cram-plan-failures:manipulation-pose-unreachable (f)
                   (declare (ignore f))
                   (move-arms-up)
                   (retry))
                 (cram-plan-failures:location-not-reached-failure (f)
                   (declare (ignore f))
                   (ros-warn (pick it up) "Cannot reach location. Retrying.")
                   (retry))
                 (cram-plan-failures:object-not-found (f)
                   (declare (ignore f))
                   (ros-warn (pick it up) "Object not found. Retrying.")
                   (retry)))
              (with-designators ((object (object (description object-prototype))))
                (let ((obj-in-hand (achieve `(cram-plan-library:object-in-hand ,object))))
                  (when put-down
                    (with-designators
                        ((put-down-loc (location
                                        `((desig-props:on Cupboard)
                                          (desig-props:name ,object-destination)))))
                      (cpl:with-failure-handling
                          ((cram-plan-failures:manipulation-pose-unreachable (f)
                             (declare (ignore f))
                             (cram-plan-library::retry-with-updated-location
                              put-down-loc (next-solution put-down-loc)))
                           (cram-plan-failures:location-not-reached-failure (f)
                             (declare (ignore f))
                             (ros-warn (pick it up)
                                       "Cannot reach location. Retrying.")
                             (retry)))
                        (achieve `(cram-plan-library:object-placed-at
                                   ,object ,put-down-loc))
                        (move-arms-up))))
                  (unless put-down obj-in-hand))))))))))

(defun drive-to-pancake-pose ()
  (drive-to-pose (tf:make-pose-stamped
                  "/map" (roslisp:ros-time)
                  (tf:make-3d-vector -0.2352 0.0133 -0.003)
                  (tf:make-quaternion 0.00357 0 -0.9996 0.02914))))
