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

;;
;; To run everyting, start the main top-level function:
;; > (demo)
;; 
;; Everything else should be handled correctly. If you want to only
;; run a part of it, you can do so this way:
;; 
;; - Only pick and place:
;;   > (demo :pancake-manipulation nil)
;;
;; - Only pancake manipulation (in this case, the designators will be
;;   initialized for the correct initial position):
;;   > (demo :pick-and-place nil)
;;

(def-top-level-cram-function demo (&key (pick-and-place t) (pancake-manipulation t))
  (prepare-settings)
  (with-process-modules
    (prepare-global-designators :pancake-manipulation-only (not pick-and-place))
    (when pick-and-place
      (pick-and-place))
    (when pancake-manipulation
      (pancake-manipulation))))

(def-cram-function pick-and-place ()
  ;; Well-defined home-pose
  (ensure-arms-up)
  ;; Get the first spatula from the kitchen island
  (pick-object *spatula-left*)
  ;; Approach the pancake table and check where the pancake maker is
  (drive-to-pancake-pose-far)
  (perceive-a *pancake-maker*)
  ;; Put the spatula down to the left of the pancake maker
  (place-object *spatula-left* *loc-putdown-spatula-left*)
  ;; Get the pancake mix and the second spatula from the sink block
  (pick-object *pancake-mix*)
  (pick-object *spatula-right*)
  ;; Place the spatula to the right of the pancake maker and the
  ;; pancake mix on the kitchen island, near to the right spatula
  (place-object *spatula-right* *loc-putdown-spatula-right*)
  (place-object *pancake-mix* *loc-putdown-pancake-mix*)
  ;; Well-defined home-pose
  (ensure-arms-up))

(def-cram-function pancake-manipulation ()
  ;; Well-defined home-pose
  (ensure-arms-up)
  ;; Position in front of pancake table (far)
  (drive-to-pancake-pose-far)
  ;; Check for pancake maker (in case we didn't perceive it before due
  ;; to not doing the pick and place part)
  (unless (desig:newest-effective-designator *pancake-maker*)
    (perceive-a *pancake-maker*))
  ;; Grasp the pancake mix
  (with-designators ((pancake-mix (object (append (description (current-desig *pancake-mix*))
                                                  `((side :left))))))
    (equate *pancake-mix* pancake-mix)
    (pick-object (current-desig pancake-mix) :stationary t)
    ;; Position in front of pancake table (close)
    (drive-to-pancake-pose-close)
    ;; Do pancake pouring here
    (demo-part-pouring (current-desig pancake-mix) (current-desig *pancake-maker*))
    ;; Put the pancake mix back
    (place-object (current-desig pancake-mix) *loc-putdown-pancake-mix*)
    ;; Position in front of pancake table (far)
    (drive-to-pancake-pose-far)
    ;; Equip the spatulas
    (with-designators ((spatula-left (object (append (description (current-desig *spatula-left*))
                                                     `((side :left)))))
                       (spatula-right (object (append (description (current-desig *spatula-right*))
                                                      `((side :right))))))
      (equate *spatula-left* spatula-left)
      (pick-object spatula-left :stationary t)
      (equate *spatula-right* spatula-right)
      (pick-object spatula-right :stationary t)
      (perceive-a *pancake*)
      ;; Position in front of pancake table (close)
      (drive-to-pancake-pose-close)
      ;; Do the pancake flipping here
      (demo-part-flipping spatula-left spatula-right *pancake* *pancake-maker*)
      ;; Position in front of pancake table (far)
      (drive-to-pancake-pose-far)
      ;; Put the spatulas back
      (place-object spatula-left *loc-putdown-spatula-left* :stationary t)
      (place-object spatula-right *loc-putdown-spatula-right* :stationary t))
    ;; Well-defined home-pose
    (ensure-arms-up)))

(defun demo-part-pouring (pancake-mix pancake-maker)
  (set-initial-joint-values-pouring)
  (cram-pr2-fccl-demo::set-pose
   pancake-mix
   (cl-tf:make-pose-stamped
    "r_gripper_tool_frame" 0
    (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    (cl-transforms:make-identity-rotation)))
  (cram-pr2-fccl-demo::set-pose
   pancake-maker
   (reference (desig-prop-value
               (desig:current-desig *pancake-maker*)
               'desig-props:at)))
  (pr2-fccl-demo::demo-part-pouring pancake-mix pancake-maker))

(defun demo-part-flipping (spatula-left spatula-right pancake pancake-maker)
  (set-initial-joint-values-flipping)
  (cram-pr2-fccl-demo::set-pose
   spatula-left
   (cl-tf:make-pose-stamped
    "l_gripper_tool_frame" 0
    (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    (cl-transforms:make-identity-rotation)))
  (cram-pr2-fccl-demo::set-pose
   spatula-right
   (cl-tf:make-pose-stamped
    "r_gripper_tool_frame" 0
    (cl-transforms:make-3d-vector 0.0 0.0 0.0)
    (cl-transforms:make-identity-rotation)))
  (cram-pr2-fccl-demo::set-pose
   pancake
   (reference (desig-prop-value
               (desig:current-desig pancake-maker)
               'desig-props:at)))
  (cram-pr2-fccl-demo::set-pose
   pancake-maker
   (reference (desig-prop-value
               (desig:current-desig pancake-maker)
               'desig-props:at)))
  (pr2-fccl-demo::demo-part-flipping spatula-left spatula-right pancake pancake-maker))
