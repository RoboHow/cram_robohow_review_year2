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

(def-top-level-cram-function demo (&key (pick-and-place t) (pancake-manipulation t))
  (prepare-settings)
  (with-process-modules
    (prepare-global-designators)
    (when pick-and-place
      (pick-and-place))
    (when pancake-manipulation
      (pancake-manipulation))))

(def-cram-function pick-and-place ()
  (let ((transport-details (list (list (cons *spatula-left* *loc-putdown-spatula-left*))
                                 (list (cons *spatula-right* *loc-putdown-spatula-right*)
                                       (cons *pancake-mix* *loc-putdown-pancake-mix*)))))
    (dolist (tr-set transport-details)
      (ensure-arms-up)
      (dolist (transport-detail tr-set)
        (destructuring-bind (object . destination) transport-detail
          (declare (ignore destination))
          (pick-object object)))
      (dolist (transport-detail tr-set)
        (unless (desig:newest-effective-designator *pancake-maker*)
          (perceive-a *pancake-maker*))
        (destructuring-bind (object . destination) transport-detail
          (place-object object destination))))))

(def-cram-function pancake-manipulation ()
  ;; Well-defined start pose
  (ensure-arms-up)
  ;; Position in front of pancake table
  (drive-to-pancake-pose)
  ;; Grasp the pancake mix
  (pick-object *pancake-mix* :stationary t)
  ;; Do pancake pouring here
  (demo-part-pouring *pancake-mix*)
  ;; Put the pancake mix back
  (place-object *pancake-mix* *loc-putdown-pancake-mix*)
  ;; Equip the spatulas
  (with-designators ((spatula-left (object (append (description *spatula-left*)
                                                   `((side :left)))))
                     (spatula-right (object (append (description *spatula-right*)
                                                    `((side :right))))))
    (equate *spatula-left* spatula-left)
    (pick-object spatula-left :stationary t)
    (equate *spatula-right* spatula-right)
    (pick-object spatula-right :stationary t)
    (perceive-a *pancake*)
    ;; Do the pancake flipping here
    (demo-part-flipping spatula-left spatula-right *pancake* *pancake-maker*)
    ;; Put the spatulas back
    (place-object spatula-left *loc-putdown-spatula-left* :stationary t)
    (place-object spatula-right *loc-putdown-spatula-right* :stationary t)))

(def-cram-function demo-part-pouring (pancake-mix)
  ;; Implement the pouring part.
  )

(def-cram-function demo-part-flipping (spatula-left spatula-right pancake pancake-maker)
  ;; Implement the flipping part.
  )
