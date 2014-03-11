; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-robohow-review-year2
  :name "cram-robohow-review-year2"
  :author "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :version "0.1"
  :maintainer "Georg Bartels <georg.bartels@cs.uni-bremen.de>"
  :licence "BSD"
  :description "CRAM code for RoboHow Year 2 review"
  :depends-on (:cram-language
               :designators
               :cram-language-designator-support
               :cram-reasoning
               :roslisp-utilities
               :roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "demo-plans" :depends-on ("package"))))))
