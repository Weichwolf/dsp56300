; SPDX-License-Identifier: GPL-3.0-only
(set-logic QF_BV)
(declare-const address (_ BitVec 32))
(declare-const begin (_ BitVec 32))
(declare-const words (_ BitVec 32))
(assert (bvule address #x00ffffff))
(assert (bvult begin #x00ffff80))
(assert (bvugt words #x00000000))
(assert (bvule words (bvsub #x00ffff80 begin)))
(define-fun interval () Bool
  (and (bvuge address begin) (bvult address (bvadd begin words))))
(push)
; The emitted SUB32/CMP32/JAE guard must equal the independent half-open interval.
(assert (distinct (bvult (bvsub address begin) words) interval))
(check-sat)
(pop)
; An incorrect signed comparison must produce a concrete counterexample.
(push)
(assert (distinct (bvslt (bvsub address begin) words) interval))
(check-sat)
(get-value (address begin words))
(pop)
