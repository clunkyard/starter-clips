; The tactic: CLIPS rules from the tick's facts to its controls. Each turn
; the SDK asserts the facts (me, contact, cue, zone, storm …; clunkyard.clp
; lists them all) and sends what these rules assert. For the same control,
; a later fact overrides an earlier one, so the low-salience rules at the
; end adjust or override what the first ones chose.

(defglobal ?*search-turn* = 0.33)  ; enough to turn against the brake in the force rulesets

(defrule brake
  =>
  (assert (forces (brake 0.2))))

(defrule track
  (me (heading ?heading) (pitch ?pitch) (aim-heading ?aim) (aim-pitch ?aim-pitch))
  (contact (precise TRUE) (rank 0) (range ?range) (bearing ?bearing) (elevation ?elevation)
           (x ?tx) (y ?ty) (z ?tz) (vx ?vx) (vy ?vy) (vz ?vz))
  =>
  (assert (drive (throttle (if (> ?range 350) then 0.7 else (if (< ?range 200) then -0.4 else 0.2)))
                 (turn (clamp (/ (angle-diff ?bearing ?heading) 45)))
                 (pitch-turn (clamp (/ (- ?elevation ?pitch) 45)))))
  (assert (scan (heading ?bearing) (arc 20) (pitch ?elevation)))  ; narrow: holds it, reaching about 735
  ; The heaviest shot that still covers the range in about a second,
  ; aimed where the target will be, lobbed under gravity.
  (bind ?power (power-for-flight-time ?range))
  (bind ?lead (lead ?tx ?ty ?tz ?vx ?vy ?vz ?power))
  (bind ?to-heading (if (multifieldp ?lead) then (nth$ 1 ?lead) else ?bearing))
  (bind ?to-pitch (if (multifieldp ?lead) then (nth$ 2 ?lead) else ?elevation))
  (assert (aim (heading ?to-heading) (pitch ?to-pitch)))
  (if (and (< (abs (angle-diff ?to-heading ?aim)) 3) (< (abs (- ?to-pitch ?aim-pitch)) 3)
           (eq (can-fire ?power 10) TRUE))
   then (assert (fire (power ?power))))
  (assert (status (text "TRACK") (value ?range))))

(defrule follow-cue
  (not (contact (precise TRUE)))
  (cue (x ?cx) (y ?cy) (z ?cz) (bearing-error ?bearing-error))
  (game (is-3d ?is-3d) (gravity-z ?gravity))
  (me (x ?x) (y ?y) (z ?z) (heading ?heading) (pitch ?pitch))
  (state (tick ?tick))
  =>
  ; Head for where it was heard, sweeping an arc that still reaches it
  ; across the bearing's error (and, flying free, the elevation's).
  (bind ?free (and (eq ?is-3d TRUE) (= ?gravity 0)))
  (bind ?bearing (bearing-to ?x ?y ?cx ?cy))
  (bind ?distance (distance ?x ?y ?z ?cx ?cy ?cz))
  (bind ?elevation (if ?free then (elevation-to ?x ?y ?z ?cx ?cy ?cz) else 0))
  (bind ?error (min ?bearing-error 30))
  (bind ?across (* (/ (- (mod ?tick 5) 2) 2) ?error))
  (bind ?up (if ?free then (* (- (mod (div ?tick 5) 3) 1) (/ ?error 2)) else 0))
  (assert (drive (throttle 0.7) (turn (clamp (/ (angle-diff ?bearing ?heading) 45)))
                 (pitch-turn (clamp (/ (- ?elevation ?pitch) 45)))))
  (assert (scan (heading (+ ?bearing ?across)) (arc (arc-for-reach (+ ?distance 100))) (pitch (+ ?elevation ?up))))
  (assert (status (text "CUE") (value ?distance))))

(defrule search
  (not (contact (precise TRUE)))
  (not (cue))
  (game (is-3d ?is-3d) (gravity-z ?gravity))
  (me (pitch ?pitch))
  (state (tick ?tick) (scan-heading ?scan))
  =>
  (assert (drive (throttle 0.5) (turn ?*search-turn*) (pitch-turn (clamp (/ (- 0 ?pitch) 45)))))
  ; Sweeps the circle in half a second, flying free at three heights in turn.
  (assert (scan (heading (mod (+ ?scan 36) 360)) (arc 40)
                (pitch (if (and (eq ?is-3d TRUE) (= ?gravity 0)) then (* (- (mod (div ?tick 10) 3) 1) 35) else 0))))
  (assert (status (text "SEARCH"))))

; Low battery: drive gently so the shots can still be paid for.
(defrule low-battery
  (declare (salience -10))
  (battery (capacity ?capacity))
  (me (battery ?charge&:(< ?charge (/ ?capacity 4))))
  (drive (throttle ?throttle&~nil))
  (not (gentle))
  =>
  (assert (gentle))
  (assert (drive (throttle (* ?throttle 0.5)))))

; The storm hurts inside: drive straight away from its centre.
(defrule leave-storm
  (declare (salience -20))
  (storm (x ?sx) (y ?sy) (margin ?margin&:(>= ?margin -60)))
  (me (x ?x) (y ?y) (heading ?heading))
  =>
  (assert (drive (throttle 0.8) (turn (clamp (/ (angle-diff (bearing-to ?sx ?sy ?x ?y) ?heading) 30))))))

; Sudden death: head back inside the safe zone.
(defrule return-to-zone
  (declare (salience -30))
  (zone (shape ?shape) (x ?zx) (y ?zy) (z ?zz) (margin ?margin&:(< ?margin 60)))
  (me (x ?x) (y ?y) (z ?z) (heading ?heading) (pitch ?pitch))
  =>
  (bind ?climb (if (eq ?shape sphere) then (elevation-to ?x ?y ?z ?zx ?zy ?zz) else 0))
  (assert (drive (throttle 0.8) (turn (clamp (/ (angle-diff (bearing-to ?x ?y ?zx ?zy) ?heading) 30)))
                 (pitch-turn (clamp (/ (- ?climb ?pitch) 30))))))
