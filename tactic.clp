; The tactic: CLIPS rules from the tick's facts to its controls. Each turn
; the SDK asserts the facts (me, contact, cue, zone, storm …; clunkyard.clp
; lists them all) and sends what these rules assert. For the same control,
; a later fact overrides an earlier one, so the low-salience rules at the
; end adjust or override what the first ones chose.
;
; Bearing is earned: a scan contact's bearing is off by up to its
; bearing-error (degrees, a fixed share of the arc), so a wide beam finds a
; robot and a narrow one aims at it. The robot narrows the beam, holds it on
; the target and aims along a line fitted through its recent sightings.
; Contacts within 120 units are exact.

(defglobal ?*search-turn* = 0.33)  ; enough to turn against the brake in the force rulesets
(defglobal ?*search-arc* = 40)     ; degrees: sweeps the circle in half a second
(defglobal ?*hold-width* = 60)     ; units across at the target: it stays in the beam from tick to tick
(defglobal ?*min-arc* = 6)         ; degrees: the narrowest the beam holds
(defglobal ?*window* = 12)         ; ticks of sightings the aim averages
(defglobal ?*sightings* = 8)       ; narrow sightings in the window before a shot
(defglobal ?*look-every* = 40)     ; ticks between looks around the target, for a nearer robot
(defglobal ?*reacquire* = 10)      ; ticks to look again where a lost target was
; Dodges taken, and the tick the current one ends.
(defglobal ?*dodges* = 0 ?*dodge-end* = 0)
; The last target's slot, for looking again where it was lost.
(defglobal ?*slot* = 0)

; Field ?k (1 tick, 3 bearing error, 5 to 7 x y z) of sighting ?i (from 0) in a history.
(deffunction seen-field (?history ?i ?k) (nth$ (+ 1 (* 7 ?i) ?k) ?history))
; Whether sighting ?i is among the narrow ones of the last *window* ticks.
(deffunction narrow-p (?history ?i ?tick ?limit)
  (and (> (seen-field ?history ?i 1) (- ?tick ?*window*)) (<= (seen-field ?history ?i 3) ?limit)))

; A straight line through the narrowest of the last *window* ticks' sightings:
; (x y z vx vy vz count), where the robot is now and how fast it moves, or
; FALSE with fewer than two. Each sighting is off by its bearing error; the
; line averages that out.
(deffunction fit (?slot ?tick ?tick-rate)
  (bind ?history (cy-tracker-history ?slot))
  (if (not (multifieldp ?history)) then (return FALSE))
  (bind ?count (nth$ 1 ?history))
  (bind ?best 1e9)
  (loop-for-count (?i 0 (- ?count 1))
    (if (> (seen-field ?history ?i 1) (- ?tick ?*window*))
     then (bind ?best (min ?best (seen-field ?history ?i 3)))))
  (bind ?limit (* 1.5 ?best))  ; the narrow ones
  (bind ?n 0) (bind ?sum-t 0.0) (bind ?sum-x 0.0) (bind ?sum-y 0.0) (bind ?sum-z 0.0)
  (loop-for-count (?i 0 (- ?count 1))
    (if (narrow-p ?history ?i ?tick ?limit)
     then (bind ?n (+ ?n 1))
          (bind ?sum-t (+ ?sum-t (seen-field ?history ?i 1)))
          (bind ?sum-x (+ ?sum-x (seen-field ?history ?i 5)))
          (bind ?sum-y (+ ?sum-y (seen-field ?history ?i 6)))
          (bind ?sum-z (+ ?sum-z (seen-field ?history ?i 7)))))
  (if (< ?n 2) then (return FALSE))
  (bind ?t0 (/ ?sum-t ?n))
  (bind ?mean-x (/ ?sum-x ?n)) (bind ?mean-y (/ ?sum-y ?n)) (bind ?mean-z (/ ?sum-z ?n))
  (bind ?spread 0.0) (bind ?cross-x 0.0) (bind ?cross-y 0.0) (bind ?cross-z 0.0)
  (loop-for-count (?i 0 (- ?count 1))
    (if (narrow-p ?history ?i ?tick ?limit)
     then (bind ?dt (- (seen-field ?history ?i 1) ?t0))
          (bind ?spread (+ ?spread (* ?dt ?dt)))
          (bind ?cross-x (+ ?cross-x (* ?dt (- (seen-field ?history ?i 5) ?mean-x))))
          (bind ?cross-y (+ ?cross-y (* ?dt (- (seen-field ?history ?i 6) ?mean-y))))
          (bind ?cross-z (+ ?cross-z (* ?dt (- (seen-field ?history ?i 7) ?mean-z))))))
  (bind ?ahead (- ?tick ?t0))
  (create$ (+ ?mean-x (* (/ ?cross-x ?spread) ?ahead)) (+ ?mean-y (* (/ ?cross-y ?spread) ?ahead)) (+ ?mean-z (* (/ ?cross-z ?spread) ?ahead))
           (* (/ ?cross-x ?spread) ?tick-rate) (* (/ ?cross-y ?spread) ?tick-rate) (* (/ ?cross-z ?spread) ?tick-rate) ?n))

(defrule brake
  =>
  (assert (forces (brake 0.2))))

(defrule track
  (me (heading ?heading) (pitch ?pitch) (aim-heading ?aim) (aim-pitch ?aim-pitch))
  (contact (precise TRUE) (rank 0) (range ?range) (bearing ?bearing) (elevation ?elevation) (bearing-error ?error)
           (x ?tx) (y ?ty) (z ?tz) (vx ?vx) (vy ?vy) (vz ?vz))
  (game (tick-rate ?tick-rate))
  (state (tick ?tick) (scan-arc ?last-arc))
  =>
  (bind ?nearest (cy-sensors-nearest))
  (if (multifieldp ?nearest) then (bind ?*slot* (nth$ 1 ?nearest)))
  (assert (drive (throttle (if (> ?range 350) then 0.7 else (if (< ?range 200) then -0.4 else 0.2)))
                 (turn (clamp (/ (angle-diff ?bearing ?heading) 45)))
                 (pitch-turn (clamp (/ (- ?elevation ?pitch) 45)))))
  ; Halve the beam each tick until it is *hold-width* across at the target:
  ; narrow enough for a precise bearing, wide enough to keep it in.
  (bind ?hold (max ?*min-arc* (* 2 (rad-deg (atan (/ (/ ?*hold-width* 2) ?range))))))
  ; In a crowd a narrow beam sees one robot. Every *look-every* ticks,
  ; widen it for a tick: a nearer robot it finds becomes the target.
  (bind ?arc (if (= (mod ?tick ?*look-every*) 0) then ?*search-arc* else (max ?hold (min ?*search-arc* (/ ?last-arc 2)))))
  (assert (scan (heading ?bearing) (arc ?arc) (pitch ?elevation)))
  ; The heaviest shot that still covers the range in about a second,
  ; aimed where the target will be, lobbed under gravity.
  (bind ?power (power-for-flight-time ?range))
  ; An exact contact: lead is the intercept if the target holds course. A
  ; noisy one: lead the fitted line, and fire only once it rests on enough sightings.
  (bind ?ready TRUE)
  (if (> ?error 0)
   then (bind ?line (fit ?*slot* ?tick ?tick-rate))
        (bind ?ready (and (multifieldp ?line) (>= (nth$ 7 ?line) ?*sightings*)))
        (bind ?lead (if (multifieldp ?line) then (lead (nth$ 1 ?line) (nth$ 2 ?line) (nth$ 3 ?line) (nth$ 4 ?line) (nth$ 5 ?line) (nth$ 6 ?line) ?power) else FALSE))
   else (bind ?lead (lead ?tx ?ty ?tz ?vx ?vy ?vz ?power)))
  (bind ?to-heading (if (multifieldp ?lead) then (nth$ 1 ?lead) else ?bearing))
  (bind ?to-pitch (if (multifieldp ?lead) then (nth$ 2 ?lead) else ?elevation))
  (assert (aim (heading ?to-heading) (pitch ?to-pitch)))
  (if (and ?ready (< (abs (angle-diff ?to-heading ?aim)) 3) (< (abs (- ?to-pitch ?aim-pitch)) 3)
           (eq (can-fire ?power 10) TRUE))
   then (assert (fire (power ?power))))
  (assert (status (text (if ?ready then "TRACK" else "NARROW")) (value ?range))))

; Lost it: when no target is in the beam but one was seen in the last
; *reacquire* ticks, note it, so the cue and search rules wait.
(defrule note-lost
  (declare (salience 10))
  (not (contact (precise TRUE)))
  (state (tick ?tick))
  (test (> ?*slot* 0))
  =>
  (bind ?seen (cy-tracker-last-seen ?*slot*))
  (if (and (numberp ?seen) (<= (- ?tick ?seen) ?*reacquire*)) then (assert (lost))))

; Look again where it was last seen, widening the beam, before searching.
(defrule reacquire
  (lost)
  (game (is-3d ?is-3d) (gravity-z ?gravity))
  (me (x ?x) (y ?y) (z ?z) (heading ?heading) (pitch ?pitch))
  (state (scan-arc ?last-arc))
  =>
  (bind ?where (cy-tracker-position ?*slot*))
  (if (not (multifieldp ?where)) then (bind ?where (create$ ?x ?y ?z)))
  (bind ?bearing (bearing-to ?x ?y (nth$ 1 ?where) (nth$ 2 ?where)))
  (bind ?elevation (if (and (eq ?is-3d TRUE) (= ?gravity 0)) then (elevation-to ?x ?y ?z (nth$ 1 ?where) (nth$ 2 ?where) (nth$ 3 ?where)) else 0))
  (assert (drive (throttle 0.5) (turn (clamp (/ (angle-diff ?bearing ?heading) 45)))
                 (pitch-turn (clamp (/ (- ?elevation ?pitch) 45)))))
  (assert (scan (heading ?bearing) (arc (min ?*search-arc* (* ?last-arc 2))) (pitch ?elevation)))
  (assert (status (text "REACQUIRE"))))

(defrule follow-cue
  (not (contact (precise TRUE)))
  (not (lost))
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
  (not (lost))
  (not (cue))
  (game (is-3d ?is-3d) (gravity-z ?gravity))
  (me (pitch ?pitch))
  (state (tick ?tick) (scan-heading ?scan))
  =>
  (assert (drive (throttle 0.5) (turn ?*search-turn*) (pitch-turn (clamp (/ (- 0 ?pitch) 45)))))
  ; Sweeps the circle in half a second, flying free at three heights in turn.
  (assert (scan (heading (mod (+ ?scan 36) 360)) (arc ?*search-arc*)
                (pitch (if (and (eq ?is-3d TRUE) (= ?gravity 0)) then (* (- (mod (div ?tick 10) 3) 1) 35) else 0))))
  (assert (status (text "SEARCH"))))

; A simple dodge: vary it, or predict the shooter. A shot heard once the
; last dodge is over starts the next for 10 ticks.
(defrule hear-shot
  (declare (salience -5))
  (exists (contact (source muzzle)))
  (state (tick ?tick&:(>= ?tick ?*dodge-end*)))
  =>
  (bind ?*dodges* (+ ?*dodges* 1))
  (bind ?*dodge-end* (+ ?tick 10)))

; Off the line of fire while it lasts: left, then right next time (turn is
; positive left, strafe positive right). Aim, scan and fire are as chosen.
(defrule dodge
  (declare (salience -6))
  (state (tick ?tick))
  (game (control ?control))
  ?drive <- (drive)
  (not (dodged))
  =>
  (if (< ?tick ?*dodge-end*) then
    (assert (dodged))
    (bind ?left (= (mod ?*dodges* 2) 1))
    (if (eq ?control forces)
     then (modify ?drive (throttle 0.5)) (assert (forces (strafe (if ?left then -1 else 1))))
     else (modify ?drive (throttle 1) (turn (if ?left then 1 else -1))))
    (assert (status (text (if ?left then "dodge:left" else "dodge:right")) (value ?*dodges*)))))

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
