# A starter robot for rules v2, in CLIPS, on the Clunkyard Python binding
# (the clunkyard package, which the arena provides) and its installed CLIPS adapter. It sweeps a
# medium scan to find a robot, or points it where a passive contact was
# heard, narrows the scan to hold it, leads it with shots whose power suits
# the range, fires only when the turret is on target, keeps battery back for
# driving and keeps out of the storm. The rules in tactic.clp decide; this
# file starts them.
#
# The trade-offs (the game, gun, battery and scanner facts have the numbers
# from hello):
#   scan     a narrow arc reaches further; a wide one finds robots nearby
#   power    heavy shots hit harder for less battery, but fly slowly
#   battery  shots and driving share it, and it refills slowly
#   turret   it turns at a limited rate; me's aim-heading is where it points now
#   cues     passive contacts (a ping, a scan beam, a shot heard) say which
#            way to look, not where to shoot: they have no id and a noisy
#            bearing, so the target is the precise contact of rank 0
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range)
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
import math
from pathlib import Path

import clunkyard as cy
import clunkyard_clips


class Starter(clunkyard_clips.ClipsRobot):
    def on_turn(self):
        super().on_turn()  # the rules set the controls


if __name__ == "__main__":
    cy.run(Starter(str(Path(__file__).with_name("tactic.clp"))), binding_name="clunkyard-clips")
