import time

from XRPLib.defaults import reflectance, rangefinder, drivetrain

# ---------------------------------------------------------------------------
# Course table — restored from the earlier measured values in the old file
# versions. Each leg waits for its trigger, performs its action, then the
# robot drives straight again.
#
# trigger:
#   ("distance", cm)           rangefinder.distance() <= cm
#   ("reflect_match", (l, r))  both reflectance channels are within
#                              REFLECT_TOL of the given (left, right) pair
#                              (the sensor is sitting over that specific pad)
#
# action: "turn_left", "turn_right", "turn_around", "stop", "none"
#   "none" just advances to the next leg without turning — used for the
#   green pad, which the old code only needed to detect, then acted on at
#   the next distance checkpoint (GREEN_MOUNTAIN_DISTANCE below).
# ---------------------------------------------------------------------------
BLUE_TARGET = (0.615, 0.552)          # measured reflectance sitting on the blue pad
GREEN_TARGET = (0.354, 0.391)         # measured reflectance sitting on the green pad
RAMP_DISTANCE = 50.0                  # distance to the ramp obstacle -> turn right
GREEN_MOUNTAIN_DISTANCE = 51.4        # distance to the green mountain, after green was seen -> turn left
STOP_DISTANCE = 12.0                  # distance to the wood pad / finish -> stop

COURSE = [
    {"trigger": ("reflect_match", BLUE_TARGET), "action": "turn_left"},
    {"trigger": ("reflect_match", GREEN_TARGET), "action": "none"},
    {"trigger": ("distance", RAMP_DISTANCE), "action": "turn_right"},
    {"trigger": ("distance", GREEN_MOUNTAIN_DISTANCE), "action": "turn_left"},
    {"trigger": ("distance", STOP_DISTANCE), "action": "stop"},
]

BASE_SPEED = 0.6
DRIFT_ADJUST = 0.0          # bump if the robot pulls to one side while driving straight
TURN_ANGLE = 90.0           # degrees; positive turns clockwise/right per XRPLib convention
TURN_SPEED = 0.5
POLL_INTERVAL = 0.02
REFLECT_TOL = 0.12
DEBUG = True
DEBUG_EVERY = 25            # ~0.5s of loop iterations at POLL_INTERVAL

_HAS_GYRO_TURN = hasattr(drivetrain, "turn")


def get_distance():
    try:
        return float(rangefinder.distance())
    except Exception:
        return None


def close_enough(a, b, tol=REFLECT_TOL):
    return abs(a - b) <= tol * max(abs(a), abs(b), 1.0)


def reflect_matches(target):
    l = reflectance.get_left()
    r = reflectance.get_right()
    return close_enough(l, target[0]) and close_enough(r, target[1])


def trigger_fired(trigger):
    kind, value = trigger
    if kind == "distance":
        d = get_distance()
        return d is not None and d <= value
    if kind == "reflect_match":
        return reflect_matches(value)
    raise ValueError("unknown trigger kind: %r" % (kind,))


def drive_straight():
    drivetrain.set_effort(BASE_SPEED - DRIFT_ADJUST, BASE_SPEED)


def stop():
    drivetrain.stop()


def turn(angle):
    stop()
    time.sleep(0.1)
    if _HAS_GYRO_TURN:
        drivetrain.turn(angle, TURN_SPEED)
    else:
        # fallback: timed open-loop turn, will drift with battery voltage
        left = -TURN_SPEED if angle > 0 else TURN_SPEED
        right = TURN_SPEED if angle > 0 else -TURN_SPEED
        drivetrain.set_effort(left, right)
        time.sleep(abs(angle) / 90.0 * 0.55)
        stop()
    time.sleep(0.1)


def do_action(action):
    if action == "turn_left":
        turn(-TURN_ANGLE)
    elif action == "turn_right":
        turn(TURN_ANGLE)
    elif action == "turn_around":
        turn(180.0)
    elif action == "stop":
        stop()
        return True
    elif action == "none":
        pass
    else:
        raise ValueError("unknown action: %r" % (action,))
    return False


def main():
    drive_straight()
    leg_index = 0
    counter = 0

    try:
        while leg_index < len(COURSE):
            # Global safety net, independent of which leg we're on: never ram
            # whatever is directly ahead, even if a leg trigger misfires.
            d = get_distance()
            if d is not None and d <= STOP_DISTANCE:
                stop()
                break

            if DEBUG and counter % DEBUG_EVERY == 0:
                l = reflectance.get_left()
                r = reflectance.get_right()
                print("d=%s refl=(%.3f, %.3f) leg=%d" % (d, l, r, leg_index))
            counter += 1

            leg = COURSE[leg_index]
            if trigger_fired(leg["trigger"]):
                if DEBUG:
                    print("leg %d fired: %r -> %s" % (leg_index, leg["trigger"], leg["action"]))
                finished = do_action(leg["action"])
                leg_index += 1
                if finished:
                    break
                drive_straight()

            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        pass
    finally:
        stop()


if __name__ == "__main__":
    main()
