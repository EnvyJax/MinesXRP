import importlib
import time

try:
    defaults = importlib.import_module("XRPLib.defaults")
    reflectance = defaults.reflectance
    drivetrain = defaults.drivetrain
    distance = defaults.distance
except Exception as exc:
    raise RuntimeError("XRPLib.defaults could not be loaded. Run this on the XRP robot.") from exc

BASE_SPEED = 0.95
TURN_SPEED = 0.95
FIRST_TURN_DISTANCE = 42.3
SECOND_TURN_DISTANCE = 51.4
REFLECT_TARGET = (0.354, 0.391)
REFLECT_TOL = 0.12
STOP_DISTANCE = 12.0
TURN_TIME = 0.55


def clamp_speed(speed):
    return max(-1.0, min(1.0, speed))


def get_reflectance():
    try:
        v = reflectance.read()
    except Exception:
        try:
            v = reflectance.get()
        except Exception:
            return None
    if not v:
        return None
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    return None


def close_enough(a, b, tol=REFLECT_TOL):
    return abs(a - b) <= tol * max(abs(a), abs(b), 1.0)


def detect_reflect_pair(target_left, target_right):
    r = get_reflectance()
    if not r:
        return False
    return close_enough(r[0], target_left) and close_enough(r[1], target_right)


def set_drive(left_speed, right_speed):
    drivetrain.set_effort(clamp_speed(left_speed), clamp_speed(right_speed))


def stop_drive():
    drivetrain.stop()


def turn_left():
    set_drive(-TURN_SPEED, TURN_SPEED)
    time.sleep(TURN_TIME)
    stop_drive()
    time.sleep(0.1)


def turn_right():
    set_drive(TURN_SPEED, -TURN_SPEED)
    time.sleep(TURN_TIME)
    stop_drive()
    time.sleep(0.1)


def main():
    phase = 0

    while True:
        try:
            d = None
            try:
                d = float(distance.get_distance())
            except Exception:
                pass

            if d is not None and d <= STOP_DISTANCE:
                stop_drive()
                break

            if phase == 0 and d is not None and d <= FIRST_TURN_DISTANCE:
                stop_drive()
                turn_left()
                phase = 1
                set_drive(BASE_SPEED, BASE_SPEED)
            elif phase == 1 and d is not None and d <= SECOND_TURN_DISTANCE:
                stop_drive()
                turn_right()
                phase = 2
                set_drive(BASE_SPEED, BASE_SPEED)
            elif phase == 2 and detect_reflect_pair(*REFLECT_TARGET):
                stop_drive()
                turn_right()
                phase = 3
                set_drive(BASE_SPEED, BASE_SPEED)
            elif phase == 3 and d is not None and d <= 50.6:
                stop_drive()
                turn_left()
                phase = 4
                set_drive(BASE_SPEED, BASE_SPEED)
            else:
                set_drive(BASE_SPEED, BASE_SPEED)

            time.sleep(0.02)
        except KeyboardInterrupt:
            stop_drive()
            break

    stop_drive()


if __name__ == "__main__":
    main()
