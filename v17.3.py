
from XRPLib.defaults import *
import time


COUNTS_PER_CM = 40.0

KP_STRAIGHT = 0.03          
STRAIGHT_TOLERANCE_CM = 0.5
TURN_TOLERANCE_DEG = 2.0
MIN_TURN_EFFORT = 0.3       

def drive_straight(distance_cm, effort):
    drivetrain.reset_encoder_position()
    imu.reset_yaw()
    direction = 1 if distance_cm >= 0 else -1
    while True:
        left = drivetrain.get_left_encoder_position()
        right = drivetrain.get_right_encoder_position()
        traveled_cm = ((left + right) / 2) / COUNTS_PER_CM
        remaining = abs(distance_cm) - traveled_cm
        if remaining <= STRAIGHT_TOLERANCE_CM:
            break
        heading_error = imu.get_yaw()  
        correction = KP_STRAIGHT * heading_error
        drivetrain.set_effort(direction * effort - correction, direction * effort + correction)
        time.sleep(0.005)
    

def turn_to(delta_deg, effort):
    imu.reset_yaw()
    direction = 1 if delta_deg >= 0 else -1
    target = abs(delta_deg)
    while True:
        turned = abs(imu.get_yaw())
        remaining = target - turned
        if remaining <= TURN_TOLERANCE_DEG:
            break
        scaled_effort = max(MIN_TURN_EFFORT, min(effort, effort * remaining / 30))
        drivetrain.set_effort(direction * scaled_effort, -direction * scaled_effort)
        time.sleep(0.005)

path = [
    (128, 1.0,  90, 0.9),
    ( 61, 1.0, -90, 0.9),
    ( 88, 1.0,  90, 0.9),
    (140, 1.0, 180, 0.9),
    (170, 1.0,  90, 0.9),
    ( 90, 1.0, None, None),
]

for straight_cm, straight_effort, turn_deg, turn_effort in path:
    drive_straight(straight_cm, straight_effort)
    if turn_deg is not None:
        turn_to(turn_deg, turn_effort)

drivetrain.stop()
board.led_on()
