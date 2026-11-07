
"""RoadWatch - drone controller, lesson 4: hold 25 m and follow the road.

What's new vs lesson 3:
  1. altitude: cascaded loop (position -> target climb speed -> thrust) = no more overshoot
  2. the road centerline, computed with the SAME B-spline math Webots uses to draw the Road
  3. pure pursuit: steer (yaw) toward a point ~15 m ahead on the road
  4. speed control: tilt forward (pitch) to cruise at a target speed, brake at the end
  5. camera gimbal points straight down

WORLD CHANGE NEEDED: in highway.wbt set  WorldInfo { gpsCoordinateSystem "local" }
so the GPS returns x, y, z in meters (we convert to lat/lon later, at HQ).
"""

import math
import csv
import os
import time

from controller import Robot


# ===============================================================
# helpers
# ===============================================================
def clamp(value, low, high):
    """Squeeze value into [low, high]."""
    return max(low, min(value, high))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi] so 'turn 350 deg left' becomes 'turn 10 deg right'."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def bspline3(points, subdivision):
    """Python port of Webots' wbgeometry.bSpline3 - the exact curve the Road PROTO draws.

    Note: a B-spline passes NEAR the waypoints, not through them
    (e.g. the road peaks at y=38.3, not 40).
    """
    p = [list(q) for q in points]
    n = len(p)
    # extend both ends the same way Webots does
    p.append([p[n - 1][k] + (p[n - 1][k] - p[n - 2][k]) for k in range(2)])
    p.append([p[n][k] + (p[n][k] - p[n - 1][k]) for k in range(2)])
    p.insert(0, [p[0][k] + (p[0][k] - p[1][k]) for k in range(2)])
    p.insert(0, [p[0][k] + (p[0][k] - p[1][k]) for k in range(2)])

    curve = [tuple(p[2])]
    for i in range(2, n + 1):
        coeffs = []
        for k in range(2):  # k=0 -> x, k=1 -> y
            a0 = (-p[i - 1][k] + 3 * p[i][k] - 3 * p[i + 1][k] + p[i + 2][k]) / 6
            a1 = (3 * p[i - 1][k] - 6 * p[i][k] + 3 * p[i + 1][k]) / 6
            a2 = (-3 * p[i - 1][k] + 3 * p[i + 1][k]) / 6
            a3 = (p[i - 1][k] + 4 * p[i][k] + p[i + 1][k]) / 6
            coeffs.append((a0, a1, a2, a3))
        for j in range(1, subdivision + 1):
            t = j / subdivision
            curve.append(tuple((a2 + t * (a1 + t * a0)) * t + a3 for (a0, a1, a2, a3) in coeffs))
    return curve


def densify(path, spacing):
    """Insert points so consecutive points are ~spacing meters apart."""
    dense = [path[0]]
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        steps = max(1, int(math.hypot(x1 - x0, y1 - y0) / spacing))
        for s in range(1, steps + 1):
            t = s / steps
            dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return dense




# ===============================================================
# constants
# ===============================================================
# --- road (must match the Road node in highway.wbt) ---
ROAD_WAYPOINTS = [(0, 0), (300, 0), (600, 40), (900, 40), (1200, 0), (1500, 0)]
ROAD_SPLINE_SUBDIVISION = 8

# --- mission ---
TARGET_ALTITUDE = 25.0      # m
CRUISE_SPEED = 5.0          # m/s along the road
LOOKAHEAD = 15.0            # m - how far ahead on the road we aim
GOAL_TOLERANCE = 5.0        # m - "arrived" radius at the end of the road

# --- base thrust + attitude (official Mavic values) ---
VERTICAL_THRUST = 68.5
K_ROLL_P = 50.0
K_PITCH_P = 30.0

# --- altitude: cascaded loop (tuned against a model fitted to your lesson-3 log) ---
K_ALT_P = 0.5               # 1 m too low -> climb at 0.5 m/s
MAX_CLIMB_SPEED = 2.0       # m/s, up or down
K_VZ_P = 4.0                # thrust per (m/s) of climb-speed error
HOVER_TRIM = 0.6            # extra thrust needed to truly hover (68.5 alone sinks slowly)
MAX_VERTICAL_INPUT = 3.0

# --- heading + speed ---
K_YAW_P = 1.0               # yaw command per radian of heading error
MAX_YAW_INPUT = 1.3         # same as the official keyboard turn
K_SPEED_P = 0.5             # pitch command per (m/s) of speed error
MAX_FORWARD_CMD = 2.0       # same as the official keyboard "forward"
MAX_BRAKE_CMD = 1.0
ALIGN_BEFORE_MOVING = 0.6   # rad (~35 deg) - don't accelerate if we're facing the wrong way

CAMERA_DOWN = 1.57          # rad - gimbal pitch for a straight-down camera (max is 1.7)

CAMERA_PERIOD_MS = 64
FRAME_SPACING = 10.0               # m - distance between consecutive camera frames (for HQ to stitch)
JPEG_QUALITY = 90                  # 0-100, higher = better quality, bigger files
# ===============================================================
# setup
# ===============================================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())
steps_per_second = 1000 // timestep


project_root = os.path.join(os.path.dirname(os.path.dirname(__file__)), "..")
run_dir = os.path.join(project_root, "data", "sim_frames", time.strftime("run_%Y-%m-%d_%H-%M-%S"))
os.makedirs(run_dir, exist_ok=True)

gps = robot.getDevice("gps")
imu = robot.getDevice("inertial unit")
gyro = robot.getDevice("gyro")
camera = robot.getDevice("camera")
for sensor in (gps, imu, gyro):
    sensor.enable(timestep)
camera.enable(CAMERA_PERIOD_MS)

camera_pitch_motor = robot.getDevice("camera pitch")
camera_pitch_motor.setPosition(CAMERA_DOWN)

front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")
motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]
for motor in motors:
    motor.setPosition(float("inf"))
    motor.setVelocity(1.0)

road = densify(bspline3(ROAD_WAYPOINTS, ROAD_SPLINE_SUBDIVISION), spacing=2.0)
goal_x, goal_y = road[-1]



log_file = open(os.path.join(run_dir, "log.csv"), "w", newline="")
pose_log = csv.writer(log_file)
pose_log.writerow(["frame", "time", "x", "y", "alt", "roll", "pitch", "yaw"])
print(f">>> saving frames to {run_dir}")
frame_id = 0
last_frame_xy = None   # (x, y) of the last camera frame we saved, or None if none yet


path_index = None   # index of the closest road point (found on the first navigation step)
step_count = 0
state = "TAKEOFF"   # TAKEOFF -> FOLLOW_ROAD -> ARRIVED




# ===============================================================
# main loop
# ===============================================================
while robot.step(timestep) != -1:
    # ---------- read ----------
    x, y, alt = gps.getValues()                # local mode -> meters
    vx, vy, vz = gps.getSpeedVector()          # m/s in world frame
    roll, pitch, yaw = imu.getRollPitchYaw()
    roll_rate, pitch_rate, yaw_rate = gyro.getValues()

    # ---------- altitude: position -> target climb speed -> thrust ----------
    target_vz = clamp(K_ALT_P * (TARGET_ALTITUDE - alt), -MAX_CLIMB_SPEED, MAX_CLIMB_SPEED)
    vertical_input = clamp(K_VZ_P * (target_vz - vz) + HOVER_TRIM,
                           -MAX_VERTICAL_INPUT, MAX_VERTICAL_INPUT)

    # ---------- navigation ----------
    yaw_input = 0.0
    forward_cmd = 0.0
    heading_error = 0.0
    cross_track = 0.0
    forward_speed = vx * math.cos(yaw) + vy * math.sin(yaw)  # speed along the drone's nose

    if state == "TAKEOFF" and alt > TARGET_ALTITUDE - 1.0:
        state = "FOLLOW_ROAD"
        print(">>> reached altitude, following the road")

    if state == "FOLLOW_ROAD":
        # 1) where am i on the road? search near the last index (fast + never jumps backwards)
        if path_index is None:
            search = range(len(road))
        else:
            search = range(path_index, min(path_index + 50, len(road)))
        path_index = min(search, key=lambda i: (road[i][0] - x) ** 2 + (road[i][1] - y) ** 2)
        cross_track = math.hypot(road[path_index][0] - x, road[path_index][1] - y)

        # 2) pure pursuit: aim at a point LOOKAHEAD meters further along the road
        lookahead_index = min(path_index + int(LOOKAHEAD / 2.0), len(road) - 1)
        aim_x, aim_y = road[lookahead_index]
        heading_error = wrap_angle(math.atan2(aim_y - y, aim_x - x) - yaw)
        yaw_input = clamp(K_YAW_P * heading_error - 1.0 * yaw_rate, -MAX_YAW_INPUT, MAX_YAW_INPUT)

        # 3) speed: cruise, but slow down near the goal and when not facing the road
        distance_to_goal = math.hypot(goal_x - x, goal_y - y)
        target_speed = min(CRUISE_SPEED, 0.5 * distance_to_goal)
        forward_cmd = clamp(K_SPEED_P * (target_speed - forward_speed), -MAX_BRAKE_CMD, MAX_FORWARD_CMD)
        if abs(heading_error) > ALIGN_BEFORE_MOVING:
            forward_cmd = min(forward_cmd, 0.0)  # turn first, then go


        if last_frame_xy is None:
            moved = FRAME_SPACING
        else:
            moved = math.hypot(x - last_frame_xy[0], y - last_frame_xy[1])
        if moved >= FRAME_SPACING:
            file_name = f"frame_{frame_id:05d}.jpg"
            camera.saveImage(os.path.join(run_dir, file_name), JPEG_QUALITY)
            pose_log.writerow([
                file_name, f"{robot.getTime():.3f}", f"{x:.2f}", f"{y:.2f}", f"{alt:.4f}", f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}",
            ])
            log_file.flush()
            last_frame_xy = (x, y)
            frame_id += 1

        if distance_to_goal < GOAL_TOLERANCE:
            state = "ARRIVED"
            print(">>> end of road reached, hovering")


    if state == "ARRIVED":
        # brake to a stop and hover
        forward_cmd = clamp(K_SPEED_P * (0.0 - forward_speed), -MAX_BRAKE_CMD, MAX_FORWARD_CMD)

    pitch_disturbance = -forward_cmd   # negative pitch = nose down = forward (official convention)

    # ---------- attitude ----------
    roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_rate
    pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_rate + pitch_disturbance

    # ---------- mixing ----------
    base = VERTICAL_THRUST + vertical_input
    fl = base - roll_input + pitch_input - yaw_input
    fr = base + roll_input + pitch_input + yaw_input
    rl = base - roll_input - pitch_input + yaw_input
    rr = base + roll_input - pitch_input - yaw_input

    front_left_motor.setVelocity(fl)
    front_right_motor.setVelocity(-fr)
    rear_left_motor.setVelocity(-rl)
    rear_right_motor.setVelocity(rr)

    # ---------- log once per second ----------
    step_count += 1
    if step_count % steps_per_second == 0:
        print(
            f"t={robot.getTime():5.1f}s {state:11s} | alt {alt:5.1f} m vz {vz:+.2f} | "
            f"pos ({x:7.1f}, {y:6.1f}) | speed {forward_speed:4.1f} m/s | "
            f"heading err {math.degrees(heading_error):+5.1f} deg | off-road {cross_track:4.1f} m | "

            f"frames {frame_id}"
        )

log_file.close()