import time
import math
from dronekit import connect, VehicleMode

# ---------------- CONFIG ----------------
POINT_B = (50.443326, 30.448078)
TARGET_ALT = 200.0

RC_ROLL     = 1
RC_PITCH    = 2
RC_THROTTLE = 3
RC_YAW      = 4

RC_MID = 1500

# PID
Kp_pos = 2.0
Kd_pos = 0.3 

Kp_alt = 1.2

MAX_ANGLE = 300 

DT = 0.1

# ----------------------------------------

def get_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def latlon_to_ne(lat, lon, target_lat, target_lon):
    d_lat = target_lat - lat
    d_lon = target_lon - lon

    north = d_lat * 111320
    east  = d_lon * 111320 * math.cos(math.radians(lat))

    return north, east


def hold_alt(vehicle):
    alt = vehicle.location.global_relative_frame.alt
    error = TARGET_ALT - alt
    throttle = RC_MID + error * Kp_alt
    return int(max(1400, min(1600, throttle)))


def main():
    vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)

    while vehicle.gps_0.fix_type < 3:
        time.sleep(1)

    fixed_yaw = vehicle.heading
    print("Yaw fixed:", fixed_yaw)

    vehicle.mode = VehicleMode("STABILIZE")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    # -------- TAKEOFF --------
    print("Takeoff")

    while True:
        alt = vehicle.location.global_relative_frame.alt
        vehicle.channels.overrides[RC_THROTTLE] = hold_alt(vehicle)

        if alt > TARGET_ALT * 0.95:
            break

        time.sleep(0.2)

    print("Flight")

    prev_forward = 0
    prev_right = 0

    # -------- FLIGHT --------
    while True:
        loc = vehicle.location.global_relative_frame
        lat, lon, alt = loc.lat, loc.lon, loc.alt

        dist = get_distance(lat, lon, POINT_B[0], POINT_B[1])
        north, east = latlon_to_ne(lat, lon, POINT_B[0], POINT_B[1])

        yaw_rad = math.radians(fixed_yaw)

        forward =  north * math.cos(yaw_rad) + east * math.sin(yaw_rad)
        right   = -north * math.sin(yaw_rad) + east * math.cos(yaw_rad)

        # ---- PID ----
        d_forward = (forward - prev_forward) / DT
        d_right   = (right   - prev_right) / DT

        prev_forward = forward
        prev_right   = right

        control_f = Kp_pos * forward + Kd_pos * d_forward
        control_r = Kp_pos * right   + Kd_pos * d_right

        max_val = max(1.0, abs(control_f), abs(control_r))
        control_f /= max_val
        control_r /= max_val

        if dist > 10:
            k = 1.0
        else:
            k = dist / 10.0

        pitch_val = int(RC_MID - control_f * MAX_ANGLE * k)
        roll_val  = int(RC_MID + control_r * MAX_ANGLE * k)

        pitch_val = max(1300, min(1700, pitch_val))
        roll_val  = max(1300, min(1700, roll_val))

        print(f"Dist: {dist:.1f} | Alt: {alt:.1f}")

        if dist < 3:
            print("Arrived")
            break

        vehicle.channels.overrides[RC_PITCH]    = pitch_val
        vehicle.channels.overrides[RC_ROLL]     = roll_val
        vehicle.channels.overrides[RC_THROTTLE] = hold_alt(vehicle)
        vehicle.channels.overrides[RC_YAW]      = RC_MID

        time.sleep(DT)

    # -------- STOP --------
    vehicle.channels.overrides[RC_PITCH] = RC_MID
    vehicle.channels.overrides[RC_ROLL]  = RC_MID
    time.sleep(2)

    print("Landing")

    # -------- LANDING --------
    while True:
        loc = vehicle.location.global_relative_frame
        lat, lon, alt = loc.lat, loc.lon, loc.alt

        dist = get_distance(lat, lon, POINT_B[0], POINT_B[1])
        north, east = latlon_to_ne(lat, lon, POINT_B[0], POINT_B[1])

        yaw_rad = math.radians(fixed_yaw)

        forward =  north * math.cos(yaw_rad) + east * math.sin(yaw_rad)
        right   = -north * math.sin(yaw_rad) + east * math.cos(yaw_rad)

        pitch_val = int(RC_MID - forward * 15)
        roll_val  = int(RC_MID + right * 15)

        pitch_val = max(1420, min(1580, pitch_val))
        roll_val  = max(1420, min(1580, roll_val))

        # плавне зниження
        if alt > 50:
            thr = 1420
        elif alt > 20:
            thr = 1400
        elif alt > 10:
            thr = 1380
        elif alt > 5:
            thr = 1360
        else:
            thr = 1340
        print(f"Landing | Alt: {alt:.1f} | Dist: {dist:.2f}")

        vehicle.channels.overrides[RC_ROLL]     = roll_val
        vehicle.channels.overrides[RC_PITCH]    = pitch_val
        vehicle.channels.overrides[RC_THROTTLE] = thr
        vehicle.channels.overrides[RC_YAW]      = RC_MID

        if alt < 0.3:
            print("Landed")
            break

        time.sleep(0.2)

    vehicle.channels.overrides = {}
    vehicle.armed = False
    vehicle.close()

    print("DONE")


if __name__ == "__main__":
    main()