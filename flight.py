import time
import math
from dronekit import connect, VehicleMode

# --- Точки ---
POINT_A = (50.450739, 30.461242)
POINT_B = (50.443326, 30.448078)
TARGET_ALT = 200.0

# RC канали
RC_ROLL     = 1
RC_PITCH    = 2
RC_THROTTLE = 3
RC_YAW      = 4

RC_MID = 1500

# --- коефіцієнти ---
Kp_alt = 1.8


# ------------------ UTILS ------------------

def normalize_angle(angle):
    return (angle + 180) % 360 - 180


def connect_vehicle():
    print("Підключення до SITL...")
    vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
    print("Підключено!")
    return vehicle


def wait_for_gps(vehicle):
    print("Чекаємо GPS...")

    while True:
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon

        if lat != 0.0 and lon != 0.0 and vehicle.gps_0.fix_type >= 3:
            print(f"GPS OK: {lat}, {lon}")
            break

        time.sleep(1)


def get_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def get_bearing(lat1, lon1, lat2, lon2):
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)

    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def hold_altitude(vehicle, target_alt):
    alt = vehicle.location.global_relative_frame.alt
    error = target_alt - alt

    throttle = RC_MID + error * Kp_alt
    throttle = max(1450, min(1650, throttle))

    return int(throttle)


# ------------------ TAKEOFF ------------------

def arm_and_takeoff(vehicle, target_alt):
    print("Армінг...")

    vehicle.mode = VehicleMode("STABILIZE")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    print("Зліт...")

    while True:
        alt = vehicle.location.global_relative_frame.alt
        throttle = hold_altitude(vehicle, target_alt)

        vehicle.channels.overrides[RC_THROTTLE] = throttle

        print(f"Alt: {alt:.1f} | Thr: {throttle}")

        if alt >= target_alt * 0.95:
            print("Висота досягнута")
            break

        time.sleep(0.2)


# ------------------ FLIGHT ------------------

def fly_to_point_b(vehicle):
    print("Політ до точки Б...")

    fixed_yaw = vehicle.heading
    vehicle.channels.overrides[RC_YAW] = RC_MID

    while True:
        loc = vehicle.location.global_relative_frame
        lat, lon, alt = loc.lat, loc.lon, loc.alt

        dist = get_distance(lat, lon, POINT_B[0], POINT_B[1])
        bearing = get_bearing(lat, lon, POINT_B[0], POINT_B[1])

        print(f"Dist: {dist:.1f} м | Alt: {alt:.1f}")

        if dist < 2:
            print("Досягли точки Б")
            break

        # --- FIX ANGLE ---
        relative_bearing = normalize_angle(bearing - fixed_yaw)
        bearing_rad = math.radians(relative_bearing)

        # --- НАПРЯМОК ---
        pitch = math.cos(bearing_rad)
        roll  = math.sin(bearing_rad)

        # --- ШВИДКІСТЬ ---
        if dist > 100:
            rc_offset = 120
        elif dist > 30:
            rc_offset = 80
        else:
            rc_offset = 50

        roll_val  = int(RC_MID + roll * rc_offset)
        pitch_val = int(RC_MID - pitch * rc_offset)

        throttle = hold_altitude(vehicle, TARGET_ALT)

        vehicle.channels.overrides[RC_ROLL]     = roll_val
        vehicle.channels.overrides[RC_PITCH]    = pitch_val
        vehicle.channels.overrides[RC_THROTTLE] = throttle

        time.sleep(0.1)   # швидше оновлення


# ------------------ LAND ------------------

def land(vehicle):
    print("Посадка...")

    fixed_yaw = vehicle.heading

    while True:
        loc = vehicle.location.global_relative_frame
        lat, lon, alt = loc.lat, loc.lon, loc.alt

        dist = get_distance(lat, lon, POINT_B[0], POINT_B[1])
        bearing = get_bearing(lat, lon, POINT_B[0], POINT_B[1])

        relative_bearing = normalize_angle(bearing - fixed_yaw)
        bearing_rad = math.radians(relative_bearing)

        pitch = math.cos(bearing_rad)
        roll  = math.sin(bearing_rad)

        rc_offset = max(30, min(80, dist))

        roll_val  = int(RC_MID + roll * rc_offset)
        pitch_val = int(RC_MID - pitch * rc_offset)

        vehicle.channels.overrides[RC_ROLL]  = roll_val
        vehicle.channels.overrides[RC_PITCH] = pitch_val
        vehicle.channels.overrides[RC_THROTTLE] = 1430

        print(f"Landing alt: {alt:.1f} | Dist: {dist:.1f}")

        if alt <= 0.3:
            print("Сіли!")
            break

        time.sleep(0.2)

    vehicle.channels.overrides = {}
    vehicle.armed = False


def main():
    vehicle = connect_vehicle()

    wait_for_gps(vehicle)

    vehicle.home_location = vehicle.location.global_frame

    arm_and_takeoff(vehicle, TARGET_ALT)
    fly_to_point_b(vehicle)
    land(vehicle)

    vehicle.close()
    print("Готово!")


if __name__ == "__main__":
    main()