import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal

# --- Точки ---
POINT_A = (50.450739, 30.461242)
POINT_B = (50.443326, 30.448078)
TARGET_ALT = 200.0
CRUISE_SPEED = 5.0  # м/с

# RC канали
RC_ROLL    = 1
RC_PITCH   = 2
RC_YAW     = 4
RC_THROTTLE= 3

RC_MID     = 1500
RC_NEUTRAL_THROTTLE = 1500

def connect_vehicle():
    print("Підключення до SITL...")
    vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
    print("Підключено!")
    return vehicle

def arm_and_takeoff(vehicle, target_alt):
    print("Перевірка армінгу...")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(1)

    while not vehicle.is_armable:
        print("Чекаємо готовності...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("Армінг...")
        time.sleep(1)
    print("Заармовано!")

    print(f"Зліт до {target_alt}м...")

    for throttle in range(1000, 1750, 10):
        vehicle.channels.overrides[RC_THROTTLE] = throttle
        time.sleep(0.05)

    vehicle.channels.overrides[RC_THROTTLE] = 1850

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Висота: {alt:.1f}м")
        if alt >= target_alt * 0.95:
            print("Висота досягнута!")
            break
        # Якщо починає падати — додаємо газу
        if alt < target_alt * 0.5:
            vehicle.channels.overrides[RC_THROTTLE] = 1900
        time.sleep(0.5)

    print("Утримуємо висоту 1580...")
    vehicle.channels.overrides[RC_THROTTLE] = 1580
    time.sleep(2)

    # Летимо вгору на повному газу
    vehicle.channels.overrides[RC_THROTTLE] = 1900

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Висота: {alt:.1f}м")
        if alt >= target_alt * 0.95:
            print("Висота досягнута!")
            break
        time.sleep(0.5)

    # Hover throttle — підбираємо 1530-1560 для SITL
    vehicle.channels.overrides[RC_THROTTLE] = 1580


    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Висота: {alt:.1f}м")
        if alt >= target_alt * 0.95:
            print("Висота досягнута!")
            break
        time.sleep(0.5)

    # Утримуємо висоту
    vehicle.channels.overrides[RC_THROTTLE] = RC_NEUTRAL_THROTTLE

def get_bearing(lat1, lon1, lat2, lon2):
    """Кут від точки А до точки Б в градусах"""
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360

def get_distance(lat1, lon1, lat2, lon2):
    """Відстань між точками в метрах"""
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

def fly_to_point_b(vehicle):
    print("Летимо до точки Б...")

    # Фіксуємо Yaw один раз на весь політ
    bearing = get_bearing(POINT_A[0], POINT_A[1], POINT_B[0], POINT_B[1])
    print(f"Напрямок до точки Б: {bearing:.1f}°")

    # Розкладаємо напрямок на roll/pitch
    bearing_rad = math.radians(bearing)
    pitch_component = math.cos(bearing_rad)  # вперед/назад
    roll_component  = math.sin(bearing_rad)  # ліво/право

    # RC відхилення для руху
    RC_OFFSET = 80  # чим більше — тим швидше

    roll_val  = int(RC_MID + roll_component  * RC_OFFSET)
    pitch_val = int(RC_MID - pitch_component * RC_OFFSET)  # мінус бо pitch вперед = менше 1500

    # Фіксований Yaw весь політ
    vehicle.channels.overrides[RC_YAW] = RC_MID

    print(f"RC Roll={roll_val}, Pitch={pitch_val}, Yaw={RC_MID} (фіксований)")

    while True:
        loc = vehicle.location.global_relative_frame
        dist = get_distance(loc.lat, loc.lon, POINT_B[0], POINT_B[1])
        alt  = loc.alt
        print(f"Відстань до Б: {dist:.1f}м | Висота: {alt:.1f}м")

        if dist < 10:
            print("Точка Б досягнута!")
            break

        # Утримуємо висоту + летимо в напрямку
        vehicle.channels.overrides[RC_ROLL]     = roll_val
        vehicle.channels.overrides[RC_PITCH]    = pitch_val
        vehicle.channels.overrides[RC_THROTTLE] = RC_NEUTRAL_THROTTLE

        time.sleep(0.3)

    # Зупиняємось
    vehicle.channels.overrides[RC_ROLL]  = RC_MID
    vehicle.channels.overrides[RC_PITCH] = RC_MID

def land(vehicle):
    print("Посадка в точці Б...")
    vehicle.channels.overrides[RC_THROTTLE] = 1400

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Висота при посадці: {alt:.1f}м")
        if alt <= 0.3:
            print("Посадка завершена!")
            break
        time.sleep(0.5)

    vehicle.channels.overrides = {}
    vehicle.armed = False
    print("Дрон роззброєно.")

def main():
    vehicle = connect_vehicle()

    print("Встановлюємо стартову позицію...")
    vehicle.home_location = vehicle.location.global_frame

    arm_and_takeoff(vehicle, TARGET_ALT)
    fly_to_point_b(vehicle)
    land(vehicle)

    vehicle.close()
    print("Готово!")

if __name__ == "__main__":
    main()
