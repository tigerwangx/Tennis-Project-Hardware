import time
import board
import busio
import adafruit_bno055
import math
import neopixel

# BLE Libraries
from adafruit_ble import BLERadio
from adafruit_ble.services.nordic import UARTService
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement

# === BLE Setup ===
ble = BLERadio()
ble.name = "TennisTracker"
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)

# Status LED
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.2)
pixel[0] = (255, 0, 0)  # red = waiting

print("Starting BLE advertising...")
ble.start_advertising(advertisement)

# === Sensor Setup ===
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

def magnitude(vector):
    if vector is None:
        return 0
    return math.sqrt(sum(x**2 for x in vector))

# === Calibration Function ===
def calibrate():
    uart.write("Calibration mode: Swing repeatedly in your hitting direction...\n".encode())
    print("Calibration mode: Swing repeatedly in your hitting direction...")
    print("Watching for consistent directional spikes...\n")

    DURATION = 7  # seconds
    ACCEL_THRESHOLD = 10.0
    POSITIVE_STREAKS = [0, 0, 0]
    NEGATIVE_STREAKS = [0, 0, 0]

    start = time.monotonic()
    while time.monotonic() - start < DURATION:
        lin_accel = sensor.linear_acceleration
        if lin_accel is None:
            continue
        for i in range(3):
            if lin_accel[i] > ACCEL_THRESHOLD:
                POSITIVE_STREAKS[i] += 1
            elif lin_accel[i] < -ACCEL_THRESHOLD:
                NEGATIVE_STREAKS[i] += 1
        print(f"X: {lin_accel[0]:.1f}, Y: {lin_accel[1]:.1f}, Z: {lin_accel[2]:.1f}")
        time.sleep(0.1)

    max_pos = max(POSITIVE_STREAKS)
    max_neg = max(NEGATIVE_STREAKS)
    if max_pos >= max_neg:
        dominant_index = POSITIVE_STREAKS.index(max_pos)
        sign = 1
    else:
        dominant_index = NEGATIVE_STREAKS.index(max_neg)
        sign = -1

    axis_names = ['X', 'Y', 'Z']
    result = f"\nCalibration complete.\nDominant axis: {axis_names[dominant_index]}, Direction: {'+' if sign == 1 else '-'}\n"
    print(result)
    uart.write(result.encode())
    return dominant_index, sign


# === Wait for BLE Connection and "start" Command ===
print("Waiting for BLE connection...")

while not ble.connected:
    pixel[0] = (255, 0, 0)
    if not ble.advertising:
        ble.start_advertising(advertisement)
    time.sleep(0.5)
pixel[0] = (0, 0, 255)  # connected

print("Bluetooth connected...\n")
print("Enter racket weight in grams:")
uart.write(b"Enter racket weight in grams:\n")
#------------------------------------------------------------------------ HERE
uart_buffer = b""
racketweight = 0
while True:
    if uart.in_waiting:
        uart_buffer += uart.read(uart.in_waiting)
        racketweight = int(uart_buffer.decode("utf-8").strip()) / 1000
        print("received racket weight: " + str(racketweight))
        uart.write("received racket weight\n".encode())
        break

    time.sleep(0.1)



print("...Type 'start' to begin calibration")

uart_buffer = b""
uart.write("...Type 'start' to begin calibration".encode())
while True:
    if uart.in_waiting:
        uart_buffer += uart.read(uart.in_waiting)
        if b"start" in uart_buffer.lower():
            uart.write("Starting calibration...\n".encode())
            print("Received 'start' command.")
            break
    time.sleep(0.1)




# === Initial Calibration ===
dominant_index, sign = calibrate()

# === Swing Detection Setup ===
SWING_THRESHOLD = 8.0
SWING_END_DELAY = 0.25
peak_power = 0
cumulative_speed = 0
cumulative_spin = 0
swing_active = False
swing_start_time = None
last_swing_time = time.monotonic()
uart_buffer = b""  # clear buffer for command listening

# Variables to calculate for averages
swing_count = 0
total_power = 0
total_speed = 0
total_spin = 0

# === Main BLE and Swing Loop ===
while True:
    if ble.connected:
        pixel[0] = (0, 0, 255)
    else:
        pixel[0] = (255, 0, 0)
        if not ble.advertising:
            ble.start_advertising(advertisement)

    if uart.in_waiting:
        uart_buffer += uart.read(uart.in_waiting)
        text = uart_buffer.decode("utf-8").strip().lower()

        if "rc" in text or "recalibrate" in text:
            uart.write("Recalibrating...\n".encode())
            print("Received 'recalibrate' command.")
            dominant_index, sign = calibrate()
            uart_buffer = b""

        elif "avg" in text or "average" in text:
            if swing_count > 0:
                avg_power = total_power / swing_count
                avg_speed = total_speed / swing_count
                avg_spin = total_spin / swing_count
                result = (
                    "\nAVERAGES (" + str(swing_count) + " swings):\n" +
                    "Power: " + str(round(avg_power, 2)) + "\n" +
                    "Speed: " + str(round(avg_speed, 2)) + "\n" +
                    "Spin: " + str(round(avg_spin, 2)) + "\n" +
                    "-------------\n"
                )
            else:
                result = "\nNo swings recorded yet.\n"
            uart.write(result.encode())
            print(result)
            uart_buffer = b""

        elif "clr" in text or "clear" in text:
            swing_count = 0
            total_power = 0
            total_speed = 0
            total_spin = 0
            uart.write("Swing session averages cleared.\n".encode())
            print("Session cleared.")
            uart_buffer = b""

    lin_accel = sensor.linear_acceleration
    gyro = sensor.gyro
    if lin_accel is None:
        continue

    axis_value = lin_accel[dominant_index] * sign
    lin_mag = magnitude(lin_accel)
    gyro_z = abs(gyro[2]) if gyro else 0
    current_time = time.monotonic()

    if axis_value > SWING_THRESHOLD:
        if not swing_active:
            print("Swing started")
            swing_active = True
            swing_start_time = current_time
            peak_power = 0
            cumulative_speed = 0
            cumulative_spin = 0

        peak_power = racketweight * axis_value
        cumulative_speed = max(cumulative_speed, axis_value)
        cumulative_spin += gyro_z * 0.05
        last_swing_time = current_time

    elif swing_active and (current_time - last_swing_time > SWING_END_DELAY):
        swing_duration = current_time - swing_start_time

        print("\n SWING DETECTED")
        print("Duration:", round(swing_duration, 2), "s")
        print("Power Index:", round(peak_power, 2), "N")
        print("Speed Index:", round(cumulative_speed, 2))
        print("Spin Index:", round(cumulative_spin, 2))
        print("------------\n")

        swing_count += 1
        total_power += peak_power
        total_speed += cumulative_speed
        total_spin += cumulative_spin

                # BLE UART Output
        if ble.connected:
            message = (
                "SWING DETECTED\n"
                "Duration: " + str(round(swing_duration, 2)) + " s\n"
                "Power Index: " + str(round(peak_power, 2)) + " N\n"
                "Speed Index: " + str(round(cumulative_speed, 2)) + "\n"
                "Spin Index: " + str(round(cumulative_spin, 2)) + "\n"
                "-------------\n"


            )
            uart.write(message.encode())

        swing_active = False

    time.sleep(0.05)

