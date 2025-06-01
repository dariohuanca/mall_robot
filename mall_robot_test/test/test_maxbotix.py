import time
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# --- GPIO Setup for shared trigger (e.g., Pin 4 of all MaxBotix sensors) ---
TRIGGER_PIN = 17
GPIO.cleanup() 
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.output(TRIGGER_PIN, GPIO.LOW)

# --- ADS1115 via I2C ---
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, address=0x49)
ads.gain = 1  # ±4.096V

# --- Define each sensor channel ---
channels = [AnalogIn(ads, getattr(ADS, f'P{i}')) for i in range(4)]

def read_sensor(idx):
    print(f"Reading sensor {idx + 1}...")
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)  # Enable ranging
    time.sleep(0.1)  # Wait for echo to stabilize
    voltage = channels[idx].voltage
    GPIO.output(TRIGGER_PIN, GPIO.LOW)   # Disable after reading

    distance_cm = voltage * 100  # 10mV/cm → 0.01V/cm → V × 100
    return round(distance_cm, 2)

def main():
    try:
        while True:
            for i in range(4):
                dist = read_sensor(i)
                print(f"Sensor {i+1}: {dist} cm")
                time.sleep(0.2)
            print("-" * 40)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
