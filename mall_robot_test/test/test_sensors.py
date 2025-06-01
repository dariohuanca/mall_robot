import time
import board
import busio
from smbus2 import SMBus
import serial
import adafruit_bno055
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ina260

# --- I2C Bus Init ---
i2c = busio.I2C(board.SCL, board.SDA)

# -------- INA260 --------
ina260 = adafruit_ina260.INA260(i2c)
'''
# --- BNO055 IMU ---
bno = adafruit_bno055.BNO055_I2C(i2c)

'''
# --- ADS1115 (2x with different I2C addresses) ---
ads1 = ADS.ADS1115(i2c, address=0x48)
ads2 = ADS.ADS1115(i2c, address=0x49)

# Create all 4 analog channels for both ADCs
ads1_channels = [AnalogIn(ads1, getattr(ADS, f'P{i}')) for i in range(4)]
ads2_channels = [AnalogIn(ads2, getattr(ADS, f'P{i}')) for i in range(4)]

# --- GPS PA1616S over UART ---
uart = serial.Serial("/dev/ttyTHS1", baudrate=9600, timeout=1)

def read_gps():
    line = uart.readline().decode('ascii', errors='replace').strip()
    if line.startswith('$GNGGA') or line.startswith('$GPRMC'):
        return line
    return None

# --- Main Loop ---
while True:
    try:
        '''
        # BNO055 orientation data
        print("BNO055 Euler (heading, roll, pitch):", bno.euler)
        '''
        # Read all channels of ADS1115 #1
        for i, ch in enumerate(ads1_channels):
            print(f"ADS1 CH{i}: {ch.voltage:.3f} V", end=" | ")                                 
        print()

        # Read all channels of ADS1115 #2
        for i, ch in enumerate(ads2_channels):
            print(f"ADS2 CH{i}: {ch.voltage:.3f} V", end=" | ")
        print()

        # INA260
        print("INA260 Voltage:", ina260.voltage, "V | Current:", ina260.current, "mA")
        
        # Read GPS line
        gps_data = read_gps()
        if gps_data:
            print("GPS:", gps_data)

        time.sleep(1)

    except Exception as e:
        print("Error:", e)
        time.sleep(1)
