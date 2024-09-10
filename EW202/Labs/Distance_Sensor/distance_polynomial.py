import time
from machine import Pin, ADC

sensor = ADC(Pin(26)) # create ADC object for Pin 26

# conversion constants
c = 1.474479111075166e-08
e = -0.001134635853031
f = 25.093091714162838

# convert ADC voltage to hole number
def calc_hole(voltage):
    return c*(voltage**2) + e*voltage + f

# convert hole number to height in cm
def calc_height(hole):
    return (14-hole)*2

# This function filters out spurious sensor readings
# time.sleep_us takes microseconds, e.g., 1500 = 1.5 milliseconds

def sample_sensor():
    # read 3 values and return the minimum
    val0 = sensor.read_u16()
    time.sleep_us(1500) #IMPORTANT time.sleep(float--> rounded to nearest ms)
    val1 = sensor.read_u16()
    time.sleep_us(1500)
    val2 = sensor.read_u16()
    return min([val0,val1,val2])

while True:
    voltage = sample_sensor() # get sensor ADC reading
    hole = calc_hole(voltage) # calculate hole number
    height = calc_height(hole) # calculate height in cm
    print(f"Height: {height}cm") # print height
    time.sleep(0.25)