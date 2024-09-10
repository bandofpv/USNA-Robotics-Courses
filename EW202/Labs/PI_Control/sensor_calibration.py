import ttyacm
import time
from machine import Pin, ADC

tty = ttyacm.open(1) # open data port

sensor = ADC(Pin(26)) # create ADC object for Pin 26

# read 3 values and return the minimum
def sample_sensor():
    val0 = sensor.read_u16()
    time.sleep_us(1500) #IMPORTANT time.sleep(float--> rounded to nearest ms)
    val1 = sensor.read_u16()
    time.sleep_us(1500)
    val2 = sensor.read_u16()
    return min([val0,val1,val2])

while True:
    msg = tty.readline() # read message from MatLab
    print(msg) # print message
    
    if msg == "read": # if the message was "read"
        voltage = sample_sensor() # get sensor ADC reading
        tty.print(voltage) # print ADC reading to serial
