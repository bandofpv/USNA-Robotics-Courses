import ttyacm
import time
from machine import Pin, ADC

tty = ttyacm.open(1) # open data port

sensor = ADC(Pin(26)) # create ADC object for Pin 26

while True:
    msg = tty.readline() # read message from MatLab
    print(msg) # print message
    
    if msg == "read": # if the message was "read"
        voltage = sensor.read_u16() # get sensor ADC reading
        tty.print(voltage) # print ADC reading to serial