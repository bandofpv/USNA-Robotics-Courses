import ttyacm
import time
from machine import Pin, ADC

tty = ttyacm.open(1) # open data port

sensor1 = ADC(Pin(27)) # create ADC object for Pin 27
sensor2 = ADC(Pin(26)) # create ADC object for Pin 26

sample_rate = int(tty.readline()) # read sample_rate from MatLab
N = int(tty.readline()) # read number of samples from MatLab

print(f"Sample Rate {sample_rate}")
print(f"Total Number of samples {N}")

# sample_rate = 2
# N = 60

interval = 1/sample_rate # set time interval between temp readings

for i in range(N):
    temp1 = sensor1.read_u16() # read value from sensor 1
    temp2 = sensor2.read_u16() # read value from sensor 2
    
    print(f"Sensor1: {temp1}") # print out temp1
    print(f"Sensor2: {temp2}") #print out temp2
    
    tty.print(temp1)
    tty.print(temp2)
    
    time.sleep(interval) # sleep for given time interval
    
