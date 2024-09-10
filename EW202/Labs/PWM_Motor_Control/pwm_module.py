from machine import Pin, PWM
import ttyacm

tty = ttyacm.open(1)

motor = PWM(Pin(16)) # DC motor control from GPIO16
motor.freq(1000) # set frequency to 1KHz -- DO NOT CHANGE
motor.duty_u16(32768) # 0-65535 for duty cycle range 0-100

######################################################
# NOTE: The DC motor of the fan is NOT BRUSHED       #
# Keep the duty cycle above 35%                      #
######################################################

# YOUR CODE HERE

duty_cycle = 0.70 # set duty cycle

while True:
#     duty_cycle = float(tty.readline()) # read duty cycle value from the serial port
#     print(duty_cycle) # print out read duty cycle
    convert_dc = int(duty_cycle * 65535) # convert duty cycle to an integer 0 to 65535
    motor.duty_u16(convert_dc) # set motor pin to new duty cycle
    if duty_cycle == -1: # terminate loop if input read from serial port is -1
        break
    
motor.duty_u16(0) # upon termination, set motor pin to duty cycle 0