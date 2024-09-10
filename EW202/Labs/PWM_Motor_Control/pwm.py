from machine import Pin
import time

led = Pin(16, Pin.OUT)  # set GPIO16 to an output

period = 4  # set period length (seconds)
duty_cycle = 0.25  # set duty cycle (%)

# loop 10 times
for i in range(10):
    led.high()  # turn on GPIO pin 
    print("ON")
    print(led.value()) # digital logic level of pin,
                        # returning 0 or 1 corresponding to low and high voltage signals
    time.sleep(period*duty_cycle)  # sleep for duty cycle percent of period length
    led.low()  # turn off GPIO pin
    print("off")
    print(led.value()) # digital logic level of pin
    time.sleep(period - period*duty_cycle)  # sleep for rest of period length