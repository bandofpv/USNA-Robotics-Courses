import time
import digitalio
import board

led = digitalio.DigitalInOut(board.GP15)
led.direction = digitalio.Direction.OUTPUT
while True:
    for x in range(3):
        led.value = True
        time.sleep(0.25)
        led.value = False
        time.sleep(0.25)
    time.sleep(1)
    for x in range(3):
        led.value = True
        time.sleep(1)
        led.value = False
        time.sleep(0.25)
    time.sleep(1)
    for x in range(3):
        led.value = True
        time.sleep(0.25)
        led.value = False
        time.sleep(0.25)
    time.sleep(2)
