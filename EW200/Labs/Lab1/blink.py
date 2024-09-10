import time
import digitalio
import board

led = digitalio.DigitalInOut(board.GP15)
led.direction = digitalio.Direction.OUTPUT
for x in range(3):
    led.value = True
    print("ON")
    time.sleep(1)
    led.value = False
    print("OFF")
    time.sleep(1)
