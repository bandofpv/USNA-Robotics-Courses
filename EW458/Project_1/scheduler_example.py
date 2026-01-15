#!/bin/env python
import time

def readSensor():
    print("Reading sensor")
    # time.sleep(1.0)

def sendMotorCommand():
    print("Sending motor command")
    # time.sleep(0.1)

def main ():
    read_timer = time.time()
    send_timer = time.time()

    while(True):
        if(time.time()-read_timer > 0.1):
            readSensor()
            read_timer = time.time()
        
        if(time.time()-send_timer > 0.5):
            sendMotorCommand()
            send_timer = time.time()

        time.sleep(0.001)

if __name__ == "__main__":
    main()