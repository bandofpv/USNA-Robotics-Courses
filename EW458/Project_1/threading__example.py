#!/bin/env python
import time
import threading

def read_sensor():
    while True:
        print("reading Sensor")
        time.sleep(0.5)

def send_command():
    while True:
        print("Sending motor command")
        time.sleep(0.1)

def main():
    read_thread = threading.Thread(target=read_sensor,daemon=True)
    send_thread = threading.Thread(target=send_command,daemon=True)

    read_thread.start()
    send_thread.start()

    while(True):
        print("Robot is Running")
        time.sleep(1)

if __name__ == "__main__":
    main()

