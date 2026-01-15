#!/bin/env python
import time
import threading

class threadClass():
    def __init__(self,name="no_name"):
        self.name = name
        self.my_var = 0.5

        self.read_thread = threading.Thread(target=self.read_sensor,daemon=True)
        self.send_thread = threading.Thread(target=self.send_command,daemon=True)

        self.read_thread.start()
        self.send_thread.start()

    def read_sensor(self):
        while True:
            print(f"{self.name}: reading Sensor")
            time.sleep(0.5)

    def send_command(self):
        while True:
            print(f"{self.name}:Sending motor command")
            time.sleep(0.1)

def main():
    alice = threadClass(name="alice")
    bob = threadClass(name="bob")
    charlie = threadClass(name="charlie")

    while(True):
        print("Robot is Running")
        time.sleep(1)

if __name__ == "__main__":
    main()

