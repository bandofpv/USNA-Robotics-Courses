#!/bin/env python
import time
import threading
import winsound

class musicClass():
    def __init__(self,name="no_name"):
        self.name = name
        self.play_now = False
        self.freq = 37
        self.dur = 0

        self.read_thread = threading.Thread(target=self.read_keyboard,daemon=True)
        self.send_thread = threading.Thread(target=self.play_note,daemon=True)

        self.read_thread.start()
        self.send_thread.start()

    def read_keyboard(self):
        while True:
            self.freq = input("Enter the Frequency in Hz \r\n")
            self.dur = input("Enter the Duration in ms \r\n ")
            self.play_now = True

    def play_note(self):
        while True:
            if(self.play_now):
                winsound.Beep(int(self.freq),int(self.dur))
                self.play_now = False

def main():
    song_loop = musicClass(name="alice")

    while(True):
        # print("Robot is Running")
        time.sleep(1)

if __name__ == "__main__":
    main()