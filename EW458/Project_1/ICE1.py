#!/bin/env python
import time
import threading
import keyboard
import winsound

class musicClass():
    def __init__(self):
        self.play_now = False
        self.freq = 37
        self.dur = 1000
        self.freq_map = {
            '1': 440,
            '2': 540,
            '3': 640,
            '4': 740,
            '5': 840,
        }
        self.dur_map = {
            '+': 250,
            '-': -250 
        }

        self.read_thread = threading.Thread(target=self.read_keyboard,daemon=True)
        self.send_thread = threading.Thread(target=self.play_note,daemon=True)

        self.read_thread.start()
        self.send_thread.start()

    def read_keyboard(self):
        print("Press keys 1-5 to set frequency, and up/down arrows to adjust duration. Press 'q' to quit.")
        while True:
            for key, freq in self.freq_map.items():
                if keyboard.is_pressed(key):
                    self.freq = freq # update the frequency
                    # print(f"Frequency: {self.freq} Hz")
                    self.play_now = True
                    time.sleep(0.05) # delay to prevent multiple updates from a single key press

            for key, dur in self.dur_map.items():
                if keyboard.is_pressed(key):
                    self.dur += dur # update the duration
                    print(f"Duration: {self.dur} ms")
                    time.sleep(0.1) # delay to prevent multiple updates from a single key press

            time.sleep(0.01)

    def play_note(self):
        while True:
            if(self.play_now):
                print(f"Playing note with frequency {self.freq} Hz and duration {self.dur} ms") # print the frequency and duration of the note being played
                winsound.Beep(int(self.freq),int(self.dur))
                self.play_now = False

            time.sleep(0.01)

def main():
    musicClass()

    while(True):
        time.sleep(1)

if __name__ == "__main__":
    main()