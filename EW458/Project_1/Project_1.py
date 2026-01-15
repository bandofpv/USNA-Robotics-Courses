#!/bin/env python
import time
import threading
import keyboard
import winsound
import os

class musicClass():
    def __init__(self):
        # Allow continuous sound by using PlaySound
        winsound.PlaySound("SystemExit", winsound.SND_ALIAS | winsound.SND_ASYNC)        

        self.play_now = False
        self.freq = 37
        self.freq_map = {
            '1': 440,
            '2': 540,
            '3': 640,
            '4': 740,
            '5': 840,
        }

        # Set up callback for key release
        for key in self.freq_map.keys():
            keyboard.on_release_key(key, self.stop_playing)

        self.read_thread = threading.Thread(target=self.read_keyboard,daemon=True)
        self.send_thread = threading.Thread(target=self.play_note,daemon=True)

        self.read_thread.start()
        self.send_thread.start()

    def stop_playing(self, event):
        self.play_now = False

    def read_keyboard(self):
        print("Press keys 1-5 to play sounds.")
        while True:
            # Check if any of the keys in the freq_map are pressed
            for key, freq in self.freq_map.items():
                if keyboard.is_pressed(key):
                    self.freq = freq # update the frequency
                    self.play_now = True
            
            time.sleep(0.01)

    def play_note(self):
        playing = False
        last_freq = 0
        
        while True:
            if self.play_now:
                # If sound isn't playing or frequency has changed, update sound
                if not playing or last_freq != self.freq:
                    script_dir = os.path.dirname(os.path.abspath(__file__))
                    filename = os.path.join(script_dir, "wavs", f"{self.freq}.wav")
                    
                    # Play from file, async, and loop for continuous sound
                    winsound.PlaySound(filename, winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_LOOP)

                    playing = True
                    last_freq = self.freq
            else:
                # Stop sound if it was playing
                if playing:
                    winsound.PlaySound(None, winsound.SND_ASYNC)
                    playing = False
            
            time.sleep(0.01)

def main():
    musicClass()

    while(True):
        time.sleep(1)

if __name__ == "__main__":
    main()