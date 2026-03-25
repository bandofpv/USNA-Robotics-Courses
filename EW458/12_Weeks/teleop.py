import math
import time
import pygame
import threading
import roslibpy
import numpy as np
from scipy.spatial.transform import Rotation as R

class CreateClass():
    def __init__(self, turn_scale=1.0, id=86):
        # Turn scale allows you to adjust how aggressively the robot turns in response to joystick input
        self.turn_scale = turn_scale

        # Joystic variables
        self.joystick = None
        self.axes = []
        self.buttons = []
        self.running = True
        
        # Vehicle state variables
        self.armed = False
        self.is_docked = None
        self.hazard_detected = False
        self.dock_id = None
        self.undock_id = None
        self.dock_result = None
        self.is_bumped = False

        # Toggle state variables
        self.arm_start_time = None
        self.mode_start_time = None
        self.dock_start_time = None
        self.hold_duration = 1.0 # seconds to hold button to toggle
        self.tap_duration = 0.05 # seconds to consider a tap
        self.arm_toggled = False
        self.mode_toggled = False
        self.dock_toggled = False
        self.last_beep_time = 0
        self.mode_switch_time = 0

        # ROS variables
        self.id = id
        self.client = roslibpy.Ros(host=f'10.24.6.{self.id}', port=9090)
        self.client.run()
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.x = None; self.y = None; self.z = None
        self.roll = None; self.pitch = None; self.yaw = None

        # ROS Topics
        self.cmd_vel_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_vel', 'geometry_msgs/Twist')
        self.led_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.beep_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')

        # ROS Subscribers
        self.hazard_sub = roslibpy.Topic(self.client, f'/create_{self.id}/hazard_detection', 'irobot_create_msgs/HazardDetectionVector')
        self.dock_sub = roslibpy.Topic(self.client, f'/create_{self.id}/dock_status', 'irobot_create_msgs/DockStatus')
        self.dock_sub.subscribe(self.dock_callback)
        self.hazard_sub.subscribe(self.hazard_callback)

        # Safety Override Service
        saftey_service = roslibpy.Service(self.client, f'/create_{self.id}/motion_control/set_parameters', 'rcl_interfaces/srv/SetParameters')
        override_request = roslibpy.ServiceRequest({
            'parameters': [
                {
                    'name': 'safety_override',
                    'value': {
                        'type': 4, # string type
                        'string_value': 'backup_only'
                    }
                }
            ]
        })
        saftey_service.call(override_request)

        # ROS Action Clients
        self.dock_client = roslibpy.ActionClient(self.client, f'/create_{self.id}/dock', 'irobot_create_msgs/Dock')
        self.undock_client = roslibpy.ActionClient(self.client, f'/create_{self.id}/undock', 'irobot_create_msgs/Undock')
        
        # Start threads for joystick reading and robot updating
        self.read_thread = threading.Thread(target=self.read_joystick, daemon=True)
        self.read_thread.start()
        self.robot_thread = threading.Thread(target=self.update_robot, daemon=True)
        self.robot_thread.start()

    def hazard_callback(self, msg):
        # print(f"Hazard Detection: {msg}")
        self.is_bumped = any(detection['type'] == 1 for detection in msg['detections']) # type 1 is bump
        self.hazard_detected = True
    
    def dock_callback(self, msg):
        self.is_docked = msg['is_docked']

    def on_result(self, result):
        self.dock_result = result

        # Clear docking state
        self.dock_id = None
        self.undock_id = None

    def on_feedback(self, feedback):
        # print(f"Docking Feedback: {feedback}")
        pass

    def on_error(self, error):
        print(f"Error: {error}")
        self.dock_result = None
        
        # Clear docking state
        self.dock_id = None
        self.undock_id = None

    def read_joystick(self):
        pygame.init()
        pygame.joystick.init()

        while self.running:
            # Handle hotplugging
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    if self.joystick is None:
                        self.joystick = pygame.joystick.Joystick(event.device_index)
                        self.joystick.init()
                        print(f"\nJoystick {self.joystick.get_name()} connected")
                        
                if event.type == pygame.JOYDEVICEREMOVED:
                    if self.joystick and self.joystick.get_instance_id() == event.instance_id:
                        print(f"\nJoystick {self.joystick.get_name()} disconnected")
                        self.joystick = None
                        self.axes = []
                        self.buttons = []
                        self.armed = False # Disarm on disconnect

            # Update joystick state
            if self.joystick:
                # Update Axes
                num_axes = self.joystick.get_numaxes()
                self.axes = [self.joystick.get_axis(i) for i in range(num_axes)]

                # Update Buttons
                num_buttons = self.joystick.get_numbuttons()
                self.buttons = [self.joystick.get_button(i) for i in range(num_buttons)]

                # Arming logic (A button)
                if self.buttons[0]:
                    # Start timer on initial press
                    if self.arm_start_time is None:
                        self.arm_start_time = time.time()
                    # Check if held long enough to toggle
                    elif (time.time() - self.arm_start_time >= self.hold_duration) and not self.arm_toggled:
                        if not self.armed:
                            prev_armed = self.armed
                            self.armed = True
                            self.arm_toggled = True
                            self.mode_switch_time = time.time()
                            self.beep([440, 540, 660], [0.2, 0.2, 0.2])
                        elif self.armed:
                            self.armed = False
                            self.arm_toggled = True
                            self.mode_switch_time = time.time()
                            self.beep([660, 540, 440], [0.2, 0.2, 0.2])
                # Reset arm timer and toggle state on release
                else:
                    self.arm_start_time = None
                    self.arm_toggled = False

                # Vehicle dock logic (Y button)
                if self.buttons[3]:
                    # Start timer on initial press
                    if self.dock_start_time is None:
                        self.dock_start_time = time.time()
                    # Check if held long enough to toggle
                    elif (time.time() - self.dock_start_time >= self.hold_duration) and not self.dock_toggled:
                        self.dock_toggled = True
                        # Toggle between dock and undock based on current state
                        if self.is_docked:
                            # print("\nUndocking...")
                            self.undock_id = self.undock_client.send_goal(roslibpy.Message({}), self.on_result, self.on_feedback, self.on_error)
                        else:
                            # print("\nDocking...")
                            self.dock_id = self.dock_client.send_goal(roslibpy.Message({}), self.on_result, self.on_feedback, self.on_error)
                # Reset dock timer and toggle state on release 
                else:
                    self.dock_start_time = None
                    self.dock_toggled = False

                # Update movement commands
                if self.joystick and self.armed:
                    self.linear_x = self.axes[1] * -1  # Invert Y axis
                    self.angular_z = self.axes[2] * -self.turn_scale  # Invert and scale rotation
            time.sleep(0.1)
        pygame.quit()

    def update_robot(self):
        while self.running: 
            if self.client.is_connected:
                self.control_leds()
                self.control_movement(self.linear_x, self.angular_z)
            time.sleep(0.1) # 10 Hz update rate

    def control_movement(self, linear_x, angular_z):
        # Only send movement commands if armed
        if self.armed:
            twist = roslibpy.Message({
                'linear': { 'x': linear_x },
                'angular': { 'z': angular_z }
            })
            self.cmd_vel_pub.publish(twist)    

    def control_leds(self):
        # Green when armed (0.5s on/off), solid red when disarmed
        led_array = []
        if self.armed:
            led_color = {'red': 0, 'green': 255, 'blue': 0}  # Green for armed
        else:
            led_color = {'red': 0, 'green': 0, 'blue': 255}  # Blue for Idle

        if self.hazard_detected:
            led_color = {'red': 255, 'green': 255, 'blue': 0}  # Yellow for hazard
            self.hazard_detected = False  # reset hazard flag after indication

        for _ in range(6):
            led_array.append(led_color)

        led_msg = roslibpy.Message({
            'leds': led_array,
            'override_system': True
        })
        self.led_pub.publish(led_msg)

    def beep(self, frequency=[540], duration=[0.5]):
        notes = []
        # Create a note for each frequency and duration pair
        for i in range(len(frequency)):
            note = {
                'frequency': frequency[i],
                'max_runtime': {'sec': int(duration[i]), 'nanosec': int((duration[i] % 1) * 1e9)}
            }
            notes.append(note)
            
        audio_msg = roslibpy.Message({
            'notes': notes
        })
        self.beep_pub.publish(audio_msg)

    def stop(self):
        # Stop the robot and threads
        led_msg = roslibpy.Message({
            'override_system': False
        })
        self.led_pub.publish(led_msg)
        self.running = False
        self.read_thread.join()
        self.robot_thread.join()

if __name__ == "__main__":
    js = CreateClass(turn_scale=0.2, id=86)  # Adjust turn_scale and id as needed
    print("Main script running. Press Ctrl+C to stop.")

    try:
        while True:
            if js.joystick:
                # Formatting for cleaner console output
                axes_str = [f"{val:5.2f}" for val in js.axes]
                status_str = "ARMED" if js.armed else "DISARMED"
                print(f"\r[{status_str}] Axes: {axes_str} | Buttons: {js.buttons}", end="", flush=True)
            else:
                print("\rWaiting for joystick connection...", end="", flush=True)

            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        js.stop()