import time
import pygame
import threading
import roslibpy

class joystickClass():
    def __init__(self, id=86):
        # Joystic variables
        self.joystick = None
        self.axes = []
        self.buttons = []
        self.running = True
        
        # Vehicle state variables
        self.armed = False
        self.is_manual_mode = True
        self.is_docked = None
        self.dock_mode = False
        self.hazard_detected = False
        self.dock_id = None
        self.undock_id = None
        self.dock_result = None
        
        # Toggle state variables
        self.arm_start_time = None
        self.mode_start_time = None
        self.dock_start_time = None
        self.hold_duration = 1.0 # seconds to hold button to toggle
        self.tap_duration = 0.05 # seconds to consider a tap
        self.arm_toggled = False
        self.mode_toggled = False
        self.dock_toggled = False

        # ROS variables
        self.id = id
        self.client = roslibpy.Ros(host=f'10.24.6.{self.id}', port=9090)
        self.client.run()
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.pose = None
        self.orientation = None

        # ROS Topics
        self.cmd_vel_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_vel', 'geometry_msgs/Twist')
        self.led_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.beep_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')

        # ROS Subscribers
        self.hazard_sub = roslibpy.Topic(self.client, f'/create_{self.id}/hazard_detection', 'irobot_create_msgs/HazardDetectionVector')
        self.dock_sub = roslibpy.Topic(self.client, f'/create_{self.id}/dock_status', 'irobot_create_msgs/DockStatus')
        self.pose_sub = roslibpy.Topic(self.client, f'/create_{self.id}/pose', 'geometry_msgs/PoseStamped')
        self.dock_sub.subscribe(self.dock_callback)
        self.hazard_sub.subscribe(self.hazard_callback)
        self.pose_sub.subscribe(self.pose_callback)

        # safety_override_param = roslibpy.Param(self.client, 'safety_override')
        # safety_override_param.set('backup_only')
        
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
        self.hazard_detected = True
    
    def dock_callback(self, msg):
        self.is_docked = msg['is_docked']

    def pose_callback(self, msg):
        self.pose = msg['pose']['position']
        self.orientation = msg['pose']['orientation'] 

    def on_result(self, result):
        self.dock_mode = False
        self.dock_result = result

        # Clear docking state
        self.dock_id = None
        self.undock_id = None

    def on_feedback(self, feedback):
        # print(f"Docking Feedback: {feedback}")
        self.dock_mode = True

    def on_error(self, error):
        print(f"Error: {error}")
        self.dock_mode = False
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
                    if self.arm_start_time is None:
                        self.arm_start_time = time.time()
                    elif (time.time() - self.arm_start_time >= self.hold_duration) and not self.arm_toggled:
                        prev_armed = self.armed
                        self.armed = not self.armed
                        # status = "ARMED" if self.armed else "DISARMED"
                        # print(f"\nRobot {status}")
                        self.arm_toggled = True
    
                        # Beep when arming/disarming
                        if self.armed and not prev_armed:
                            self.beep([440, 540, 660], [0.2, 0.2, 0.2])
                        elif not self.armed:
                            self.beep([660, 540, 440], [0.2, 0.2, 0.2])
                else:
                    self.arm_start_time = None
                    self.arm_toggled = False

                # Vehicle mode logic (X button)
                if self.buttons[2]:
                    if self.mode_start_time is None:
                        self.mode_start_time = time.time()
                    elif (time.time() - self.mode_start_time >= self.tap_duration) and not self.mode_toggled:
                        self.is_manual_mode = not self.is_manual_mode
                        # status = "MANUAL MODE" if self.is_manual_mode else "AUTONOMOUS MODE"
                        # print(f"\nRobot in {status}")
                        self.mode_toggled = True

                        # Beep on mode change
                        if self.is_manual_mode:
                            self.beep([660, 660], [0.3, 0.3]) # Higher beep for manual
                        elif not self.is_manual_mode:
                            self.beep([360, 360], [0.3, 0.3]) # Lower beep for autonomous
                else:
                    self.mode_start_time = None
                    self.mode_toggled = False

                # Vehicle mode logic (Y button)
                if self.buttons[3]:
                    if self.dock_start_time is None:
                        self.dock_start_time = time.time()
                    elif (time.time() - self.dock_start_time >= self.hold_duration) and not self.dock_toggled:
                        self.dock_toggled = True

                        if self.is_docked and not self.dock_mode:
                            # print("\nUndocking...")
                            self.undock_id = self.undock_client.send_goal(roslibpy.Message({}), self.on_result, self.on_feedback, self.on_error)
                        elif not self.is_docked and not self.dock_mode:
                            # print("\nDocking...")
                            self.dock_id = self.dock_client.send_goal(roslibpy.Message({}), self.on_result, self.on_feedback, self.on_error)
                        
                        # Doesn't work :(
                        elif self.undock_id and self.dock_mode:
                            self.undock_client.cancel_goal(self.undock_id)
                            while self.dock_result is None:
                                time.sleep(0.1)
                            print("Undock cancelled")
                            self.dock_result = None
                            self.is_manual_mode = True
                        elif self.dock_id and self.dock_mode:
                            self.dock_client.cancel_goal(self.dock_id)
                            while self.dock_result is None:
                                time.sleep(0.1)
                            print("Dock cancelled")
                            self.dock_result = None
                            self.is_manual_mode = True
                else:
                    self.dock_start_time = None
                    self.dock_toggled = False


                # Update movement commands
                if self.joystick and self.is_manual_mode:
                    self.linear_x = self.axes[1] * -1  # Invert Y axis
                    self.angular_z = self.axes[2] * -2 # Invert and scale rotation

            time.sleep(0.1)
        
        pygame.quit()

    def update_robot(self):
        while self.running: 
            if self.client.is_connected:
                self.control_movement(self.linear_x, self.angular_z)
                self.control_leds()

            time.sleep(0.1)

    def control_movement(self, linear_x, angular_z):
        if self.armed:
            twist = roslibpy.Message({
                'linear': { 'x': linear_x },
                'angular': { 'z': angular_z }
            })
            self.cmd_vel_pub.publish(twist)    

    def control_leds(self):
        # Blink green when armed (0.5s on/off), solid red when disarmed
        led_array = []
        if self.armed and self.is_manual_mode:
            # Blink green: toggle every 0.5 seconds
            if int(time.time() / 0.5) % 2 == 0:
                led_color = {'red': 0, 'green': 255, 'blue': 0}  # Green on
            else:
                led_color = {'red': 0, 'green': 0, 'blue': 0}  # Off
        elif self.armed and not self.is_manual_mode:
            # Blink blue: toggle every 0.5 seconds
            if int(time.time() / 0.5) % 2 == 0:
                led_color = {'red': 0, 'green': 0, 'blue': 255}  # Blue on
            else:
                led_color = {'red': 0, 'green': 0, 'blue': 0}  # Off
        elif self.dock_mode:
            led_color = {'red': 255, 'green': 165, 'blue': 0}  # Orange for docking/undocking
        else:
            led_color = {'red': 255, 'green': 0, 'blue': 0}  # Red for disarmed

        if self.hazard_detected:
            led_color = {'red': 255, 'green': 255, 'blue': 0}  # Yellow for hazard
            self.hazard_detected = False  # reset hazard flag after indication

        for i in range(6):
            led_array.append(led_color)

        led_msg = roslibpy.Message({
            'leds': led_array,
            'override_system': True
        })
        self.led_pub.publish(led_msg)

    def beep(self, frequency=[540], duration=[0.5]):
        notes = []
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
        # Turn off LEDs
        led_msg = roslibpy.Message({
            'override_system': False
        })
        self.led_pub.publish(led_msg)

        self.running = False

        self.read_thread.join()
        self.robot_thread.join()

if __name__ == "__main__":
    js = joystickClass()
    print("Main script running. Press Ctrl+C to stop.")

    try:
        while True:
            # This loop represents your "other script" logic
            
            if js.joystick:
                # Formatting for cleaner console output
                axes_str = [f"{val:5.2f}" for val in js.axes]
                status_str = "ARMED" if js.armed else "DISARMED"
                mode_str = "MANUAL" if js.is_manual_mode else "AUTO"
                print(f"\r[{status_str} | {mode_str}] Axes: {axes_str} | Buttons: {js.buttons}", end="", flush=True)
            else:
                print("\rWaiting for joystick connection...", end="", flush=True)
            
            # Your main script can run at a different speed than the joystick update
            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        js.stop()