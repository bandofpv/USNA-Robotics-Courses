import math
import time
import pygame
import threading
import roslibpy
import numpy as np
from scipy.spatial.transform import Rotation as R

class CreateClass():
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
        self.hazard_detected = False
        self.dock_id = None
        self.undock_id = None
        self.dock_result = None
        self.is_bumped = False

        # Initialize Empty Arrays to be filled with data for plotting
        self.time = [] # time array
        self.pos_dat = [] # position of vehicle over time
        self.vel_dat = [] # velocity of vehicle over time
        self.eul_dat = [] # Euler angles of vehicle over time
        self.ang_vel_dat = [] # angular velocity of vehicle over 
        self.ctrl_data = [] # robot control commands
        self.des_pos_dat = [] # desired positions over time
        self.des_psi_dat = [] # desired headings over time

        # Define Desired Path (waypoints) 
        self.p_des = np.array([
            [2.50, 0.75],  [2.50, -1.00],  
            [2.15, -1.12], [2.15, 0.60], 
            [1.70, 0.70],  [1.60, -1.06],
            [1.23, -1.15], [1.23, 0.60],
            [0.75, 0.70],  [0.75, -1.05],
            [0.33, -1.12], [0.22, 0.60],
            [-0.15, 0.70],  [-0.25, -1.10],
            [-0.60, -1.16], [-0.70, 0.60],
            [-1.05, 0.70],  [-1.16, -0.15],
            [-1.50, -0.23], [-1.58, 0.57]
        ])

        # Define Controller Gains and algorithm constants
        self.numWypts = np.shape(self.p_des)[0] - 1 # number of waypoints
        self.wp_num = 0 # waypoint index
        self.wp_rad = 0.075 # waypoint radius
        self.Kp_yaw = 2 # heading gain
        self.Kp_speed = 1 # forward speed gain
        self.yaw_thresh = math.radians(5) # yaw error threshold for stopping in degrees

        self.run_t = 10000000 # run time
        self.elapsedTime = 0
        
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
        self.pose_sub = roslibpy.Topic(self.client, f'/create_{self.id}/pose', 'geometry_msgs/PoseStamped')
        self.dock_sub.subscribe(self.dock_callback)
        self.hazard_sub.subscribe(self.hazard_callback)
        self.pose_sub.subscribe(self.pose_callback)

        # Safety Override Service
        saftey_service = roslibpy.Service(self.client, f'/create_{self.id}/motion_control/set_parameters', 'rcl_interfaces/srv/SetParameters')
        override_request = roslibpy.ServiceRequest({
            'parameters': [
                {
                    'name': 'safety_override',
                    'value': {
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
        self.mowing_sound_thread = threading.Thread(target=self.mowing_sound, daemon=True)
        self.mowing_sound_thread.start()

    def hazard_callback(self, msg):
        # print(f"Hazard Detection: {msg}")
        self.is_bumped = any(detection['type'] == 1 for detection in msg['detections']) # type 1 is bump
        self.hazard_detected = True
    
    def dock_callback(self, msg):
        self.is_docked = msg['is_docked']

    def pose_callback(self, msg):
        self.x = msg['pose']['position']['x']
        self.y = msg['pose']['position']['y']
        self.z = msg['pose']['position']['z']

        r = R.from_quat([
            msg['pose']['orientation']['x'],
            msg['pose']['orientation']['y'],
            msg['pose']['orientation']['z'],
            msg['pose']['orientation']['w']
        ])
        self.roll, self.pitch, self.yaw = r.as_euler('xyz', degrees=False)
        self.yaw -= math.pi/2 # adjust for different frame convention
        self.yaw = self.wrapToPi(self.yaw) # wrap yaw to [-pi, pi]
        # print(f"Pose Update: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad", end="\r", flush=True)

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

    def wrapToPi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def waypoint_navigation(self):
        start_time = time.time()

        # Switch to manual mode if all waypoints reached or if bumped
        if self.wp_num > self.numWypts:
            u_des = 0; r_des = 0
            self.is_manual_mode = True
            self.armed = False
            self.wp_num = 0 # reset waypoints for next time
            self.beep([660, 540, 440], [0.2, 0.2, 0.2])
            print("All waypoints reached.")

        if self.is_bumped:
            u_des = 0; r_des = 0
            self.is_manual_mode = True
            self.armed = False
            self.wp_num = 0 # reset waypoints for next time
            self.beep([660, 540, 440], [0.2, 0.2, 0.2])
            print("Bump detected! Stopping movement.")

        # Set Desired Position (waypoint location at time t)
        x_des = self.p_des[self.wp_num, 0]
        y_des = self.p_des[self.wp_num, 1]
        # print(f"Navigating to waypoint {self.wp_num}: ({x_des}, {y_des})")

        # -------------------- Waypoint Control ------------------

        # Calculate the waypoint control algorithm
        x_error = x_des - self.x
        y_error = y_des - self.y
        psi_des = math.atan2(y_error, x_error) # calculated desired heading
        dist2wp = math.sqrt(x_error**2 + y_error**2)
        u_des = self.Kp_speed*dist2wp # proportional forward speed control

        # print(f"\nCurrent Position: ({self.x:.2f}, {self.y:.2f}), Desired Position: ({x_des:.2f}, {y_des:.2f}), Distance to Waypoint: {dist2wp:.2f} m")

        # -------------------------------------------------------
        # -----------------Speed Control -----------------------

        psi = self.yaw
        yaw_error = self.wrapToPi(psi_des - psi)
        r_des = self.Kp_yaw * yaw_error # proportional heading control

        # Calculate distance to previous waypoint to determine if we should stop to turn
        x_des_prev = self.p_des[self.wp_num-1, 0]
        y_des_prev = self.p_des[self.wp_num-1, 1]
        x_error_prev = x_des_prev - self.x
        y_error_prev = y_des_prev - self.y
        dist2wp_prev = math.sqrt(x_error_prev**2 + y_error_prev**2)

        # print(f"Current Heading: {self.yaw:.2f} rad, Desired Heading: {psi_des:.2f} rad, Yaw Error: {yaw_error:.2f} rad")

        # Don't move forward until facing right direction
        if abs(yaw_error) > self.yaw_thresh and dist2wp_prev < self.wp_rad:
            u_des = 0

        # Iterate waypoints after reaching waypoint radius
        if dist2wp < self.wp_rad:
            self.wp_num += 1
        
        # Bound forward speed and turn rate
        u = max(0,min(u_des,0.306))
        r = max(-1.57,min(r_des,1.57))

        # Set the servo and esc command signals must be integers
        self.control_movement(u, r)

        elapsedTime = time.time() - start_time

        # Append data from this time step to arrays add data from this time step to arrays
        self.time.append(elapsedTime);
        self.des_pos_dat.append([x_des, y_des])
        self.des_psi_dat.append(psi_des);
        self.pos_dat.append([self.x, self.y]);
        self.eul_dat.append(self.yaw);
        self.ctrl_data.append([u,r]);
        
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
                        self.is_manual_mode = True # Force manual mode on disconnect
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
                        # Arming is only allowed into manual mode. Disarming is always allowed.
                        if not self.armed and self.is_manual_mode:
                            prev_armed = self.armed
                            self.armed = True
                            self.arm_toggled = True
                            self.mode_switch_time = time.time()
                            self.beep([440, 540, 660], [0.2, 0.2, 0.2])
                        elif self.armed:
                            self.armed = False
                            self.is_manual_mode = True # force manual mode when disrmed
                            self.arm_toggled = True
                            self.mode_switch_time = time.time()
                            self.beep([660, 540, 440], [0.2, 0.2, 0.2])
                # Reset arm timer and toggle state on release
                else:
                    self.arm_start_time = None
                    self.arm_toggled = False

                # Vehicle mode logic (X button)
                if self.buttons[2] and self.armed:
                    # Start timer on initial press
                    if self.mode_start_time is None:
                        self.mode_start_time = time.time()
                    # Check if held long enough to toggle
                    elif (time.time() - self.mode_start_time >= self.tap_duration) and not self.mode_toggled:
                        self.is_manual_mode = not self.is_manual_mode
                        # status = "MANUAL MODE" if self.is_manual_mode else "AUTONOMOUS MODE"
                        # print(f"\nRobot in {status}")
                        self.mode_toggled = True

                        # Beep on mode change
                        self.mode_switch_time = time.time()
                        if self.is_manual_mode and self.armed:
                            self.beep([660, 660], [0.3, 0.3]) # Higher beep for manual
                        elif not self.is_manual_mode and self.armed:
                            self.beep([360, 360], [0.3, 0.3]) # Lower beep for autonomous
                # Reset mode timer and toggle state on release 
                else:
                    self.mode_start_time = None
                    self.mode_toggled = False

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
                if self.joystick and self.is_manual_mode:
                    self.linear_x = self.axes[1] * -1  # Invert Y axis
                    self.angular_z = self.axes[2] * -2 # Invert and scale rotation
            time.sleep(0.1)
        pygame.quit()

    def update_robot(self):
        while self.running: 
            if self.client.is_connected:
                self.control_leds()
                # Autonomous waypoint navigation
                if self.armed and not self.is_manual_mode:
                    self.waypoint_navigation()
                else:
                    self.control_movement(self.linear_x, self.angular_z)
            time.sleep(0.1) # 10 Hz update rate

    def mowing_sound(self):
        while self.running:
            # Check for conflict with transition sounds (wait 1.5s)
            if time.time() - self.mode_switch_time > 1.5:
                if self.client.is_connected and self.armed and not self.is_manual_mode:
                    self.beep([540], [0.1]) # Audio Cue for blade spining
            time.sleep(0.45)

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
        if self.armed and self.is_manual_mode:
            led_color = {'red': 0, 'green': 255, 'blue': 0}  # Green for manual mode
        elif self.armed and not self.is_manual_mode:
            # Blink yellow: toggle every 0.5 seconds
            if int(time.time() / 0.5) % 2 == 0:
                led_color = {'red': 255, 'green': 255, 'blue': 0}  # Flashing Yellow for autonomous mode
            else:
                led_color = {'red': 0, 'green': 0, 'blue': 0}  # Off
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
        self.mowing_sound_thread.join()

if __name__ == "__main__":
    js = CreateClass()
    print("Main script running. Press Ctrl+C to stop.")

    try:
        while True:
            if js.joystick:
                # Formatting for cleaner console output
                axes_str = [f"{val:5.2f}" for val in js.axes]
                status_str = "ARMED" if js.armed else "DISARMED"
                mode_str = "MANUAL" if js.is_manual_mode else "AUTO"
                print(f"\r[{status_str} | {mode_str}] Axes: {axes_str} | Buttons: {js.buttons}", end="", flush=True)
            else:
                print("\rWaiting for joystick connection...", end="", flush=True)

            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        js.stop()