import math
import time
import pygame
import threading
import roslibpy
import numpy as np
from RRT import RRT
import matplotlib.pyplot as plt
from matplotlib.path import Path
from PIL import Image
from scipy import ndimage
from scipy.spatial.transform import Rotation as R

class CreateClass():
    def __init__(self, id=86, map_id=81):
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

        # Initialize empty arrays to be filled with data for plotting
        self.robot_path = [] # path of robot
        self.des_path = None # path from planner in map frame
        self.p_des = None # path in robot frame
        self.rrt_tree_nodes = None # RRT tree nodes for visualization
        self.numWypts = 0 # number of waypoints in path
        
        # Initialize timing variables for performance analysis
        self.rrt_start_time = None
        self.rrt_end_time = None
        self.pose_data = []
        self.des_pose_data = []

        # Occupancy grid map variables
        self.occupancy_grid = None
        self.grid_size = None
        self.width = None; self.height = None
        self.map_x_min = None; self.map_x_max = None; self.map_y_min = None; self.map_y_max = None
        self.robot_radius = 0.175 # m

        # Matplotlib figure/axes for plotting and close handler
        self.fig, self.ax = plt.subplots(figsize=(9, 6))
        self.fig.canvas.mpl_connect('close_event', self.on_plot_close)

        # Define Controller Gains and algorithm constants
        self.wp_num = 0 # waypoint index
        self.wp_rad = 0.075 # waypoint radius
        self.Kp_yaw = 2 # heading gain
        self.Kp_speed = 1 # forward speed gain
        self.yaw_thresh = math.radians(5) # yaw error threshold for stopping in degrees

        self.run_t = 10000000 # run time
        
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
        self.map_id = map_id
        self.client = roslibpy.Ros(host=f'10.24.6.{self.id}', port=9090)
        self.client.run()
        self.map_client = roslibpy.Ros(host=f'10.24.6.{self.map_id}', port=9090)
        self.map_client.run()
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.x = None; self.y = None; self.z = None
        self.roll = None; self.pitch = None; self.yaw = None
        self.map_x = None; self.map_y = None; self.map_z = None

        # ROS Topics
        self.cmd_vel_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_vel', 'geometry_msgs/Twist')
        self.led_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_lightring', 'irobot_create_msgs/LightringLeds')
        self.beep_pub = roslibpy.Topic(self.client, f'/create_{self.id}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')

        # ROS Subscribers
        self.hazard_sub = roslibpy.Topic(self.client, f'/create_{self.id}/hazard_detection', 'irobot_create_msgs/HazardDetectionVector')
        self.dock_sub = roslibpy.Topic(self.client, f'/create_{self.id}/dock_status', 'irobot_create_msgs/DockStatus')
        self.pose_sub = roslibpy.Topic(self.client, f'/create_{self.id}/pose', 'geometry_msgs/PoseStamped')
        self.map_pos_sub = roslibpy.Topic(self.map_client, f'/create_{self.map_id}/pose', 'geometry_msgs/PoseStamped')
        self.map_og_sub = roslibpy.Topic(self.map_client, f'/create_{self.map_id}/occupancy_grid', 'nav_msgs/OccupancyGrid')
        self.dock_sub.subscribe(self.dock_callback)
        self.hazard_sub.subscribe(self.hazard_callback)
        self.pose_sub.subscribe(self.pose_callback)
        self.map_pos_sub.subscribe(self.map_pose_callback)
        self.map_og_sub.subscribe(self.map_occupancy_callback)

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
        self.mowing_sound_thread = threading.Thread(target=self.mowing_sound, daemon=True)
        #self.mowing_sound_thread.start()

        # Wait for the first pose message to be received before proceeding
        print("Waiting for first pose message...")
        while self.x is None or self.y is None:
            time.sleep(0.01)  # Wait for the first pose message to be received
        print("Pose received. Starting main loop.")

    def hazard_callback(self, msg):
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

    def map_pose_callback(self, msg):
        self.map_x = msg['pose']['position']['x']
        self.map_y = msg['pose']['position']['y']
        self.map_z = msg['pose']['position']['z']

        # print(f"Pose Update: x={self.map_x:.2f}, y={self.map_y:.2f}", end="\r", flush=True)

    def map_occupancy_callback(self, msg):
        self.width = msg['info']['width']
        self.height = msg['info']['height']
        self.grid_size = msg['info']['resolution']
        x_half = (self.width * self.grid_size) / 2
        y_half = (self.height * self.grid_size) / 2
        self.map_x_min = -x_half; self.map_x_max = x_half; self.map_y_min = -y_half; self.map_y_max = y_half
        
        # Convert the flattened list back into a 2D numpy array, scale to [0, 1] and invert it
        grid_data = np.array(msg['data']).reshape((self.height, self.width))
        self.occupancy_grid = 1.0 - (grid_data / 100.0)

        # Inflate obstacles in occupancy grid by robot radius to account for robot size in planning
        if self.robot_radius > 0:
            inflation_radius = int(np.ceil(self.robot_radius / self.grid_size))
            inflated_obstacles = ndimage.binary_dilation(self.occupancy_grid <= 0.0, iterations=inflation_radius)
            self.occupancy_grid[inflated_obstacles] = 0.0

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

    def on_plot_close(self, event):
        print("\nPlot window closed. Stopping...")
        try:
            self.stop()
        except Exception as e:
            print(f"Error during stop: {e}")

    def wrapToPi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def compute_rrt_path(self, goal, expand_dis=0.25, max_iter=5000):
        if goal is None:
            goal = [self.map_x, self.map_y] # update goal to latest position of robot in map frame

        start_pos = [self.x, self.y] # start position is where robot is located when RRT is started
        map_params_config = {
            'res': self.grid_size,
            'x_limit': [self.map_x_min, self.map_x_max],
            'y_limit': [self.map_y_min, self.map_y_max],
            'origin': [self.map_x_min, self.map_y_min]
        }
        planner = RRT(start=start_pos, 
                        goal=goal, 
                        map_grid=self.occupancy_grid, 
                        map_params=map_params_config, 
                        expand_dis=expand_dis, 
                        max_iter=max_iter) 
        
        # Run the RRT planning algorithm
        path_result = planner.plan()
        self.rrt_tree_nodes = np.array(planner.tree)
        
        # If a path is found, store it and negate the waypoints to match the robot's coordinate frame
        if path_result is not None:
            self.des_path = np.array(path_result)
            self.p_des = self.des_path # match the robot's actual coordinate frame for navigation
            self.numWypts = len(self.p_des)
            print(f"Path found with {self.numWypts} waypoints")
        else: 
            print("No path found.")
            return 
            # self.stop()  # Stop the robot and threads if no path is found

    def waypoint_navigation(self):
        # If no desired path is set, compute a new RRT path to the goal
        if self.p_des is None:
            self.compute_rrt_path(goal=None) # use latest position as goal if none provided

        # Switch to manual mode if all waypoints reached or if bumped
        if self.wp_num >= self.numWypts:
            u_des = 0; r_des = 0
            self.robot_path = []
            self.p_des = None
            self.des_path = None
            self.rrt_tree_nodes = None
            self.is_manual_mode = True
            self.armed = False
            self.wp_num = 0 # reset waypoints for next time
            self.beep([660, 540, 440], [0.2, 0.2, 0.2])
            print("All waypoints reached.")
            return

        # If the robot is bumped, stop movement and switch to manual mode
        if self.is_bumped:
            u_des = 0; r_des = 0
            self.robot_path = []
            self.p_des = None
            self.des_path = None
            self.rrt_tree_nodes = None
            self.is_manual_mode = True
            self.armed = False
            self.wp_num = 0 # reset waypoints for next time
            self.beep([660, 540, 440], [0.2, 0.2, 0.2])
            print("Bump detected! Stopping movement.")
            return

        # Set Desired Position (waypoint location at time t)
        x_des = self.p_des[self.wp_num, 0]
        y_des = self.p_des[self.wp_num, 1]

        # -------------------- Waypoint Control ------------------

        # Calculate the waypoint control algorithm
        x_error = x_des - self.x
        y_error = y_des - self.y
        psi_des = math.atan2(y_error, x_error) # calculated desired heading
        dist2wp = math.sqrt(x_error**2 + y_error**2)
        u_des = self.Kp_speed*dist2wp # proportional forward speed control
        
        # -----------------Speed Control -----------------------

        psi = self.yaw
        yaw_error = self.wrapToPi(psi_des - psi)
        r_des = self.Kp_yaw * yaw_error # proportional heading control

        # Calculate distance to previous waypoint to determine if we should stop to turn
        if self.wp_num > 0:
            x_des_prev = self.p_des[self.wp_num-1, 0]
            y_des_prev = self.p_des[self.wp_num-1, 1]
            x_error_prev = x_des_prev - self.x
            y_error_prev = y_des_prev - self.y
            dist2wp_prev = math.sqrt(x_error_prev**2 + y_error_prev**2)
        else:
            dist2wp_prev = 0 # No previous waypoint, assume we are at it

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

        # Store data for plotting and analysis
        self.pose_data.append([self.x, self.y, self.yaw])
        self.des_pose_data.append([x_des, y_des, psi_des])
        self.robot_path.append([self.x, self.y]) # store robot path for plotting

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

                    if self.rrt_start_time is None:
                        self.rrt_start_time = time.time() # start RRT timer on first autonomous action
                    self.rrt_end_time = time.time() # update RRT end time
                else:
                    self.control_movement(self.linear_x, self.angular_z)
                    self.robot_path = []
                    self.p_des = None
                    self.des_path = None
                
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

    def plot_occupancy_grid(self):
        if self.occupancy_grid is None:
            return

        self.ax.cla()
        self.ax.imshow(
            self.occupancy_grid,
            extent=[self.map_x_min, self.map_x_max, self.map_y_min, self.map_y_max],
            origin='lower',
            cmap='gray',
            vmin=0.0,
            vmax=1.0,
        )

        # Plot mocap origin.
        # self.ax.plot(0.0, 0.0, 'b*', markersize=10)

        # Plot RRT tree nodes if available
        if self.rrt_tree_nodes is not None and len(self.rrt_tree_nodes) > 0:
            self.ax.plot(self.rrt_tree_nodes[:, 0], self.rrt_tree_nodes[:, 1], 'c.', markersize=2)

        # Plot goal point if available
        if self.des_path is not None and len(self.des_path) > 0:
            goal_point = self.des_path[-1]
            self.ax.plot(goal_point[0], goal_point[1], 'g*', markersize=15)

        # Plot desired path if available
        if self.des_path is not None:
            self.ax.plot(self.des_path[:, 0], self.des_path[:, 1], 'g-', linewidth=2)

            # Highlight current target waypoint
            if self.wp_num < len(self.des_path):
                self.ax.plot(self.des_path[self.wp_num, 0], self.des_path[self.wp_num, 1], 'y.', markersize=10)

        # Plot robot position if available
        if self.x is not None and self.y is not None:
            self.ax.plot(self.x, self.y, 'ro', markersize=5)
            self.ax.arrow(self.x, self.y, 0.5 * math.cos(self.yaw), 0.5 * math.sin(self.yaw), head_width=0.1, head_length=0.1, fc='r', ec='r')

        # Plot mapping robot position if available
        if self.map_x is not None and self.map_y is not None:
            self.ax.plot(self.map_x, self.map_y, 'bx', markersize=5)
        
        # Plot robot path if available
        if len(self.robot_path) > 0:
            robot_path_array = np.array(self.robot_path)
            self.ax.plot(robot_path_array[:, 0], robot_path_array[:, 1], 'r-', linewidth=1)

        # Keep display size and axis limits static.
        self.ax.set_xlim(self.map_x_min, self.map_x_max)
        self.ax.set_ylim(self.map_y_min, self.map_y_max)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Occupancy Grid')
        self.ax.grid()
        plt.pause(0.01)

    def stop(self):
        # Stop the robot and threads
        if not self.running:
            return

        led_msg = roslibpy.Message({
            'override_system': False
        })
        self.led_pub.publish(led_msg)
        self.running = False
        self.read_thread.join()
        self.robot_thread.join()
        # self.mowing_sound_thread.join()
        plt.close('all')

    def plot_results(self):
        if len(self.pose_data) > 0 and len(self.des_pose_data) > 0:
            posistion_error = np.linalg.norm(pose_arr[:, :2] - des_pose_arr[:, :2], axis=1)
            yaw_error = np.abs(self.wrapToPi(pose_arr[:, 2] - des_pose_arr[:, 2]))
            travel_time = self.rrt_end_time - self.rrt_start_time if self.rrt_start_time and self.rrt_end_time else 0
            distance_to_goal = self.des_path[-1] - np.array([self.x, self.y]) if self.des_path is not None else np.array([0, 0])
            print(f"Average Position Error: {np.mean(posistion_error):.3f} m")
            print(f"Average Yaw Error: {math.degrees(np.mean(yaw_error)):.2f} degrees")
            print(f"Travel Time: {travel_time:.2f} s")
            print(f"Distance to Goal: {np.linalg.norm(distance_to_goal):.3f} m")

            # Calculate closest distance between robot path and obstacles
            x_obs = [u**self.grid_size + self.map_x_min for u in range(self.width) for v in range(self.height) if self.occupancy_grid[v, u] <= 0.0]
            y_obs = [v**self.grid_size + self.map_y_min for u in range(self.width) for v in range(self.height) if self.occupancy_grid[v, u] <= 0.0]
            obs_points = np.array(list(zip(x_obs, y_obs)))
            distance_to_obstacles = np.min(np.linalg.norm(obs_points[:, np.newaxis] - self.pose_arr[:, :2], axis=2), axis=0)
            print(f"Minimum Distance to Obstacles: {np.min(distance_to_obstacles):.3f} m")

            '''SAVE DATA TO JSON FILE'''

            print("\nPlotting results...")
            plt.figure(figsize=(9, 6))
            pose_arr = np.array(self.pose_data)
            des_pose_arr = np.array(self.des_pose_data)
            plt.plot(des_pose_arr[:, 0], des_pose_arr[:, 1], 'g--', label='Desired Path')
            plt.plot(pose_arr[:, 0], pose_arr[:, 1], 'r-', label='Robot Path')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title('Robot vs Desired Path')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            plt.show()

if __name__ == "__main__":
    js = CreateClass(id=86, map_id=87)
    print("Main script running. Press Ctrl+C to stop.")

    try:
        while js.running:
            if js.joystick:
                # Formatting for cleaner console output
                axes_str = [f"{val:5.2f}" for val in js.axes]
                status_str = "ARMED" if js.armed else "DISARMED"
                mode_str = "MANUAL" if js.is_manual_mode else "AUTO"
                print(f"\r[{status_str} | {mode_str}] Axes: {axes_str} | Buttons: {js.buttons}", end="", flush=True)
            else:
                print("\rWaiting for joystick connection...", end="", flush=True)

            js.plot_occupancy_grid()

            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        js.stop()
        js.plot_results()