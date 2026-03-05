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

        # Initialize empty arrays to be filled with data for plotting
        self.robot_path = [] # path of robot
        self.des_path = None # path from planner in map frame
        self.p_des = None # path in robot frame
        self.rrt_tree_nodes = None # RRT tree nodes for visualization

        # Occupancy grid map (black/white grayscale image)
        self.map = Image.open('Project_4/Occupancy208_100m2p.png').convert('L')
        self.base_occupancy_grid = np.array(self.map, dtype=float) / 255.0
        self.occupancy_grid = self.base_occupancy_grid.copy()

        # Define map scaling and mocap origin.
        self.map_wall = 0.75 # meters from image top-left corner to wall
        self.meters_per_pixel = 0.01 * (245/369) # 100 pixels per meter #=0.0066
        # Mocap origin (pose topic frame) measured from image top-left corner [x_right, y_down] in meters.
        self.origin = [5.3848 + self.map_wall, 3.0988 + self.map_wall] # [x, y] in meters
        grid_height, grid_width = self.base_occupancy_grid.shape
        self.map_x_min = -self.origin[0]
        self.map_x_max = self.map_x_min + grid_width * self.meters_per_pixel
        self.map_y_max = self.origin[1]
        self.map_y_min = self.map_y_max - grid_height * self.meters_per_pixel

        # Define obstacles in the map [x, y, theta] in meters and radians
        box_poses = [[-0.45, 0.45, 0.0], [-2.74, 0.0, 0.0], [1.85, -0.45, 0.0]] # position of center of object
        box_dims = [[1, 1] for _ in range(len(box_poses))]  # boxes have the same dimensions
        desk_poses = [[0.225,2.5,np.pi/2],[-0.225,2.5,np.pi/2],[-1.94,2.5,np.pi/2],[-2.63,2.5,np.pi/2],[5.37,-0.45,np.pi/2],[5.37,1.35,np.pi/2],[2.74,2.9,0],[4.57,2.9,0.0]]
        desk_dims = [[1.83, 0.68] for _ in range(len(desk_poses))]  # desks have the same dimensions
        podium_pose = [-2.85,-1.6, 0.0] # center of podium
        podium_dims = [1.6, 0.45]  # podium dimensions

        self.robot_radius = 0.175 # m
        self.add_rectangle_obstacles(
            poses=box_poses + desk_poses + [podium_pose],
            dims=box_dims + desk_dims + [podium_dims],
            robot_radius=self.robot_radius,
        )

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
        self.mowing_sound_thread.start()

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

    def add_rectangle_obstacles(self, poses, dims, robot_radius=0):
        # Create a grid of coordinates corresponding to the occupancy grid
        grid_height, grid_width = self.base_occupancy_grid.shape
        x_coords = np.linspace(self.map_x_min, self.map_x_max, grid_width)
        y_coords = np.linspace(self.map_y_max, self.map_y_min, grid_height)
        x_grid, y_grid = np.meshgrid(x_coords, y_coords)

        self.occupancy_grid = self.base_occupancy_grid.copy()

        # Loop through each rectangle and mark the corresponding grid cells as occupied
        for i, (x, y, theta) in enumerate(poses):
            # Calculate the corners of the rectangle based of center position (x, y) and orientation theta
            half_width = dims[i][0] / 2
            half_height = dims[i][1] / 2
            corners = np.array([
                [-half_width, -half_height],
                [ half_width, -half_height],
                [ half_width,  half_height],
                [-half_width,  half_height]
            ])

            # Rotation matrix for the rectangle
            rotation_matrix = np.array([
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta),  np.cos(theta)]
            ])

            # Rotate and translate corners to the rectangle's position
            rectangle_vertices = (rotation_matrix @ corners.T).T + np.array([-x, -y])

            path = Path(rectangle_vertices)
            contains = path.contains_points(np.vstack((x_grid.flatten(), y_grid.flatten())).T)
            self.occupancy_grid[contains.reshape(x_grid.shape)] = 0.0

        # Inflate obstacles by robot radius
        if robot_radius > 0:
            inflation_radius= int(np.ceil(robot_radius / self.meters_per_pixel))
            inflated_obstacles = ndimage.binary_dilation(self.occupancy_grid == 0.0, iterations=inflation_radius)
            self.occupancy_grid[inflated_obstacles] = 0.0

    def compute_rrt_path(self, goal=[-4.0, 2.0], expand_dis=0.25, max_iter=5000):
        start_pos = [-self.x, -self.y] # start position is where robot is located when RRT is started
        map_params_config = {
            'res': self.meters_per_pixel,
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
            self.p_des = -self.des_path # negate waypoints to match the robot's actual coordinate frame for navigation
            self.numWypts = len(self.p_des)
            print(f"Path found with {self.numWypts} waypoints")
        else: 
            print("No path found.")
            return 
            # self.stop()  # Stop the robot and threads if no path is found

    def waypoint_navigation(self):
        # If no desired path is set, compute a new RRT path to the goal
        if self.p_des is None:
            self.compute_rrt_path(goal=[5.0, 0.0])

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

        self.robot_path.append([-self.x, -self.y]) # store robot path for plotting

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
        self.ax.cla()
        self.ax.imshow(
            self.occupancy_grid,
            extent=[self.map_x_min, self.map_x_max, self.map_y_min, self.map_y_max],
            origin='upper',
            cmap='gray',
            vmin=0.0,
            vmax=1.0,
        )

        # Plot mocap origin.
        self.ax.plot(0.0, 0.0, 'b*', markersize=10)

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
            self.ax.plot(-self.x, -self.y, 'ro', markersize=5)
            self.ax.arrow(-self.x, -self.y, -0.5 * math.cos(self.yaw), -0.5 * math.sin(self.yaw), head_width=0.1, head_length=0.1, fc='r', ec='r')
        
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
        self.mowing_sound_thread.join()
        plt.close('all')

if __name__ == "__main__":
    js = CreateClass()
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