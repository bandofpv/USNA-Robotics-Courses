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
    def __init__(self, id=81):
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
        self.attempted_goal = None # latest attempted RRT goal for debug plotting
        self.numWypts = 0 # number of waypoints in path
        
        # Initialize timing variables for performance analysis
        self.rrt_start_time = None
        self.rrt_end_time = None
        self.pose_data = []
        self.des_pose_data = []
        self.wypt_error = []
        self.yaw_error = []

        # Occupancy grid map variables
        self.occupancy_grid = None
        self.grid_size = None
        self.width = None; self.height = None
        self.map_x_min = None; self.map_x_max = None; self.map_y_min = None; self.map_y_max = None
        self.fov_occupancy_grid = None
        self.fov_grid_size = None
        self.fov_width = None; self.fov_height = None
        self.fov_x_min = None; self.fov_x_max = None; self.fov_y_min = None; self.fov_y_max = None
        self.camera_range = 3.0 # m expected camera sensing range for frontier targeting
        self.frontier_range_tolerance = 1.0 # m allowed deviation from camera range for preferred frontiers
        self.frontier_wall_clearance = 0.30 # m minimum clearance from occupied/unknown cells for frontier candidates
        self.robot_radius = 0.22 # m
        self.plot_window_size = 5.0 # m half-width for fixed-size plot window centered on robot
        self.expand_dis = 0.60 # m default expansion distance for RRT tree
        self.default_expand_dis = self.expand_dis # reset value after successful plan
        self.min_expand_dis = 0.10 # m minimum expansion distance for adaptive replanning
        self.expand_dis_decay = 0.75 # multiplicative decay when replanning fails
        self.rrt_iter_scale_cap = 4.0 # max multiplier for adaptive RRT iterations
        self.map_update_count = 0 # increments on each occupancy-grid update
        self.last_replan_map_update = -1 # map update count used by the last replan attempt
        self.escape_mode = False # true when executing a short path to exit occupied space
        self.escape_min_distance = 0.40 # m preferred minimum distance from occupied start when escaping
        self.initial_scan_done = False # run one full 360 spin before first exploration plan
        self.initial_scan_prev_yaw = None
        self.initial_scan_accum_yaw = 0.0
        self.initial_scan_rate = 0.35 # rad/s in-place scan speed

        # Matplotlib figure/axes for plotting and close handler
        self.fig, (self.ax_full, self.ax_fov) = plt.subplots(1, 2, figsize=(13, 6))
        self.ax_full.set_aspect('equal', adjustable='box', anchor='C')
        self.ax_fov.set_aspect('equal', adjustable='box', anchor='C')
        self.fig.canvas.mpl_connect('close_event', self.on_plot_close)

        # Define Controller Gains and algorithm constants
        self.wp_num = 0 # waypoint index
        self.wp_rad = 0.15 # waypoint radius
        self.Kp_yaw = 0.75 # heading gain
        self.Kp_speed = 0.5 # forward speed gain
        self.yaw_thresh = math.radians(5) # yaw error threshold for stopping in degrees

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
        self.odom_sub = roslibpy.Topic(self.client, f'/create_{self.id}/odom', 'nav_msgs/Odometry')
        self.map_og_sub = roslibpy.Topic(self.client, f'/create_{self.id}/occupancy_grid', 'nav_msgs/OccupancyGrid')
        self.map_og_fov_sub = roslibpy.Topic(self.client, f'/create_{self.id}/occupancy_grid_fov', 'nav_msgs/OccupancyGrid')
        self.dock_sub.subscribe(self.dock_callback)
        self.hazard_sub.subscribe(self.hazard_callback)
        self.odom_sub.subscribe(self.odom_callback)
        self.map_og_sub.subscribe(self.map_occupancy_callback)
        self.map_og_fov_sub.subscribe(self.map_fov_occupancy_callback)

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

    def odom_callback(self, msg):
        self.pose_time = msg['header']['stamp']['sec'] + msg['header']['stamp']['nanosec'] * 1e-9 

        self.x = msg['pose']['pose']['position']['x']
        self.y = msg['pose']['pose']['position']['y']
        self.z = msg['pose']['pose']['position']['z']
        r = R.from_quat([
            msg['pose']['pose']['orientation']['x'],
            msg['pose']['pose']['orientation']['y'],
            msg['pose']['pose']['orientation']['z'],
            msg['pose']['pose']['orientation']['w']
        ])
        self.roll, self.pitch, self.yaw = r.as_euler('xyz', degrees=False)
        # print(f"Pose Update: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad", end="\r", flush=True)

    def map_occupancy_callback(self, msg):
        self.map_update_count += 1
        self.width = msg['info']['width']
        self.height = msg['info']['height']
        self.grid_size = msg['info']['resolution']
        self.map_x_min = msg['info']['origin']['position']['x']
        self.map_y_min = msg['info']['origin']['position']['y']
        self.map_x_max = self.map_x_min + self.width * self.grid_size
        self.map_y_max = self.map_y_min + self.height * self.grid_size

        # Convert flattened occupancy data to [1.0 free, 0.5 unknown, 0.0 occupied]
        grid_data = np.array(msg['data']).reshape((self.height, self.width))
        self.occupancy_grid = 1.0 - (grid_data / 100.0)

        # Inflate obstacles in occupancy grid by robot radius to account for robot size in planning
        if self.robot_radius > 0:
            inflation_radius = int(np.ceil(self.robot_radius / self.grid_size))
            inflated_obstacles = ndimage.binary_dilation(self.occupancy_grid <= 0.0, iterations=inflation_radius)
            self.occupancy_grid[inflated_obstacles] = 0.0

    def map_fov_occupancy_callback(self, msg):
        self.fov_width = msg['info']['width']
        self.fov_height = msg['info']['height']
        self.fov_grid_size = msg['info']['resolution']
        self.fov_x_min = msg['info']['origin']['position']['x']
        self.fov_y_min = msg['info']['origin']['position']['y']
        self.fov_x_max = self.fov_x_min + self.fov_width * self.fov_grid_size
        self.fov_y_max = self.fov_y_min + self.fov_height * self.fov_grid_size

        grid_data = np.array(msg['data']).reshape((self.fov_height, self.fov_width))
        self.fov_occupancy_grid = 1.0 - (grid_data / 100.0)

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

    def grid_idx_to_world(self, x_idx, y_idx, x_min, y_min, res):
        x = x_idx * res + x_min
        y = y_idx * res + y_min
        return np.array([x, y])

    def world_to_grid_idx(self, x, y, x_min, y_min, res):
        x_idx = int(np.floor((x - x_min) / res))
        y_idx = int(np.floor((y - y_min) / res))
        return x_idx, y_idx

    def get_nearest_free_start(self, start_world):
        if self.occupancy_grid is None or self.grid_size is None:
            return None, False

        free_mask = self.occupancy_grid > 0.5
        if not np.any(free_mask):
            return None, False

        sx, sy = start_world
        sx_idx, sy_idx = self.world_to_grid_idx(sx, sy, self.map_x_min, self.map_y_min, self.grid_size)
        in_bounds = (0 <= sx_idx < self.occupancy_grid.shape[1]) and (0 <= sy_idx < self.occupancy_grid.shape[0])

        if in_bounds and free_mask[sy_idx, sx_idx]:
            return np.array([sx, sy]), False

        free_cells = np.argwhere(free_mask)  # [y_idx, x_idx]
        query_idx = np.array([
            (sy - self.map_y_min) / self.grid_size,
            (sx - self.map_x_min) / self.grid_size
        ])
        deltas = free_cells - query_idx
        d2 = np.sum(deltas * deltas, axis=1)

        # Prefer escape targets with a minimum standoff distance from the occupied start.
        min_escape_cells = self.escape_min_distance / self.grid_size
        far_enough_mask = d2 >= (min_escape_cells ** 2)

        if np.any(far_enough_mask):
            candidate_indices = np.where(far_enough_mask)[0]
            nearest_candidate_idx = candidate_indices[int(np.argmin(d2[candidate_indices]))]
            nearest_y_idx, nearest_x_idx = free_cells[nearest_candidate_idx]
        else:
            # Fallback: if map is too constrained, use closest free cell.
            nearest_idx = int(np.argmin(d2))
            nearest_y_idx, nearest_x_idx = free_cells[nearest_idx]

        safe_start_world = self.grid_idx_to_world(nearest_x_idx, nearest_y_idx, self.map_x_min, self.map_y_min, self.grid_size)
        return safe_start_world, True

    def is_pose_in_free_cell(self, x, y):
        if self.occupancy_grid is None or self.grid_size is None:
            return False
        x_idx, y_idx = self.world_to_grid_idx(x, y, self.map_x_min, self.map_y_min, self.grid_size)
        if x_idx < 0 or x_idx >= self.occupancy_grid.shape[1] or y_idx < 0 or y_idx >= self.occupancy_grid.shape[0]:
            return False
        return self.occupancy_grid[y_idx, x_idx] > 0.5

    def path_collision_free_in_full_map(self, path):
        if path is None or len(path) < 2 or self.occupancy_grid is None:
            return False

        # Reuse RRT segment checker with the full occupancy map to verify every segment.
        map_params_config = {
            'res': self.grid_size,
            'x_limit': [self.map_x_min, self.map_x_max],
            'y_limit': [self.map_y_min, self.map_y_max],
            'origin': [self.map_x_min, self.map_y_min]
        }
        checker = RRT(
            start=path[0],
            goal=path[-1],
            map_grid=self.occupancy_grid,
            map_params=map_params_config,
            expand_dis=self.expand_dis,
            max_iter=1
        )

        for idx in range(len(path) - 1):
            start_node = np.array(path[idx])
            end_node = np.array(path[idx + 1])
            if not checker._is_segment_collision_free(start_node, end_node):
                return False
        return True

    def select_frontier_goal(self):
        if self.fov_occupancy_grid is None or self.fov_grid_size is None:
            return None
        if self.occupancy_grid is None or self.grid_size is None:
            return None
        if self.x is None or self.y is None:
            return None

        # Snapshot map data to avoid shape races while ROS callbacks update grids.
        fov_grid = np.array(self.fov_occupancy_grid, copy=True)
        full_grid = np.array(self.occupancy_grid, copy=True)
        fov_x_min = self.fov_x_min
        fov_y_min = self.fov_y_min
        fov_grid_size = self.fov_grid_size
        map_x_min = self.map_x_min
        map_y_min = self.map_y_min
        grid_size = self.grid_size

        # Build a wall-proximity mask in full map so we can reject frontiers hugging walls.
        occupied_or_unknown = full_grid <= 0.5
        clearance_cells = int(np.ceil(self.frontier_wall_clearance / grid_size))
        if clearance_cells > 0:
            near_wall_mask = ndimage.binary_dilation(occupied_or_unknown, iterations=clearance_cells)
        else:
            near_wall_mask = occupied_or_unknown

        eps = 1e-3
        fov_unknown = np.isclose(fov_grid, 0.5, atol=eps)
        fov_free = fov_grid > 0.5 + eps

        # Frontier candidates are free cells adjacent to unknown cells.
        unknown_neighbors = ndimage.binary_dilation(fov_unknown, structure=np.ones((3, 3), dtype=bool))
        if unknown_neighbors.shape != fov_free.shape:
            min_rows = min(unknown_neighbors.shape[0], fov_free.shape[0])
            min_cols = min(unknown_neighbors.shape[1], fov_free.shape[1])
            unknown_neighbors = unknown_neighbors[:min_rows, :min_cols]
            fov_free = fov_free[:min_rows, :min_cols]
        frontier_free = fov_free & unknown_neighbors
        frontier_idx = np.argwhere(frontier_free)
        if frontier_idx.size == 0:
            return None

        robot_xy = np.array([self.x, self.y])
        candidate_world = []
        candidate_dist = []

        for y_idx, x_idx in frontier_idx:
            p_world = self.grid_idx_to_world(x_idx, y_idx, fov_x_min, fov_y_min, fov_grid_size)

            # Validate candidate on full map: in bounds and free.
            full_x_idx = int((p_world[0] - map_x_min) / grid_size)
            full_y_idx = int((p_world[1] - map_y_min) / grid_size)
            if full_x_idx < 0 or full_x_idx >= full_grid.shape[1]:
                continue
            if full_y_idx < 0 or full_y_idx >= full_grid.shape[0]:
                continue
            if full_grid[full_y_idx, full_x_idx] <= 0.5:
                continue
            if near_wall_mask[full_y_idx, full_x_idx]:
                continue

            candidate_world.append(p_world)
            candidate_dist.append(np.linalg.norm(p_world - robot_xy))

        if not candidate_world:
            return None

        candidate_dist = np.array(candidate_dist)

        # Prefer frontiers near camera range to push exploration outward from the robot.
        near_range_mask = np.abs(candidate_dist - self.camera_range) <= self.frontier_range_tolerance
        if np.any(near_range_mask):
            masked_indices = np.where(near_range_mask)[0]
            best_local = int(np.argmin(np.abs(candidate_dist[masked_indices] - self.camera_range)))
            best_idx = int(masked_indices[best_local])
        else:
            # Fallback: choose the frontier closest to the target camera range.
            best_idx = int(np.argmin(np.abs(candidate_dist - self.camera_range)))

        return candidate_world[best_idx].tolist()

    def compute_rrt_path(self, goal,  max_iter=10000):
        if self.occupancy_grid is None or self.grid_size is None:
            print("Full occupancy grid not available yet.")
            return False

        if goal is None:
            goal = self.select_frontier_goal()
            if goal is None:
                self.attempted_goal = None
                print("No reachable frontier found in camera FOV map.")
                return False

        self.attempted_goal = np.array(goal)

        start_pos = np.array([self.x, self.y]) # start position is where robot is located when RRT is started
        safe_start_pos, start_was_adjusted = self.get_nearest_free_start(start_pos)
        if safe_start_pos is None:
            print("No free cell available for a valid RRT start position.")
            self.des_path = None
            self.p_des = None
            self.numWypts = 0
            return False
        if start_was_adjusted:
            # Enter escape mode: only move out of occupied space first, then replan frontier.
            self.escape_mode = True
            self.des_path = np.array([start_pos, safe_start_pos])
            self.p_des = self.des_path
            self.numWypts = len(self.p_des)
            self.wp_num = 0
            self.rrt_tree_nodes = None
            print(f"Start pose occupied/out of bounds. Escaping to nearest free cell at x={safe_start_pos[0]:.2f}, y={safe_start_pos[1]:.2f}.")
            return True
        self.escape_mode = False
        start_pos = safe_start_pos.tolist()

        map_params_config = {
            'res': self.grid_size,
            'x_limit': [self.map_x_min, self.map_x_max],
            'y_limit': [self.map_y_min, self.map_y_max],
            'origin': [self.map_x_min, self.map_y_min]
        }
        # Record that this map update has consumed one replan attempt.
        self.last_replan_map_update = self.map_update_count

        # Run one attempt per map update to avoid blocking/replanning loops.
        expand_try = max(self.expand_dis, self.min_expand_dis)

        # Use more iterations for smaller expansion distances.
        iter_scale = self.default_expand_dis / max(expand_try, 1e-6)
        iter_scale = min(max(iter_scale, 1.0), self.rrt_iter_scale_cap)
        iter_try = int(max_iter * iter_scale)

        planner = RRT(start=start_pos,
                      goal=goal,
                      map_grid=self.occupancy_grid,
                      map_params=map_params_config,
                      expand_dis=expand_try,
                  max_iter=iter_try)

        path_result = planner.plan()
        self.rrt_tree_nodes = np.array(planner.tree)

        if path_result is not None and self.path_collision_free_in_full_map(path_result):
            self.des_path = np.array(path_result)
            self.p_des = self.des_path # match the robot's actual coordinate frame for navigation
            self.numWypts = len(self.p_des)
            self.expand_dis = self.default_expand_dis
            print(f"Path found with {self.numWypts} waypoints (expand_dis={expand_try:.2f}, max_iter={iter_try})")
            return True

        # Planning failed for this map update; lower expand distance for the next attempt.
        self.des_path = None
        self.p_des = None
        self.numWypts = 0
        next_expand = max(self.min_expand_dis, expand_try * self.expand_dis_decay)
        self.expand_dis = next_expand
        print(f"No valid path at expand_dis={expand_try:.2f}, max_iter={iter_try}. Next attempt will use expand_dis={self.expand_dis:.2f}.")
        return False

    def run_initial_exploration_scan(self):
        if self.yaw is None:
            self.control_movement(0.0, 0.0)
            return False

        if self.initial_scan_prev_yaw is None:
            self.initial_scan_prev_yaw = self.yaw
            self.initial_scan_accum_yaw = 0.0

        delta_yaw = self.wrapToPi(self.yaw - self.initial_scan_prev_yaw)
        self.initial_scan_accum_yaw += abs(delta_yaw)
        self.initial_scan_prev_yaw = self.yaw

        if self.initial_scan_accum_yaw >= (2.0 * math.pi):
            self.control_movement(0.0, 0.0)
            self.initial_scan_done = True
            self.initial_scan_prev_yaw = None
            self.initial_scan_accum_yaw = 0.0
            self.last_replan_map_update = -1
            print("Initial 360 scan complete. Starting frontier exploration.")
            return True

        self.control_movement(0.0, self.initial_scan_rate)
        return False

    def waypoint_navigation(self):
        # Perform one-time 360 scan before first exploration planning.
        if not self.initial_scan_done:
            self.run_initial_exploration_scan()
            return

        # If escaping occupied space, stop as soon as we are free and trigger fresh frontier planning.
        if self.escape_mode and self.x is not None and self.y is not None and self.is_pose_in_free_cell(self.x, self.y):
            self.control_movement(0.0, 0.0)
            self.escape_mode = False
            self.p_des = None
            self.des_path = None
            self.rrt_tree_nodes = None
            self.numWypts = 0
            self.wp_num = 0
            self.last_replan_map_update = -1
            print("Robot exited occupied region. Replanning frontier exploration.")

        # If no desired path is set, compute a new RRT path to the goal
        if self.p_des is None:
            # Replan only when fresh map data is available to avoid repetitive retries.
            if self.last_replan_map_update != self.map_update_count:
                self.compute_rrt_path(goal=None)
            if self.p_des is None:
                self.control_movement(0.0, 0.0)
                return

        # If all waypoints are reached, start a new frontier exploration cycle.
        if self.wp_num >= self.numWypts:
            u_des = 0; r_des = 0
            self.control_movement(u_des, r_des)
            self.robot_path = []
            self.p_des = None
            self.des_path = None
            self.rrt_tree_nodes = None
            self.wp_num = 0 # reset waypoints for next time
            # Force the next loop to request a fresh frontier plan.
            self.last_replan_map_update = -1
            print("All waypoints reached. Replanning new frontier exploration.")
            return

        # If the robot is bumped, stop and immediately replan a new path.
        if self.is_bumped:
            u_des = 0; r_des = 0
            self.control_movement(u_des, r_des)
            self.robot_path = []
            self.p_des = None
            self.des_path = None
            self.rrt_tree_nodes = None
            self.wp_num = 0 # reset waypoints so next path starts from waypoint 0
            self.is_bumped = False
            # Force immediate replan after bump even if map hasn't changed yet.
            self.last_replan_map_update = -1
            self.compute_rrt_path(goal=None)
            if self.p_des is None:
                print("Bump detected. Replanning...")
                return
            print("Bump detected. New path planned.")
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
        else:
            self.yaw_error.append(yaw_error)

        # Iterate waypoints after reaching waypoint radius
        if dist2wp < self.wp_rad:
            self.wypt_error.append(dist2wp)
            self.wp_num += 1
        
        # Bound forward speed and turn rate
        u = max(0,min(u_des,0.306))
        r = max(-0.25,min(r_des,0.25))

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

        self.ax_full.clear()
        self.ax_fov.clear()

        self.ax_full.imshow(
            self.occupancy_grid,
            extent=[self.map_x_min, self.map_x_max, self.map_y_min, self.map_y_max],
            origin='lower',
            cmap='gray',
            vmin=0.0,
            vmax=1.0
        )

        if self.fov_occupancy_grid is not None and self.fov_x_min is not None:
            self.ax_fov.imshow(
                self.fov_occupancy_grid,
                extent=[self.fov_x_min, self.fov_x_max, self.fov_y_min, self.fov_y_max],
                origin='lower',
                cmap='gray',
                vmin=0.0,
                vmax=1.0
            )

        # Plot robot position if available
        if self.x is not None and self.y is not None:
            for ax in [self.ax_full, self.ax_fov]:
                ax.plot(self.x, self.y, 'ro', markersize=5)
                ax.arrow(self.x, self.y, 0.5 * math.cos(self.yaw), 0.5 * math.sin(self.yaw), head_width=0.1, head_length=0.1, fc='r', ec='r')

        # Plot RRT tree nodes and path on both plots for direct comparison.
        if self.rrt_tree_nodes is not None and len(self.rrt_tree_nodes) > 0:
            self.ax_full.plot(self.rrt_tree_nodes[:, 0], self.rrt_tree_nodes[:, 1], 'c.', markersize=2)
            self.ax_fov.plot(self.rrt_tree_nodes[:, 0], self.rrt_tree_nodes[:, 1], 'c.', markersize=2)

        # Plot the latest attempted RRT goal for debugging.
        if self.attempted_goal is not None and len(self.attempted_goal) == 2:
            self.ax_full.plot(self.attempted_goal[0], self.attempted_goal[1], 'mx', markersize=10, markeredgewidth=2)
            self.ax_fov.plot(self.attempted_goal[0], self.attempted_goal[1], 'mx', markersize=10, markeredgewidth=2)

        if self.des_path is not None and len(self.des_path) > 0:
            goal_point = self.des_path[-1]
            self.ax_full.plot(goal_point[0], goal_point[1], 'g*', markersize=15)
            self.ax_fov.plot(goal_point[0], goal_point[1], 'g*', markersize=15)
            self.ax_full.plot(self.des_path[:, 0], self.des_path[:, 1], 'g-', linewidth=2)
            self.ax_fov.plot(self.des_path[:, 0], self.des_path[:, 1], 'g-', linewidth=2)
            if self.wp_num < len(self.des_path):
                self.ax_full.plot(self.des_path[self.wp_num, 0], self.des_path[self.wp_num, 1], 'y.', markersize=10)
                self.ax_fov.plot(self.des_path[self.wp_num, 0], self.des_path[self.wp_num, 1], 'y.', markersize=10)

        # Plot robot path if available
        if len(self.robot_path) > 0:
            robot_path_array = np.array(self.robot_path)
            self.ax_full.plot(robot_path_array[:, 0], robot_path_array[:, 1], 'r-', linewidth=1)
            self.ax_fov.plot(robot_path_array[:, 0], robot_path_array[:, 1], 'r-', linewidth=1)

        # Keep a fixed-size view window centered on the robot (same behavior as mapping.py).
        if self.x is not None and self.y is not None:
            self.ax_full.set_xlim(self.x - self.plot_window_size, self.x + self.plot_window_size)
            self.ax_full.set_ylim(self.y - self.plot_window_size, self.y + self.plot_window_size)
            self.ax_fov.set_xlim(self.x - self.plot_window_size, self.x + self.plot_window_size)
            self.ax_fov.set_ylim(self.y - self.plot_window_size, self.y + self.plot_window_size)
        else:
            self.ax_full.set_xlim(self.map_x_min, self.map_x_max)
            self.ax_full.set_ylim(self.map_y_min, self.map_y_max)
            if self.fov_x_min is not None:
                self.ax_fov.set_xlim(self.fov_x_min, self.fov_x_max)
                self.ax_fov.set_ylim(self.fov_y_min, self.fov_y_max)

        self.ax_full.set_aspect('equal', adjustable='box', anchor='C')
        self.ax_full.set_xlabel('X (m)')
        self.ax_full.set_ylabel('Y (m)')
        self.ax_full.set_title('Full Lidar Occupancy Grid')
        self.ax_full.grid()

        self.ax_fov.set_aspect('equal', adjustable='box', anchor='C')
        self.ax_fov.set_xlabel('X (m)')
        self.ax_fov.set_ylabel('Y (m)')
        self.ax_fov.set_title('Camera FOV Occupancy Grid')
        self.ax_fov.grid()
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

    def save_results(self):
        # Save pose and desired pose data to CSV files for analysis
        if len(self.pose_data) > 0 and len(self.des_pose_data) > 0:
            np.savetxt("12_Weeks/pose_data.csv", self.pose_data, delimiter=",", header="x,y,yaw", comments="")
            np.savetxt("12_Weeks/des_pose_data.csv", self.des_pose_data, delimiter=",", header="x_des,y_des,psi_des", comments="")
            print("Pose data saved to pose_data.csv and des_pose_data.csv")

        # Save occupancy grid and paths for visualization
        if self.occupancy_grid is not None:
            np.save("12_Weeks/occupancy_grid.npy", self.occupancy_grid)
            occupancy_grid_info = {
                'grid_size': self.grid_size,
                'width': self.width,
                'height': self.height,
                'map_x_min': self.map_x_min,
                'map_x_max': self.map_x_max,
                'map_y_min': self.map_y_min,
                'map_y_max': self.map_y_max
            }
            np.save("12_Weeks/occupancy_grid_info.npy", occupancy_grid_info)

            print("Occupancy grid and info saved to occupancy_grid.npy and occupancy_grid_info.npy")

        print(f"Path execution time: {self.rrt_end_time - self.rrt_start_time:.2f} seconds" if self.rrt_start_time is not None and self.rrt_end_time is not None else "RRT execution time not available") 
        print(f"Final robot position: x={self.x:.2f} m, y={self.y:.2f} m" if self.x is not None and self.y is not None else "Final robot position not available")
        print(f"Average waypoint error: {np.mean(self.wypt_error):.2f} m" if len(self.wypt_error) > 0 else "Waypoint error not available")
        print(f"Average yaw error: {math.degrees(np.mean(self.yaw_error)):.2f} degrees" if len(self.yaw_error) > 0 else "Yaw error not available")

if __name__ == "__main__":
    js = CreateClass(id=86)
    print("Main script running. Press Ctrl+C to stop.")

    try:
        while js.running:
            if js.joystick:
                # Formatting for cleaner console output
                axes_str = [f"{val:5.2f}" for val in js.axes]
                status_str = "ARMED" if js.armed else "DISARMED"
                mode_str = "MANUAL" if js.is_manual_mode else "AUTO"
                # print(f"\r[{status_str} | {mode_str}] Axes: {axes_str} | Buttons: {js.buttons}", end="", flush=True)
            else:
                print("\rWaiting for joystick connection...", end="", flush=True)

            js.plot_occupancy_grid()

            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        js.stop()
        js.save_results()