import math
import time
import roslibpy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

class MapClass():
    def __init__(self, source, plot_map=False, id=86, update_delay=1.0, camera_fov_deg=66.0, camera_range=3.0):
        self.plot_map = plot_map
        self.running = True
        self.update_delay = update_delay
        self.last_update_time = None
        self.camera_fov_deg = camera_fov_deg
        self.camera_fov_half_rad = math.radians(camera_fov_deg / 2.0)
        self.camera_range = camera_range
        
        # ROS variables
        self.id = id
        self.client = roslibpy.Ros(host=f'10.24.6.{self.id}', port=9090)
        self.client.run()
        self.x = None; self.y = None; self.z = None
        self.roll = None; self.pitch = None; self.yaw = None
        self.pose_time = None; self.lidar_time = None
        self.pose_history = []

        # Lidar variables
        self.x_min = -2.0; self.x_max = 2.0; self.y_min = -2.0; self.y_max = 2.0
        self.grid_size = 0.1
        n_x = (self.x_max - self.x_min) / self.grid_size
        n_y = (self.y_max - self.y_min) / self.grid_size
        self.occupancy_grid = np.full((int(np.ceil(n_y)), int(np.ceil(n_x))), 0.5)  # Initialize with unknown state (0.5)
        self.x_min_fov = self.x_min; self.x_max_fov = self.x_max; self.y_min_fov = self.y_min; self.y_max_fov = self.y_max
        self.fov_occupancy_grid = np.full((int(np.ceil(n_y)), int(np.ceil(n_x))), 0.5)  # Camera FOV-limited map
        self.x_scan_global = np.array([])
        self.y_scan_global = np.array([])
        self.x_scan_global_fov = np.array([])
        self.y_scan_global_fov = np.array([])

        # Matplotlib figure for plotting and close handler
        self.fig, (self.ax_full, self.ax_fov) = plt.subplots(1, 2, figsize=(12, 6))
        self.ax_full.set_aspect('equal', adjustable='box', anchor='C')
        self.ax_fov.set_aspect('equal', adjustable='box', anchor='C')
        self.fig.canvas.mpl_connect('close_event', self.on_plot_close)

        # Reset Odometry Service
        reset_odom_service = roslibpy.Service(self.client, f'/create_{self.id}/reset_pose', 'irobot_create_msgs/srv/ResetPose')
        reset_odom_request = roslibpy.ServiceRequest({
            'pose': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
        })
        reset_odom_service.call(reset_odom_request)

        # ROS Subscribers
        self.mocap_sub = roslibpy.Topic(self.client, f'/create_{self.id}/pose', 'geometry_msgs/PoseStamped')
        self.odom_sub = roslibpy.Topic(self.client, f'/create_{self.id}/odom', 'nav_msgs/Odometry')
        self.lidar_sub = roslibpy.Topic(self.client, f'/create_{self.id}/scan', 'sensor_msgs/LaserScan')
        if source == 'odom':
            self.odom_sub.subscribe(self.odom_callback)
        elif source == 'mocap':
            self.mocap_sub.subscribe(self.pose_callback)
        else:
            print("Invalid source specified. Use 'odom' or 'mocap'.")
            self.running = False
        self.lidar_sub.subscribe(self.lidar_callback)

        # ROS Publishers
        self.occupancy_grid_pub = roslibpy.Topic(self.client, f'/create_{self.id}/occupancy_grid', 'nav_msgs/OccupancyGrid')
        self.fov_occupancy_grid_pub = roslibpy.Topic(self.client, f'/create_{self.id}/occupancy_grid_fov', 'nav_msgs/OccupancyGrid')

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

        # Store pose history for synchronization with lidar scans
        self.pose_history.append((self.pose_time, self.x, self.y, self.yaw))
        if len(self.pose_history) > 100:
            self.pose_history.pop(0)

    def pose_callback(self, msg):
        self.pose_time = msg['header']['stamp']['sec'] + msg['header']['stamp']['nanosec'] * 1e-9
        # print("pose callback: time={:.2f} sec".format(self.pose_time))

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
        self.yaw = self.wrapToPi(self.yaw).item() # wrap yaw to [-pi, pi]
        # print(f"Pose Update: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad", end="\r", flush=True)

        # Store pose history for synchronization with lidar scans
        self.pose_history.append((self.pose_time, self.x, self.y, self.yaw))
        if len(self.pose_history) > 100:
            self.pose_history.pop(0)
    
    def lidar_callback(self, msg):
        self.lidar_time = msg['header']['stamp']['sec'] + msg['header']['stamp']['nanosec'] * 1e-9
        # print("lidar callback: time={:.2f} sec".format(self.lidar_time))

        ranges_raw = np.array(msg['ranges'])

        # Calculate angles for valid ranges only
        angles_raw = np.linspace(msg['angle_min'], msg['angle_max'], len(msg['ranges']))

        # Filter out invalid ranges (NaN, Inf, out of range)
        valid_ranges = np.isfinite(ranges_raw)
        # valid_ranges &= (ranges_raw >= msg['range_min'])
        valid_ranges &= (ranges_raw >= 0.1) # add small threshold to filter out super close readings
        # valid_ranges &= (ranges_raw <= msg['range_max'])
        valid_ranges &= (ranges_raw <= 20.0) # add upper threshold to filter out spurious long readings
        ranges = ranges_raw[valid_ranges]
        angles = angles_raw[valid_ranges]

        # Find closest pose in history
        if not self.pose_history:
            return
        _, pose_x, pose_y, pose_yaw = min(self.pose_history, key=lambda p: abs(p[0] - self.lidar_time))

        # Transform full-map valid lidar points to Global Frame
        self.R_BG = np.array([[math.cos(pose_yaw), -math.sin(pose_yaw)], [math.sin(pose_yaw), math.cos(pose_yaw)]])
        if ranges.size > 0:
            self.x_scan = ranges * np.cos(angles)
            self.y_scan = ranges * np.sin(angles)
            P_G = self.R_BG @ np.array([self.x_scan, self.y_scan]) + np.array([[pose_x], [pose_y]])
            self.x_scan_global = P_G[0, :]
            self.y_scan_global = P_G[1, :]
        else:
            self.x_scan_global = np.array([])
            self.y_scan_global = np.array([])

        # Camera FOV and range filtering
        camera_fov_mask_raw = np.abs(angles_raw) <= self.camera_fov_half_rad
        camera_angles = angles_raw[camera_fov_mask_raw]
        camera_ranges_raw = ranges_raw[camera_fov_mask_raw]

        # Trace rays out to max camera range for all angles within FOV, then mark valid hits separately
        trace_ranges = np.where(np.isfinite(camera_ranges_raw), camera_ranges_raw, self.camera_range)
        trace_ranges = np.clip(trace_ranges, 0.0, self.camera_range) # ensure trace ranges are within camera range
        x_scan_fov_trace = trace_ranges * np.cos(camera_angles)
        y_scan_fov_trace = trace_ranges * np.sin(camera_angles)
        P_G_fov_trace = self.R_BG @ np.array([x_scan_fov_trace, y_scan_fov_trace]) + np.array([[pose_x], [pose_y]])
        x_scan_global_fov_trace = P_G_fov_trace[0, :]
        y_scan_global_fov_trace = P_G_fov_trace[1, :]

        valid_camera_hits = np.isfinite(camera_ranges_raw)
        valid_camera_hits &= (camera_ranges_raw >= msg['range_min'])
        valid_camera_hits &= (camera_ranges_raw <= msg['range_max'])
        valid_camera_hits &= (camera_ranges_raw <= self.camera_range)
        if np.any(valid_camera_hits):
            x_scan_fov_hits = camera_ranges_raw[valid_camera_hits] * np.cos(camera_angles[valid_camera_hits])
            y_scan_fov_hits = camera_ranges_raw[valid_camera_hits] * np.sin(camera_angles[valid_camera_hits])
            P_G_fov_hits = self.R_BG @ np.array([x_scan_fov_hits, y_scan_fov_hits]) + np.array([[pose_x], [pose_y]])
            self.x_scan_global_fov = P_G_fov_hits[0, :]
            self.y_scan_global_fov = P_G_fov_hits[1, :]
        else:
            self.x_scan_global_fov = np.array([])
            self.y_scan_global_fov = np.array([])

        # Only update the occupancy grid at the specified rate
        if self.update_delay > 0 and self.last_update_time is not None and (self.lidar_time - self.last_update_time) < self.update_delay:
            return
        self.last_update_time = self.lidar_time

        self.occupancy_grid, self.x_min, self.x_max, self.y_min, self.y_max = self.update_occupancy_grid(
            self.occupancy_grid,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.x_scan_global,
            self.y_scan_global,
            pose_x,
            pose_y,
            msg['range_max']
        )

        self.fov_occupancy_grid, self.x_min_fov, self.x_max_fov, self.y_min_fov, self.y_max_fov = self.update_occupancy_grid(
            self.fov_occupancy_grid,
            self.x_min_fov,
            self.x_max_fov,
            self.y_min_fov,
            self.y_max_fov,
            x_scan_global_fov_trace, # use trace points to clear rays out to max camera range
            y_scan_global_fov_trace,
            pose_x,
            pose_y,
            self.camera_range,
            x_occupied_global=self.x_scan_global_fov, # only mark valid hits as occupied in the FOV-limited map
            y_occupied_global=self.y_scan_global_fov
        )

        # Publish both occupancy grids
        self.publish_occupancy_grid(
            self.occupancy_grid_pub,
            self.occupancy_grid,
            self.x_min,
            self.y_min,
            self.lidar_time
        )
        self.publish_occupancy_grid(
            self.fov_occupancy_grid_pub,
            self.fov_occupancy_grid,
            self.x_min_fov,
            self.y_min_fov,
            self.lidar_time
        )

    def update_occupancy_grid(self, grid, x_min, x_max, y_min, y_max, x_scan_global, y_scan_global, pose_x, pose_y, range_max, x_occupied_global=None, y_occupied_global=None):
        # Dynamically expand the grid if points or robot fall outside bounds
        all_x = np.append(x_scan_global, pose_x) if x_scan_global.size > 0 else np.array([pose_x])
        all_y = np.append(y_scan_global, pose_y) if y_scan_global.size > 0 else np.array([pose_y])
        min_x, max_x = np.min(all_x), np.max(all_x)
        min_y, max_y = np.min(all_y), np.max(all_y)

        pad_left = pad_right = pad_bottom = pad_top = 0
        if min_x < x_min:
            pad_left = int(np.ceil((x_min - min_x) / self.grid_size))
            x_min -= pad_left * self.grid_size
        if max_x > x_max:
            pad_right = int(np.ceil((max_x - x_max) / self.grid_size))
            x_max += pad_right * self.grid_size
        if min_y < y_min:
            pad_bottom = int(np.ceil((y_min - min_y) / self.grid_size))
            y_min -= pad_bottom * self.grid_size
        if max_y > y_max:
            pad_top = int(np.ceil((max_y - y_max) / self.grid_size))
            y_max += pad_top * self.grid_size

        if any([pad_left, pad_right, pad_bottom, pad_top]):
            grid = np.pad(
                grid,
                ((pad_bottom, pad_top), (pad_left, pad_right)),
                mode='constant',
                constant_values=0.5
            )

        if x_scan_global.size == 0:
            return grid, x_min, x_max, y_min, y_max

        if x_occupied_global is None or y_occupied_global is None:
            x_occupied_global = x_scan_global
            y_occupied_global = y_scan_global

        # Clear cells traversed by each laser ray
        dx = x_scan_global - pose_x
        dy = y_scan_global - pose_y
        distances = np.sqrt(dx**2 + dy**2)

        # Limit ray tracing to the maximum range to avoid marking cells beyond valid hits as free
        trace_distances = np.minimum(distances, range_max)
        N_list = trace_distances / self.grid_size # number of steps to trace for each ray, based on limited trace distance

        max_steps = int(np.ceil(np.max(N_list))) if N_list.size > 0 else 0
        for step in range(max_steps):
            mask = step < (N_list - 1.0) # only trace up to one step before the occupied cell to avoid clearing it
            if not np.any(mask):
                break

            t = step / N_list[mask] 
            x = pose_x + dx[mask] * t
            y = pose_y + dy[mask] * t

            i = ((x - x_min) / self.grid_size).astype(int)
            j = ((y - y_min) / self.grid_size).astype(int)
            valid_idx = (i >= 0) & (i < grid.shape[1]) & (j >= 0) & (j < grid.shape[0])
            grid[j[valid_idx], i[valid_idx]] = 0.0

        # Mark hit cells as occupied
        if x_occupied_global.size == 0:
            return grid, x_min, x_max, y_min, y_max
        i = ((x_occupied_global - x_min) / self.grid_size).astype(int)
        j = ((y_occupied_global - y_min) / self.grid_size).astype(int)
        valid_idx = (i >= 0) & (i < grid.shape[1]) & (j >= 0) & (j < grid.shape[0])
        grid[j[valid_idx], i[valid_idx]] = 1.0

        return grid, x_min, x_max, y_min, y_max

    def publish_occupancy_grid(self, publisher, grid, x_min, y_min, stamp_time):
        grid_height, grid_width = grid.shape
        occupancy_msg = roslibpy.Message({
            'header': {
                'stamp': {
                    'sec': int(stamp_time),
                    'nanosec': int((stamp_time - int(stamp_time)) * 1e9)
                },
                'frame_id': 'map'
            },
            'info': {
                'width': grid_width,
                'height': grid_height,
                'resolution': self.grid_size,
                'origin': {
                    'position': {'x': x_min, 'y': y_min, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            },
            'data': (grid.flatten() * 100).astype(int).tolist()
        })
        publisher.publish(occupancy_msg)

    def on_plot_close(self, event):
        print("\nPlot window closed. Stopping...")
        self.running = False
        try:
            self.client.terminate()
        except Exception as e:
            print(f"Error during stop: {e}")

    def wrapToPi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def plot_scan(self):
        self.ax_full.clear()
        self.ax_fov.clear()

        self.ax_full.imshow(
            self.occupancy_grid,
            extent=[self.x_min, self.x_max, self.y_min, self.y_max],
            origin='lower',
            cmap='Greys',
            vmin=0,
            vmax=1
        )
        self.ax_fov.imshow(
            self.fov_occupancy_grid,
            extent=[self.x_min_fov, self.x_max_fov, self.y_min_fov, self.y_max_fov],
            origin='lower',
            cmap='Greys',
            vmin=0,
            vmax=1
        )

        if self.x_scan_global.size and self.y_scan_global.size:
            self.ax_full.plot(self.x_scan_global, self.y_scan_global, 'b.', markersize=1)
        if self.x_scan_global_fov.size and self.y_scan_global_fov.size:
            self.ax_fov.plot(self.x_scan_global_fov, self.y_scan_global_fov, 'g.', markersize=1)

        if self.x is not None and self.y is not None:
            for ax in [self.ax_full, self.ax_fov]:
                ax.plot(self.x, self.y, 'ro', markersize=5)
                ax.arrow(
                    self.x,
                    self.y,
                    0.5 * math.cos(self.yaw),
                    0.5 * math.sin(self.yaw),
                    head_width=0.1,
                    head_length=0.1,
                    fc='r',
                    ec='r'
                )

            # Keep robot centered in each view
            window_size = 5.0
            self.ax_full.set_xlim(self.x - window_size, self.x + window_size)
            self.ax_full.set_ylim(self.y - window_size, self.y + window_size)
            self.ax_fov.set_xlim(self.x - window_size, self.x + window_size)
            self.ax_fov.set_ylim(self.y - window_size, self.y + window_size)

        self.ax_full.set_xlabel('X (m)')
        self.ax_full.set_ylabel('Y (m)')
        self.ax_full.set_title('Full Lidar Occupancy Grid')
        self.ax_full.grid()
        self.ax_full.set_aspect('equal', adjustable='box', anchor='C')

        self.ax_fov.set_xlabel('X (m)')
        self.ax_fov.set_ylabel('Y (m)')
        self.ax_fov.set_title(f'Camera FOV Occupancy Grid ({self.camera_fov_deg:.0f} deg)')
        self.ax_fov.grid()
        self.ax_fov.set_aspect('equal', adjustable='box', anchor='C')
        plt.pause(0.01)

if __name__ == "__main__":
    map = MapClass('odom', plot_map=False, id=86, update_delay=1.0, camera_fov_deg=66.0, camera_range=3.0)
    print("Main script running. Press Ctrl+C to stop.")

    try: 
        while map.running:
            if plt.fignum_exists(map.fig.number) and map.plot_map:
                map.plot_scan()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")