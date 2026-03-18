import math
import time
import roslibpy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

class MapClass():
    def __init__(self, id=86):
        self.running = True
        
        # ROS variables
        self.id = id
        self.client = roslibpy.Ros(host=f'10.24.6.{self.id}', port=9090)
        self.client.run()
        self.x = None; self.y = None; self.z = None
        self.roll = None; self.pitch = None; self.yaw = None
        self.pose_time = None; self.lidar_time = None

        # Lidar variables
        self.x_min = -10; self.x_max = 10; self.y_min = -5; self.y_max = 5
        self.grid_size = 0.1
        n_x = (self.x_max - self.x_min) / self.grid_size
        n_y = (self.y_max - self.y_min) / self.grid_size
        self.occupancy_grid = np.zeros((int(n_y), int(n_x)))
        self.x_scan_global = np.array([])
        self.y_scan_global = np.array([])

        # Matplotlib figure for plotting and close handler
        self.fig = plt.figure()
        self.fig.canvas.mpl_connect('close_event', self.on_plot_close)

        # ROS Subscribers
        self.pose_sub = roslibpy.Topic(self.client, f'/create_{self.id}/pose', 'geometry_msgs/PoseStamped')
        self.odom_sub = roslibpy.Topic(self.client, f'/create_{self.id}/odom', 'nav_msgs/Odometry')
        self.lidar_sub = roslibpy.Topic(self.client, f'/create_{self.id}/scan', 'sensor_msgs/LaserScan')
        self.pose_sub.subscribe(self.pose_callback)
        # self.odom_sub.subscribe(self.odom_callback)
        self.lidar_sub.subscribe(self.lidar_callback)

        # ROS Publishers
        self.occupancy_grid_pub = roslibpy.Topic(self.client, f'/create_{self.id}/occupancy_grid', 'nav_msgs/OccupancyGrid')

        # Reset Odometry Service
        reset_odom_service = roslibpy.Service(self.client, f'/create_{self.id}/reset_pose', 'irobot_create_msgs/srv/ResetPose')
        reset_odom_request = roslibpy.ServiceRequest({
            'pose': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
        })
        reset_odom_service.call(reset_odom_request)

    def odom_callback(self, msg):
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
        self.yaw = self.wrapToPi(self.yaw) # wrap yaw to [-pi, pi]
        # print(f"Pose Update: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad", end="\r", flush=True)
    
    def lidar_callback(self, msg):
        self.lidar_time = msg['header']['stamp']['sec'] + msg['header']['stamp']['nanosec'] * 1e-9
        # print("lidar callback: time={:.2f} sec".format(self.lidar_time))
        if self.pose_time is not None and self.lidar_time is not None:
            time_diff = self.lidar_time - self.pose_time
            if abs(time_diff) > 0.05:
                return

        ranges = np.array(msg['ranges'])

        # Calculate angles for valid ranges only
        angles = np.linspace(msg['angle_min'], msg['angle_max'], len(msg['ranges']))

        valid_ranges = np.isfinite(ranges)
        valid_ranges &= (ranges >= msg['range_min'])
        valid_ranges &= (ranges <= msg['range_max'])
        ranges = ranges[valid_ranges]
        angles = angles[valid_ranges]

        if ranges.size == 0:
            return

        # Convert to Cartesian coordinates in Body Frame
        self.x_scan = ranges * np.cos(angles)
        self.y_scan = ranges * np.sin(angles)

        # Check if pose has been initialized
        if self.x is None or self.y is None or self.yaw is None:
            return

        self.R_BG = np.array([[math.cos(self.yaw), -math.sin(self.yaw)], [math.sin(self.yaw), math.cos(self.yaw)]])
        P_G = self.R_BG @ np.array([self.x_scan, self.y_scan]) + np.array([[self.x], [self.y]])

        self.x_scan_global = P_G[0, :]
        self.y_scan_global = P_G[1, :]

        # Clear all cells the laser passes through (ray tracing)
        dx = self.x_scan_global - self.x
        dy = self.y_scan_global - self.y
        distances = np.sqrt(dx**2 + dy**2)
        N_list = distances / self.grid_size
        max_steps = int(np.ceil(np.max(N_list))) if N_list.size > 0 else 0
        
        for step in range(max_steps):
            # Only step through a ray strictly up to the obstacle hits
            mask = step < (N_list - 1.0) 
            if not np.any(mask): break
            
            t = step / N_list[mask]
            x = self.x + dx[mask] * t
            y = self.y + dy[mask] * t
            
            i = ((x - self.x_min) / self.grid_size).astype(int)
            j = ((y - self.y_min) / self.grid_size).astype(int)
            
            valid_idx = (i >= 0) & (i < self.occupancy_grid.shape[1]) & (j >= 0) & (j < self.occupancy_grid.shape[0])
            self.occupancy_grid[j[valid_idx], i[valid_idx]] = 0.0
            # self.occupancy_grid[j[valid_idx], i[valid_idx]] -= 0.1
            # self.occupancy_grid = np.clip(self.occupancy_grid, 0.0, 1.0)

        # Mark the cells corresponding to the laser scan points as occupied
        i = ((self.x_scan_global - self.x_min) / self.grid_size).astype(int)
        j = ((self.y_scan_global - self.y_min) / self.grid_size).astype(int)
        valid_idx = (i >= 0) & (i < self.occupancy_grid.shape[1]) & (j >= 0) & (j < self.occupancy_grid.shape[0])
        self.occupancy_grid[j[valid_idx], i[valid_idx]] = 1.0
        # self.occupancy_grid[j[valid_idx], i[valid_idx]] += 0.3
        # self.occupancy_grid = np.clip(self.occupancy_grid, 0.0, 1.0)

        grid_height, grid_width = self.occupancy_grid.shape
        occupancy_msg = roslibpy.Message({
            'header': {
                'stamp': roslibpy.Time.now(),
                'frame_id': 'map'
            },
            'info': {
                'width': grid_width,
                'height': grid_height,
                'resolution': self.grid_size,
            },
            'data': self.occupancy_grid.flatten().tolist()
        })
        self.occupancy_grid_pub.publish(occupancy_msg)

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
        plt.clf()
        grid_height, grid_width = self.occupancy_grid.shape
        x_half = (grid_width * self.grid_size) / 2
        y_half = (grid_height * self.grid_size) / 2
        plt.imshow(self.occupancy_grid, extent=[-x_half, x_half, -y_half, y_half], origin='lower', cmap='Greys')
        if self.x_scan_global.size and self.y_scan_global.size:
            plt.plot(self.x_scan_global, self.y_scan_global, 'b.', markersize=1)
        if self.x is not None and self.y is not None:
            plt.plot(self.x, self.y, 'ro', markersize=5)  # Robot position
            plt.arrow(self.x, self.y, 0.5 * math.cos(self.yaw), 0.5 * math.sin(self.yaw), head_width=0.1, head_length=0.1, fc='r', ec='r')  # Robot heading
        plt.xlim(-x_half, x_half)
        plt.ylim(-y_half, y_half)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Occupancy Grid')
        plt.grid()
        plt.pause(0.01)

if __name__ == "__main__":
    map = MapClass()
    print("Main script running. Press Ctrl+C to stop.")

    try: 
        while map.running:
            if plt.fignum_exists(map.fig.number):
                map.plot_scan()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")