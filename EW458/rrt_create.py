import time
import math
import numpy as np
from RRT import RRT
import matplotlib.pyplot as plt
from create3_sim import CreateSim

def wrapToPi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

create = CreateSim(name='create')

occupancy_grid = np.zeros((100, 100))
grid_size = 0.1  # Size of each grid cell in meters
# Grid definition: 
# Center (index 50,50) is (0,0) meters.
# Extent is -5 to 5 meters.
# origin (index 0,0) is at (-5, -5) meters.

# Define Map Parameters for RRT
map_params_config = {
    'res': grid_size,
    'x_limit': [-5, 5],
    'y_limit': [-5, 5],
    'origin': [-5, -5] 
}

# Set Manual Goal
GOAL_POINT = [-4.0, -4.0] 

# Simulation control flag
sim_running = True

def on_close(event):
    global sim_running
    print("Window closed. Exiting simulation...")
    sim_running = False

# Path Following State
wp_index = 0
wp_radius = 0.2
active_path = None

if __name__ == '__main__':
    create.start()
    time.sleep(1.0)
    
    # Initialize plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(8,8))
    
    # Connect the close event
    fig.canvas.mpl_connect('close_event', on_close)

    t_start = create.get_sim_time()
    create.update()
    
    # Controller Gains
    Kp_yaw = 0.5
    Kp_speed = 2.0
    
    # Run simulation loop
    while sim_running: 
        t = create.get_sim_time()

        # Update Grid from Lidar
        scan_data = create.get_scan_data()
        
        if scan_data is not None:
            n = len(scan_data)
            scan_data = np.array(scan_data)
            
            angle_min = -120.0*math.pi/180.0
            angle_max = 120.0*math.pi/180.0
            angles = np.linspace(angle_min, angle_max, n)

            ind = scan_data < 5.0
            scan_data = scan_data[ind]
            x_B = scan_data * np.cos(angles[ind])
            y_B = scan_data * np.sin(angles[ind])

            R_BG = np.array([[np.cos(create.pose[2]), -np.sin(create.pose[2])],
                            [np.sin(create.pose[2]),  np.cos(create.pose[2])]])
            
            P_G = R_BG @ np.array([x_B, y_B]) + np.array([[create.pose[0]], [create.pose[1]]])

            # Fill Occupancy Grid
            for i in range(P_G.shape[1]):
                x_idx = int((P_G[0, i] + 5.0) / grid_size)
                y_idx = int((P_G[1, i] + 5.0) / grid_size)
                if 0 <= x_idx < occupancy_grid.shape[1] and 0 <= y_idx < occupancy_grid.shape[0]:
                    occupancy_grid[y_idx, x_idx] = 1.0

        # Run RRT Planning periodically
        # Logic: If we don't have a path, plan one. If we have one, just follow it. 
        # You can add logic to re-plan if the path becomes blocked.
        rrt_tree_nodes = []
        
        # Check if current path is blocked by new obstacles
        if active_path is not None:
            # Create a temporary planner just for collision checking utils
            # (We use current pose as start, though strictly we only need map params)
            temp_planner = RRT(start=[0,0], goal=[0,0], map_grid=occupancy_grid, map_params=map_params_config)
            
            # Check segments from current position to next few waypoints
            # precise checking: check segment from robot to current target, and subsequent segments
            path_valid = True
            
            # Check robot to current target
            if not temp_planner._is_segment_collision_free(np.array([create.pose[0], create.pose[1]]), active_path[wp_index]):
                path_valid = False
            else:
                # Check remaining path segments
                for i in range(wp_index, len(active_path)-1):
                    if not temp_planner._is_segment_collision_free(active_path[i], active_path[i+1]):
                        path_valid = False
                        break
            
            if not path_valid:
                print("Obstacle detected on path! Replanning...")
                active_path = None
                create.set_cmd_vel(0,0)

        if active_path is None:
             # Stop while planning
            create.set_cmd_vel(0,0)
            
            # Plan only if figure is open and enough time has passed to accumulate map data
            if plt.fignum_exists(fig.number) and (t - t_start > 1.0):
                start_pos = [create.pose[0], create.pose[1]]
                print("Planning path...")
                planner = RRT(start=start_pos, 
                              goal=GOAL_POINT, 
                              map_grid=occupancy_grid, 
                              map_params=map_params_config, 
                              expand_dis=0.5, 
                              max_iter=500) 
                
                path_result = planner.plan()
                rrt_tree_nodes = np.array(planner.tree)
                
                if path_result is not None:
                    active_path = np.array(path_result)
                    wp_index = 1 # Start at index 1 because 0 is current position
                    print(f"Path found with {len(active_path)} waypoints")
        
        # --- PATH FOLLOWING CONTROLLER ---
        u_cmd = 0.0
        r_cmd = 0.0
        
        if active_path is not None:
            if wp_index < len(active_path):
                # Get current target waypoint
                current_target = active_path[wp_index]
                
                # Calculate Error
                dx = current_target[0] - create.pose[0]
                dy = current_target[1] - create.pose[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                # Check if reached waypoint
                if dist < wp_radius:
                    wp_index += 1
                else:
                    # Compute Control Inputs
                    psi_des = math.atan2(dy, dx)
                    psi_err = wrapToPi(psi_des - create.pose[2])
                    
                    # Simple P-Controller
                    r_cmd = Kp_yaw * psi_err
                    
                    # Slow down if heading error is large
                    if abs(psi_err) > math.radians(20):
                        u_cmd = 0.0
                    else:
                        u_cmd = min(0.3, Kp_speed * dist) # Cap speed at 0.3 m/s
                        
            else:
                # Reached Goal
                print("Goal Reached!")
                u_cmd = 0.0
                r_cmd = 0.0
                # Optionally reset to find new goal or stop
                # active_path = None 
        else:
            # No path logic (spin to scan)
            u_cmd = 0.0
            r_cmd = 0.4
            
        create.set_cmd_vel(u_cmd, r_cmd)
        create.update()

        # Visualization
        # Downsample plotting to save resources
        if int(t * 10) % 2 == 0: 
            ax.clear()
            
            # Plot Occupancy Grid
            ax.imshow(occupancy_grid, extent=[-5, 5, -5, 5], origin='lower', cmap='gray', alpha=0.6)
            
            # Plot Robot
            ax.plot(create.pose[0], create.pose[1], 'ro', label='Robot')
            ax.arrow(create.pose[0], create.pose[1], 0.5*np.cos(create.pose[2]), 0.5*np.sin(create.pose[2]), 
                        head_width=0.2, head_length=0.3, fc='r', ec='r')

            # Plot Scan Points
            if scan_data is not None:
                ax.plot(P_G[0, :], P_G[1, :], 'b.', markersize=2, label='Lidar Points')
            
            # Plot Goal
            ax.plot(GOAL_POINT[0], GOAL_POINT[1], 'g*', markersize=15, label='Goal')

            # Plot Active Path
            if active_path is not None:
                ax.plot(active_path[:, 0], active_path[:, 1], 'g-', linewidth=2, label='Plan')
                # Highlight current target
                if wp_index < len(active_path):
                    ax.plot(active_path[wp_index, 0], active_path[wp_index, 1], 'y.', markersize=10)

            ax.set_xlim(-5, 5)
            ax.set_ylim(-5, 5)
            ax.set_title(f"Sim Time: {t:.2f}s")
            plt.draw()
            plt.pause(0.001)

    time.sleep(1.0)
    create.stop()
    plt.close('all')