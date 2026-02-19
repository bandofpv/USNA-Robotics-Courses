import time
import math
from matplotlib.path import Path
import numpy as np
from scipy import ndimage
from RRT import RRT
import matplotlib.pyplot as plt
from create3_sim import CreateSim

def wrapToPi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def on_close(event):
    global sim_running
    print("Window closed. Exiting simulation...")
    sim_running = False

create = CreateSim()

occupancy_grid = np.zeros((100, 100))
grid_size = 0.1  # Size of each grid cell
x_range = np.round(np.linspace(-5.0, 5.0, len(occupancy_grid)),decimals=1)
y_range = np.round(np.linspace(-5.0, 5.0, len(occupancy_grid)),decimals=1)
x_grid, y_grid = np.meshgrid(x_range, y_range)

robot_radius = 0.3  # Example robot radius in meters
inflation_radius = int(np.ceil(robot_radius / grid_size))  # Convert to grid cells

# Define map parameters for RRT
map_params_config = {
    'res': grid_size,
    'x_limit': [-5, 5],
    'y_limit': [-5, 5],
    'origin': [-5, -5] 
}

# Set goal point
GOAL_POINT = [-4.0, 2.0] 

# Simulation control flag
sim_running = True

# Path Following State
wp_index = 0
wp_radius = 0.1 # Radius to consider waypoint reached
active_path = None
robot_path = [] # For visualization of robot trajectory

if __name__ == '__main__':
    create.start()
    time.sleep(1.0)
    
    # Initialize plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(6,6))
    
    # Connect the close event
    fig.canvas.mpl_connect('close_event', on_close)

    # Build Occupancy Grid from Walls
    for i in range(21):
        ob_name = f'/wall[{i}]'
        wall_h = create.sim.getObjectHandle(ob_name)
        pos = create.sim.getObjectPosition(wall_h)
        eul = create.sim.getObjectOrientation(wall_h)
        res,type,dim = create.sim.getShapeGeomInfo(wall_h)
        
        pts_b = np.array([[dim[0]/2, -dim[0]/2, -dim[0]/2, dim[0]/2],
                        [dim[1]/2, dim[1]/2, -dim[1]/2, -dim[1]/2]])

        R_bg = np.array([[np.cos(eul[2]),-np.sin(eul[2])],[np.sin(eul[2]),np.cos(eul[2])]])

        pts_g = R_bg @ pts_b + np.array([[pos[0],pos[0],pos[0],pos[0]],
                                        [pos[1],pos[1],pos[1],pos[1]]])

        path = Path(pts_g.T)

        contains = path.contains_points(np.vstack((x_grid.flatten(), y_grid.flatten())).T)
        occupancy_grid[contains.reshape(x_grid.shape)] = 255

    # Inflate the occupied cells to create a buffer around obstacles
    occupancy_grid = ndimage.binary_dilation(occupancy_grid, iterations=inflation_radius).astype(occupancy_grid.dtype)

    t_start = create.get_sim_time()
    create.update()

    # Controller Gains
    Kp_yaw = 0.5
    Kp_speed = 2.0
    
    # Run simulation loop
    while sim_running: 
        t = create.get_sim_time()

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
        u = 0.0
        r = 0.0
        
        if active_path is not None:
            if wp_index < len(active_path):
                # Get current target waypoint
                current_target = active_path[wp_index]
                
                # Calculate Error
                dx = current_target[0] - create.pose[0]
                dy = current_target[1] - create.pose[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                # Speed control
                u_des = Kp_speed * dist

                # Check if reached waypoint
                if dist < wp_radius:
                    wp_index += 1
                else:
                    # Compute Control Inputs
                    psi_des = math.atan2(dy, dx)
                    psi_err = wrapToPi(psi_des - create.pose[2])
                    
                    # Heading controller
                    r_des = Kp_yaw * psi_err
                    r = max(-1.57, min(r_des, 1.57)) # Cap turn rate
                    
                    # Slow down if heading error is large
                    if abs(psi_err) > math.radians(20):
                        u = 0.0
                    else:
                        u = max(0, min(u_des, 0.306)) # Cap speed 
                        
            else:
                # Reached Goal
                print("Goal Reached!")
                u = 0.0
                r = 0.0
                # Optionally reset to find new goal or stop
                # active_path = None 
        else:
            # No path logic (spin to scan)
            u = 0.0
            r = 0.4
            
        create.set_cmd_vel(u, r)
        create.update()
        robot_path.append((create.pose[0], create.pose[1]))

        # Visualization
        # Downsample plotting to save resources
        if int(t * 10) % 2 == 0: 
            ax.clear()
            
            # Plot Occupancy Grid
            ax.imshow(occupancy_grid, extent=[-5, 5, -5, 5], origin='lower', cmap='gray', alpha=0.6)
            
            # Plot Robot
            ax.plot(create.pose[0], create.pose[1], 'ro')
            ax.arrow(create.pose[0], create.pose[1], 0.5*np.cos(create.pose[2]), 0.5*np.sin(create.pose[2]), 
                        head_width=0.2, head_length=0.3, fc='r', ec='r')

            # Plot RRT Tree
            if len(rrt_tree_nodes) > 0:
                ax.plot(rrt_tree_nodes[:, 0], rrt_tree_nodes[:, 1], 'c.', markersize=2)  

            # Plot Goal
            ax.plot(GOAL_POINT[0], GOAL_POINT[1], 'g*', markersize=15)

            # Plot Active Path
            if active_path is not None:
                ax.plot(active_path[:, 0], active_path[:, 1], 'g-', linewidth=2)
                # Highlight current target
                if wp_index < len(active_path):
                    ax.plot(active_path[wp_index, 0], active_path[wp_index, 1], 'y.', markersize=10)

            # Plot Robot Path
            if len(robot_path) > 1:
                path_array = np.array(robot_path)
                ax.plot(path_array[:, 0], path_array[:, 1], 'r-')

            ax.set_xlim(-5, 5)
            ax.set_ylim(-5, 5)
            ax.set_title(f"Sim Time: {t:.2f}s")
            plt.draw()
            plt.pause(0.001)

    time.sleep(1.0)
    create.stop()
    plt.close('all')