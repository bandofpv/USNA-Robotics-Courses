import math
import random
import numpy as np

class RRT:
    def __init__(
        self,
        start,
        goal,
        map_grid,
        map_params,
        expand_dis=0.5,
        max_iter=500,
        enable_pruning=True,
    ):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.map = map_grid
        self.res = map_params['res']
        self.x_limit = map_params['x_limit'] # [min, max]
        self.y_limit = map_params['y_limit'] # [min, max]
        self.origin = map_params['origin']   # [x, y] in meters corresponding to grid index (0,0)

        self.expand_dis = expand_dis
        self.max_iter = max_iter
        self.enable_pruning = enable_pruning
        self.tree = [self.start]
        self.parent = {tuple(self.start): None}

    # Convert continuous point to grid index
    def convert_to_grid(self, point):
        # x_idx corresponds to columns, y_idx corresponds to rows
        # Image row 0 is map_y_min, row end is map_y_max
        x_idx = int((point[0] - self.x_limit[0]) / self.res)
        y_idx = int((point[1] - self.y_limit[0]) / self.res)
        return (x_idx, y_idx)
    
    def plan(self):
        for _ in range(self.max_iter):
            # 1. Sample a random point
            rnd_point = np.array([random.uniform(self.x_limit[0], self.x_limit[1]), 
                                    random.uniform(self.y_limit[0], self.y_limit[1])])            
            
            # 2. Find the nearest node in the tree
            nearest_node = self.tree[np.argmin([np.linalg.norm(n - rnd_point) for n in self.tree])]
            
            # 3. Step toward the random point
            direction = (rnd_point - nearest_node) / np.linalg.norm(rnd_point - nearest_node)
            new_node = nearest_node + direction * self.expand_dis
            
            # 4. Collision Check (Segment Check) & Add to Tree
            if self.is_segment_collision_free(nearest_node, new_node):
                self.tree.append(new_node)
                self.parent[tuple(new_node)] = nearest_node
                
                # Check if goal is reached
                if np.linalg.norm(new_node - self.goal) <= self.expand_dis:
                    # Try to connect directly to goal
                    if self.is_segment_collision_free(new_node, self.goal):
                        self.tree.append(self.goal)
                        self.parent[tuple(self.goal)] = new_node
                        path = self.extract_path()
                        if self.enable_pruning:
                            return self.prune_path(path)
                        return path
        return None

    # Check if a single node is collision-free
    def is_collision_free(self, node):
        x_idx, y_idx = self.convert_to_grid(node)
        
        # Check bounds
        if x_idx < 0 or x_idx >= self.map.shape[1] or y_idx < 0 or y_idx >= self.map.shape[0]:
            return False
        
        # Check if the cell is occupied (1.0 means free, 0.5 mean unknown, 0.0 means occupied)
        if self.map[y_idx, x_idx] != 1.0:
            return False
        
        return True

    # Check if the line segment between start_node and end_node is collision-free
    def is_segment_collision_free(self, start_node, end_node):
        dist = np.linalg.norm(end_node - start_node)
        if dist == 0: return True
        
        # Calculate number of steps to check based on grid resolution
        steps = int(math.ceil(dist / (self.res * 0.5)))
        
        direction = (end_node - start_node) / dist
        
        for i in range(steps + 1):
             check_point = start_node + direction * (i * (dist / steps))
             if not self.is_collision_free(check_point):
                 return False
                 
        return True

    def extract_path(self):
        path = [self.goal]
        curr = tuple(self.goal)
        while curr is not None:
            if curr not in self.parent:
               break 
            
            val = self.parent[curr]
            if val is not None:
                path.append(val) 
            curr = tuple(val) if val is not None else None
        return path[::-1] # Return reversed path (start -> goal)

    # Shortcut-based path pruning: remove intermediate waypoints when a direct
    # segment between two farther points is collision-free.
    def prune_path(self, path):
        if path is None or len(path) <= 2:
            return path

        path_np = [np.array(p) for p in path]
        pruned = [path_np[0]]
        i = 0

        while i < len(path_np) - 1:
            next_i = i + 1

            # Greedily connect to the farthest reachable waypoint.
            for j in range(len(path_np) - 1, i, -1):
                if self.is_segment_collision_free(path_np[i], path_np[j]):
                    next_i = j
                    break

            pruned.append(path_np[next_i])
            i = next_i

        return pruned