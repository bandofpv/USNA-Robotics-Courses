import math
import random
import numpy as np

class RRT:
    def __init__(self, start, goal, map_grid, map_params, expand_dis=0.5, max_iter=500):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.map = map_grid
        self.res = map_params['res']
        self.x_limit = map_params['x_limit'] # [min, max]
        self.y_limit = map_params['y_limit'] # [min, max]
        self.origin = map_params['origin']   # [x, y] in meters corresponding to grid index (0,0)

        self.expand_dis = expand_dis
        self.max_iter = max_iter
        self.tree = [self.start]
        self.parent = {tuple(self.start): None}

    # Convert continuous point to grid index
    def convert_to_grid(self, point):
        x_idx = int((point[0] - self.origin[0]) / self.res)
        y_idx = int((point[1] - self.origin[1]) / self.res)
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
            if self._is_segment_collision_free(nearest_node, new_node):
                self.tree.append(new_node)
                self.parent[tuple(new_node)] = nearest_node
                
                # Check if goal is reached
                if np.linalg.norm(new_node - self.goal) <= self.expand_dis:
                    # Try to connect directly to goal
                    if self._is_segment_collision_free(new_node, self.goal):
                        self.tree.append(self.goal)
                        self.parent[tuple(self.goal)] = new_node
                        return self._extract_path()
        return None

    # Check if a single node is collision-free
    def _is_collision_free(self, node):
        x_idx, y_idx = self.convert_to_grid(node)
        
        # Check bounds
        if x_idx < 0 or x_idx >= self.map.shape[1] or y_idx < 0 or y_idx >= self.map.shape[0]:
            return False
        
        # Check if grid cell is occupied (1.0 means occupied in your grid)
        if self.map[y_idx, x_idx] > 0.5:
            return False
        
        return True

    # Check if the line segment between start_node and end_node is collision-free
    def _is_segment_collision_free(self, start_node, end_node):
        dist = np.linalg.norm(end_node - start_node)
        if dist == 0: return True
        
        # Calculate number of steps to check based on grid resolution
        steps = int(math.ceil(dist / (self.res * 0.5)))
        
        direction = (end_node - start_node) / dist
        
        for i in range(steps + 1):
             check_point = start_node + direction * (i * (dist / steps))
             if not self._is_collision_free(check_point):
                 return False
                 
        return True

    def _extract_path(self):
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