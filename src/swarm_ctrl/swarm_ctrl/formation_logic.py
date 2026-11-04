import math
import json
import os

class FormationPlanner:
    def __init__(self, config_path="~/swarm_ws/src/swarm_ctrl/swarm_ctrl/mission_config.json"):
        self.config_path = os.path.expanduser(config_path)
        self.load_config()

    def load_config(self):
        with open(self.config_path, 'r') as f:
            self.config = json.load(f)
        
        self.shape = self.config['shape']
        self.dimensions = self.config['dimensions']
        self.path_type = self.config['path_type']
        self.path_distance = self.config['path_distance']
        self.speed = self.config.get('speed', 1.0)
        
        # CRITICAL: Spawn offsets from restart_swarm.sh (Drone 1 is +3X, Drone 2 is -3X)
        # Used to transform World coordinates into each drone's local EKF frame.
        self.spawn_offsets = {
            'drone0': (0.0, 0.0),
            'drone1': (3.0, 0.0),
            'drone2': (-3.0, 0.0)
        }
        
        self.shape_offsets = self._calculate_shape_offsets()
        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.is_moving = False
        self.last_time = None

    def _calculate_shape_offsets(self):
        # Returns (dx, dy) for each drone relative to the formation centroid
        if self.shape == 'equilateral':
            L = self.dimensions['side_length']
            h = L * math.sqrt(3) / 2
            cy = -h / 3
            return {
                'drone0': (0.0, 2*h/3),      # Top
                'drone1': (-L/2, cy),        # Bottom Left
                'drone2': (L/2, cy)          # Bottom Right
            }
        elif self.shape == 'right_triangle':
            a = self.dimensions['side_a']
            b = self.dimensions['side_b']
            return {
                'drone0': (0.0, b),          
                'drone1': (0.0, 0.0),        # Right angle vertex
                'drone2': (a, 0.0)           
            }
        elif self.shape == 'custom':
            return {
                'drone0': tuple(self.dimensions['drone0']),
                'drone1': tuple(self.dimensions['drone1']),
                'drone2': tuple(self.dimensions['drone2'])
            }
        return {'drone0': (0,0), 'drone1': (0,0), 'drone2': (0,0)}

    def start_moving(self):
        self.is_moving = True
        self.last_time = None

    def update(self, current_time_msg):
        if self.last_time is None:
            self.last_time = current_time_msg
            return
        
        dt = (current_time_msg.sec - self.last_time.sec) + \
             (current_time_msg.nanosec - self.last_time.nanosec) / 1e9
        self.last_time = current_time_msg

        if self.is_moving and self.path_type == 'forward':
            if self.centroid_x < self.path_distance:
                self.centroid_x += self.speed * dt
                if self.centroid_x >= self.path_distance:
                    self.centroid_x = self.path_distance
                    self.is_moving = False

    def get_local_setpoints(self):
        # 1. Calculate World targets (relative to Drone 0's spawn at 0,0)
        world_targets = {}
        for drone_id, (dx, dy) in self.shape_offsets.items():
            wx = self.centroid_x + dx
            wy = self.centroid_y + dy
            world_targets[drone_id] = (wx, wy)
            
        # 2. CRITICAL TRANSFORM: Convert World targets to Local frames for each drone
        local_setpoints = {}
        for drone_id, (wx, wy) in world_targets.items():
            spawn_ox, spawn_oy = self.spawn_offsets[drone_id]
            lx = wx - spawn_ox
            ly = wy - spawn_oy
            local_setpoints[drone_id] = (lx, ly, 5.0) # 5.0m altitude
            
        return local_setpoints

    def is_complete(self):
        return not self.is_moving and self.centroid_x >= self.path_distance