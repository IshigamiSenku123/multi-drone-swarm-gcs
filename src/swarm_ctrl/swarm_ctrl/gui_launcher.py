import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import subprocess

class SwarmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Swarm Mission Planner")
        self.root.geometry("350x450")
        self.config_path = os.path.expanduser("~/swarm_ws/src/swarm_ctrl/swarm_ctrl/mission_config.json")
        
        self.shape_var = tk.StringVar(value="equilateral")
        self.path_var = tk.StringVar(value="forward")
        
        self.create_widgets()

    def create_widgets(self):
        # Shape Selection
        shape_frame = ttk.LabelFrame(self.root, text="1. Formation Shape")
        shape_frame.pack(fill="x", padx=10, pady=5)
        ttk.Radiobutton(shape_frame, text="Equilateral Triangle", variable=self.shape_var, value="equilateral", command=self.update_dim_fields).pack(anchor="w")
        ttk.Radiobutton(shape_frame, text="Right Triangle", variable=self.shape_var, value="right_triangle", command=self.update_dim_fields).pack(anchor="w")
        ttk.Radiobutton(shape_frame, text="Custom Offsets", variable=self.shape_var, value="custom", command=self.update_dim_fields).pack(anchor="w")
        
        # Dimensions
        self.dim_frame = ttk.LabelFrame(self.root, text="2. Dimensions (meters)")
        self.dim_frame.pack(fill="x", padx=10, pady=5)
        self.dim_entries = {}
        self.update_dim_fields()
        
        # Path Selection
        path_frame = ttk.LabelFrame(self.root, text="3. Mission Path")
        path_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(path_frame, text="Path Type:").grid(row=0, column=0, padx=5, pady=5)
        ttk.Combobox(path_frame, textvariable=self.path_var, values=["forward", "circular (coming soon)"], state="readonly").grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(path_frame, text="Distance (m):").grid(row=1, column=0, padx=5, pady=5)
        self.dist_entry = ttk.Entry(path_frame)
        self.dist_entry.insert(0, "30.0")
        self.dist_entry.grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Label(path_frame, text="Speed (m/s):").grid(row=2, column=0, padx=5, pady=5)
        self.speed_entry = ttk.Entry(path_frame)
        self.speed_entry.insert(0, "1.0")
        self.speed_entry.grid(row=2, column=1, padx=5, pady=5)
        
        # Launch Button
        self.go_btn = ttk.Button(self.root, text="🚀 Let's Go!", command=self.launch_mission, style="Accent.TButton")
        self.go_btn.pack(pady=20, ipady=10, ipadx=20)

    def update_dim_fields(self):
        for widget in self.dim_frame.winfo_children(): widget.destroy()
        self.dim_entries.clear()
        shape = self.shape_var.get()
        if shape == 'equilateral':
            ttk.Label(self.dim_frame, text="Side Length (m):").grid(row=0, column=0, padx=5, pady=5)
            e = ttk.Entry(self.dim_frame); e.insert(0, "4.0"); e.grid(row=0, column=1, padx=5, pady=5)
            self.dim_entries['side_length'] = e
        elif shape == 'right_triangle':
            ttk.Label(self.dim_frame, text="Side A (m):").grid(row=0, column=0, padx=5, pady=5)
            e1 = ttk.Entry(self.dim_frame); e1.insert(0, "3.0"); e1.grid(row=0, column=1, padx=5, pady=5)
            self.dim_entries['side_a'] = e1
            ttk.Label(self.dim_frame, text="Side B (m):").grid(row=1, column=0, padx=5, pady=5)
            e2 = ttk.Entry(self.dim_frame); e2.insert(0, "4.0"); e2.grid(row=1, column=1, padx=5, pady=5)
            self.dim_entries['side_b'] = e2
        elif shape == 'custom':
            for i, d in enumerate(['drone0', 'drone1', 'drone2']):
                ttk.Label(self.dim_frame, text=f"{d} (x,y):").grid(row=i, column=0, padx=5, pady=5)
                e = ttk.Entry(self.dim_frame)
                e.insert(0, "0,0" if i==0 else "3,0" if i==1 else "-3,0")
                e.grid(row=i, column=1, padx=5, pady=5)
                self.dim_entries[d] = e

    def launch_mission(self):
        try:
            dist = float(self.dist_entry.get())
            speed = float(self.speed_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Distance and Speed must be numbers.")
            return
            
        config = {'shape': self.shape_var.get(), 'path_type': self.path_var.get(), 'path_distance': dist, 'speed': speed, 'dimensions': {}}
        shape = config['shape']
        if shape == 'equilateral': config['dimensions']['side_length'] = float(self.dim_entries['side_length'].get())
        elif shape == 'right_triangle': 
            config['dimensions']['side_a'] = float(self.dim_entries['side_a'].get())
            config['dimensions']['side_b'] = float(self.dim_entries['side_b'].get())
        elif shape == 'custom':
            for d in ['drone0', 'drone1', 'drone2']:
                vals = self.dim_entries[d].get().split(',')
                config['dimensions'][d] = [float(vals[0]), float(vals[1])]
                
        with open(self.config_path, 'w') as f: json.dump(config, f, indent=4)
        messagebox.showinfo("Success", "Config saved. Launching Controller...")
        
        # Launch the ROS 2 node via bash to ensure environment is sourced
        cmd = "source ~/swarm_ws/install/setup.bash && ros2 run swarm_ctrl swarm_controller"
        subprocess.Popen(['bash', '-c', cmd])

if __name__ == "__main__":
    root = tk.Tk()
    app = SwarmGUI(root)
    root.mainloop()