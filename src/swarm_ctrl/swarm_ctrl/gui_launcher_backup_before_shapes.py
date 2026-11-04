#!/usr/bin/env python3
"""
Thin GUI Launcher for the Swarm Controller.
Does NOT modify flight logic. Only launches the proven CLI command.
"""
import os
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox

WORKSPACE = os.path.expanduser("~/swarm_ws")

class SwarmLauncherGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Swarm Mission Launcher")
        self.root.geometry("400x380")
        self.vars = {}

        info = tk.Label(
            self.root,
            text="1. Start restart_swarm.sh first.\n2. Wait for 'SWARM READY'.\n3. Click 'Let's Go!'",
            fg="darkblue",
            justify="left",
            font=("Arial", 10)
        )
        info.pack(padx=10, pady=(10, 0), anchor="w")

        frame = ttk.LabelFrame(self.root, text="Mission Parameters")
        frame.pack(fill="both", expand=True, padx=10, pady=10)

        fields = [
            ("forward_distance_m", "Forward Distance (m)", "30.0"),
            ("forward_speed_mps", "Forward Speed (m/s)", "0.5"),
            ("formation_hold_before_forward_sec", "Hold Before Forward (s)", "5.0"),
            ("formation_separation_m", "Formation Separation (m)", "2.0"),
            ("altitude_stagger_m", "Altitude Stagger (m)", "2.0"),
            ("base_altitude_m", "Base Altitude (m)", "5.0"),
        ]

        for row, (name, label, default) in enumerate(fields):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", padx=8, pady=6)
            var = tk.StringVar(value=default)
            entry = ttk.Entry(frame, textvariable=var, width=12)
            entry.grid(row=row, column=1, sticky="e", padx=8, pady=6)
            self.vars[name] = var

        self.launch_button = ttk.Button(
            self.root, text="🚀 Let's Go!", command=self.launch_controller
        )
        self.launch_button.pack(pady=12, ipady=8, ipadx=16)

        self.status_label = tk.Label(self.root, text="Ready.", fg="green", font=("Arial", 10, "bold"))
        self.status_label.pack(pady=(0, 10))

    def validate_floats(self):
        values = {}
        for name, var in self.vars.items():
            try:
                values[name] = float(var.get().strip())
            except ValueError:
                raise ValueError(f"{name} must be a number. Current: '{var.get()}'")
        if values["forward_distance_m"] < 0.0:
            raise ValueError("Forward distance cannot be negative.")
        if values["forward_speed_mps"] <= 0.0:
            raise ValueError("Forward speed must be > 0.")
        return values

    def launch_controller(self):
        try:
            values = self.validate_floats()
        except ValueError as e:
            messagebox.showerror("Invalid Parameter", str(e))
            return

        ros_args = " ".join([f"-p {name}:={value}" for name, value in values.items()])
        setup_path = os.path.join(WORKSPACE, "install", "setup.bash")
        cmd = f"source {setup_path} && ros2 run swarm_ctrl swarm_controller --ros-args {ros_args}"

        try:
            self.launch_button.config(state=tk.DISABLED)
            self.status_label.config(text="Launching controller...", fg="orange")
            self.root.update()
            
            subprocess.Popen(["bash", "-c", cmd])
            
            self.status_label.config(text="Controller launched! Check terminal.", fg="green")
            self.launch_button.config(state=tk.NORMAL)
        except Exception as e:
            self.launch_button.config(state=tk.NORMAL)
            self.status_label.config(text="Launch failed.", fg="red")
            messagebox.showerror("Launch Failed", str(e))

def main(args=None):
    root = tk.Tk()
    SwarmLauncherGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
