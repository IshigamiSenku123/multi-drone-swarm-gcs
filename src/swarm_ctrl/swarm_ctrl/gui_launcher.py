#!/usr/bin/env python3
"""
Thin GUI Launcher for the Swarm Controller.

This file does NOT modify flight logic.
It only launches the working ROS 2 controller with parameters.

Token format note (verified from the earlier rclpy UnknownROSArgsError):
ROS 2 / rclpy require '-p' and 'name:=value' as SEPARATE argv tokens.
Therefore build_parameter_tokens() emits them as two list items each,
and launch_controller() shell-quotes every token before joining.
"""

import os
import shlex
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox


WORKSPACE = os.path.expanduser("~/swarm_ws")


class SwarmLauncherGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Swarm Mission Launcher")
        self.root.geometry("500x580")

        self.shape_var = tk.StringVar(value="equilateral")
        self.shape_entries = {}
        self.common_entries = {}

        info = tk.Label(
            self.root,
            text=(
                "1. Start restart_swarm.sh first.\n"
                "2. Wait for 'SWARM READY'.\n"
                "3. Choose shape and mission parameters.\n"
                "4. Click 'Let's Go!'"
            ),
            fg="darkblue",
            justify="left",
            font=("Arial", 10),
        )
        info.pack(padx=10, pady=(10, 0), anchor="w")

        # -----------------------------
        # Shape selection
        # -----------------------------
        shape_box = ttk.LabelFrame(self.root, text="1. Formation Shape")
        shape_box.pack(fill="x", padx=10, pady=8)

        ttk.Label(shape_box, text="Shape:").grid(
            row=0, column=0, sticky="w", padx=8, pady=6
        )

        shape_combo = ttk.Combobox(
            shape_box,
            textvariable=self.shape_var,
            values=["equilateral", "right_triangle", "custom"],
            state="readonly",
            width=18,
        )
        shape_combo.grid(row=0, column=1, sticky="w", padx=8, pady=6)
        shape_combo.bind("<<ComboboxSelected>>", self.on_shape_changed)

        self.shape_field_box = ttk.LabelFrame(
            shape_box,
            text="Shape Parameters",
        )
        self.shape_field_box.grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="ew",
            padx=8,
            pady=8,
        )

        self.build_shape_fields()

        # -----------------------------
        # Common mission parameters
        # -----------------------------
        common_box = ttk.LabelFrame(
            self.root,
            text="2. Common Mission Parameters",
        )
        common_box.pack(fill="x", padx=10, pady=8)

        common_fields = [
            ("forward_distance_m", "Forward distance (m)", "30.0"),
            ("forward_speed_mps", "Forward speed (m/s)", "0.5"),
            ("formation_hold_before_forward_sec", "Hold before forward (s)", "5.0"),
            ("altitude_stagger_m", "Altitude stagger (m)", "2.0"),
            ("base_altitude_m", "Base altitude (m)", "5.0"),
        ]

        for row, (name, label, default) in enumerate(common_fields):
            ttk.Label(common_box, text=label).grid(
                row=row,
                column=0,
                sticky="w",
                padx=8,
                pady=6,
            )

            entry = ttk.Entry(common_box, width=14)
            entry.insert(0, default)
            entry.grid(
                row=row,
                column=1,
                sticky="e",
                padx=8,
                pady=6,
            )

            self.common_entries[name] = entry

        # -----------------------------
        # Launch button
        # -----------------------------
        self.launch_button = ttk.Button(
            self.root,
            text="Let's Go!",
            command=self.launch_controller,
        )
        self.launch_button.pack(pady=12, ipady=8, ipadx=16)

        self.status_label = tk.Label(
            self.root,
            text="Ready.",
            fg="green",
            font=("Arial", 10, "bold"),
        )
        self.status_label.pack(pady=(0, 10))

    def on_shape_changed(self, event=None):
        self.build_shape_fields()

    def build_shape_fields(self):
        for widget in self.shape_field_box.winfo_children():
            widget.destroy()

        self.shape_entries.clear()

        shape = self.shape_var.get()

        if shape == "equilateral":
            fields = [
                ("formation_separation_m", "Side length (m)", "2.0", 14),
            ]

        elif shape == "right_triangle":
            fields = [
                ("right_leg_a_m", "Leg A along +X (m)", "2.0", 14),
                ("right_leg_b_m", "Leg B along +Y (m)", "2.0", 14),
            ]

        elif shape == "custom":
            fields = [
                (
                    "custom_offsets_str",
                    "Offsets: drone0;drone1;drone2",
                    "0,0;2,0;1,1.732",
                    36,
                ),
            ]

        else:
            fields = []

        for row, (name, label, default, width) in enumerate(fields):
            ttk.Label(self.shape_field_box, text=label).grid(
                row=row,
                column=0,
                sticky="w",
                padx=8,
                pady=6,
            )

            entry = ttk.Entry(self.shape_field_box, width=width)
            entry.insert(0, default)
            entry.grid(
                row=row,
                column=1,
                sticky="e",
                padx=8,
                pady=6,
            )

            self.shape_entries[name] = entry

    def read_float(self, name, entry):
        raw = entry.get().strip()

        try:
            return float(raw)
        except ValueError:
            raise ValueError(
                f"{name} must be a number. Current value: '{raw}'"
            )

    def validate_custom_offsets(self, raw):
        raw = (raw or "").strip()

        if not raw:
            raise ValueError(
                "Custom offsets cannot be empty. "
                "Use format: '0,0;2,0;1,1.732'"
            )

        parts = [p.strip() for p in raw.split(";") if p.strip()]

        if len(parts) != 3:
            raise ValueError(
                "Custom offsets must contain exactly 3 pairs, "
                "separated by ';'. Example: '0,0;2,0;1,1.732'"
            )

        for i, part in enumerate(parts):
            coords = [c.strip() for c in part.split(",")]

            if len(coords) != 2:
                raise ValueError(
                    f"Custom offset {i} must be 'x,y'. Got: '{part}'"
                )

            try:
                float(coords[0])
                float(coords[1])
            except ValueError:
                raise ValueError(
                    f"Custom offset {i} is not numeric: '{part}'"
                )

    def build_parameter_tokens(self):
        # Each parameter contributes TWO separate argv tokens: '-p' and 'name:=value'.
        # This is required by rclpy (see the earlier UnknownROSArgsError, where a
        # combined '-p name:=value' string was rejected).
        shape = self.shape_var.get()

        tokens = ["-p", f"formation_shape:={shape}"]

        if shape == "equilateral":
            value = self.read_float(
                "formation_separation_m",
                self.shape_entries["formation_separation_m"],
            )
            tokens.extend(["-p", f"formation_separation_m:={value}"])

        elif shape == "right_triangle":
            a = self.read_float(
                "right_leg_a_m",
                self.shape_entries["right_leg_a_m"],
            )
            b = self.read_float(
                "right_leg_b_m",
                self.shape_entries["right_leg_b_m"],
            )
            tokens.extend(["-p", f"right_leg_a_m:={a}"])
            tokens.extend(["-p", f"right_leg_b_m:={b}"])

        elif shape == "custom":
            raw = self.shape_entries["custom_offsets_str"].get().strip()
            self.validate_custom_offsets(raw)
            tokens.extend(["-p", f"custom_offsets_str:={raw}"])

        else:
            raise ValueError(f"Unsupported shape: {shape}")

        for name, entry in self.common_entries.items():
            value = self.read_float(name, entry)
            tokens.extend(["-p", f"{name}:={value}"])

        forward_distance = self.read_float(
            "forward_distance_m",
            self.common_entries["forward_distance_m"],
        )
        forward_speed = self.read_float(
            "forward_speed_mps",
            self.common_entries["forward_speed_mps"],
        )

        if forward_distance < 0.0:
            raise ValueError("forward_distance_m cannot be negative.")

        if forward_distance > 0.0 and forward_speed <= 0.0:
            raise ValueError(
                "forward_speed_mps must be greater than 0 "
                "when forward_distance_m is greater than 0."
            )

        return tokens

    def launch_controller(self):
        try:
            tokens = self.build_parameter_tokens()

            setup_path = os.path.join(WORKSPACE, "install", "setup.bash")

            if not os.path.exists(setup_path):
                raise FileNotFoundError(
                    f"ROS 2 workspace setup file not found: {setup_path}"
                )

            # Quote every token individually so the shell passes each as one argv
            # element. This keeps '-p' and 'name:=value' separate, and protects
            # semicolons in custom_offsets_str.
            quoted_args = " ".join(shlex.quote(token) for token in tokens)

            cmd = (
                f"source {setup_path} && "
                f"ros2 run swarm_ctrl swarm_controller --ros-args {quoted_args}"
            )

            self.launch_button.config(state=tk.DISABLED)
            self.status_label.config(
                text="Launching controller...",
                fg="orange",
            )
            self.root.update()

            subprocess.Popen(["bash", "-c", cmd])

            self.status_label.config(
                text="Controller launched. Watch the terminal.",
                fg="green",
            )

            self.root.after(3000, lambda: self.launch_button.config(state=tk.NORMAL))

        except Exception as exc:
            self.launch_button.config(state=tk.NORMAL)
            self.status_label.config(text="Launch failed.", fg="red")
            messagebox.showerror("Launch failed", str(exc))


def main(args=None):
    root = tk.Tk()
    SwarmLauncherGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
