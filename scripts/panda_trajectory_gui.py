#!/usr/bin/env python3

"""
Franka Panda Trajectory GUI

This GUI allows users to control and create joint-space trajectories for the
Franka Emika Panda robot arm in a Gazebo simulation or real-world setup.

Features:
- Manual joint position control (slider + entry)
- Waypoint creation and management
- Save/load trajectories in YAML
- Play trajectories interactively
- Export full trajectory as JointTrajectory message for ROS controllers
"""

import os
import time
import yaml
import rospy
import threading
import tkinter as tk
from tkinter import filedialog
from functools import partial
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import JointCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# === Constants ===
JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7"
]

JOINT_LIMITS = {
    "panda_joint1": (-2.8973, 2.8973),
    "panda_joint2": (-1.7628, 1.7628),
    "panda_joint3": (-2.8973, 2.8973),
    "panda_joint4": (-3.0718, -0.0698),
    "panda_joint5": (-2.8973, 2.8973),
    "panda_joint6": (-0.0175, 3.7525),
    "panda_joint7": (-2.8973, 2.8973),
}

PREDEFINED_POSES = {
    "Initial Pose": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    "Ready Pose": [0.0, -1.0, 0.0, -1.5, 0.0, 2.0, 0.5],
    "Vertical Up": [0.0, -1.57, 0.0, -1.57, 0.0, 1.57, 0.0],
}

POSITION_MODE = 1

class PandaTrajectoryGUI:
    def __init__(self, master):
        self.master = master
        master.title("Franka Panda Trajectory GUI")

        self.entries = []
        self.slider_vars = []
        self.sliders = []
        self.waypoints = []
        self.joint_positions = {name: 0.0 for name in JOINT_NAMES}
        self.received_joint_states = False

        # --- ROS Setup ---
        rospy.init_node('panda_joint_gui', anonymous=True)
        self.joint_pub = rospy.Publisher('/panda_simulator/motion_controller/arm/joint_commands', JointCommand, queue_size=10)
        self.trajectory_pub = rospy.Publisher('/panda_trajectory_gui/follow_joint_trajectory', JointTrajectory, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # --- Layout Frames ---
        joint_frame = tk.LabelFrame(master, text="Joint Control", padx=10, pady=10)
        joint_frame.grid(row=0, column=0, padx=10, pady=10, sticky="n")

        wp_frame = tk.LabelFrame(master, text="Waypoint Manager", padx=10, pady=10)
        wp_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")

        traj_frame = tk.LabelFrame(master, text="Trajectory Tools", padx=10, pady=10)
        traj_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="ew")

        # --- Joint Sliders ---
        for i, joint in enumerate(JOINT_NAMES):
            tk.Label(joint_frame, text=joint).grid(row=i, column=0, sticky="e")

            entry = tk.Entry(joint_frame, width=8)
            entry.grid(row=i, column=1, padx=5)
            entry.insert(0, "0.0")
            entry.bind('<Return>', partial(self.update_slider_from_entry, i))
            entry.bind('<FocusOut>', partial(self.update_slider_from_entry, i))
            self.entries.append(entry)

            slider_var = tk.DoubleVar()
            self.slider_vars.append(slider_var)

            slider = tk.Scale(joint_frame, from_=JOINT_LIMITS[joint][0], to=JOINT_LIMITS[joint][1],
                              resolution=0.01, orient=tk.HORIZONTAL, length=200, variable=slider_var,
                              command=partial(self.update_entry_from_slider, i))
            slider.grid(row=i, column=2, padx=5)
            self.sliders.append(slider)

        tk.Button(joint_frame, text="Send Joint Positions", command=self.send_joint_positions).grid(row=8, column=0, columnspan=3, pady=5)
        tk.Button(joint_frame, text="Save as Waypoint", command=self.save_waypoint).grid(row=9, column=0, columnspan=3, pady=5)

         # --- Predefined Poses ---
        pose_frame = tk.LabelFrame(joint_frame, text="Predefined Poses", padx=5, pady=5)
        pose_frame.grid(row=10, column=0, columnspan=3, pady=10)
        for i, (label, pose) in enumerate(PREDEFINED_POSES.items()):
            tk.Button(pose_frame, text=label, width=18, command=partial(self.set_predefined_pose, pose)).grid(row=i, column=0, pady=2)


        # --- Waypoint Manager ---
        self.listbox = tk.Listbox(wp_frame, height=10, width=30, exportselection=False)
        self.listbox.grid(row=0, column=0, columnspan=2, pady=5)

        tk.Button(wp_frame, text="Delete", command=self.delete_waypoint).grid(row=1, column=0, pady=2, sticky="ew")
        tk.Button(wp_frame, text="Move Up", command=self.move_waypoint_up).grid(row=2, column=0, pady=2, sticky="ew")
        tk.Button(wp_frame, text="Move Down", command=self.move_waypoint_down).grid(row=3, column=0, pady=2, sticky="ew")
        tk.Button(wp_frame, text="Set Joint Positions", command=self.set_joint_positions_from_waypoint).grid(row=4, column=0, pady=2, sticky="ew")
        tk.Button(wp_frame, text="Play All", command=self.play_all_waypoints).grid(row=5, column=0, pady=2, sticky="ew")

        # --- Trajectory Tools ---
        tk.Label(traj_frame, text="Trajectory Name:").grid(row=0, column=0, sticky="e")
        self.trajectory_name_entry = tk.Entry(traj_frame, width=25)
        self.trajectory_name_entry.grid(row=0, column=1, padx=5, pady=5, sticky="w")

        tk.Button(traj_frame, text="Save Trajectory", command=self.save_trajectory_to_file).grid(row=1, column=0, pady=5, sticky="ew")
        tk.Button(traj_frame, text="Load Trajectory", command=self.load_trajectory_from_file).grid(row=1, column=1, pady=5, sticky="ew")
        tk.Button(traj_frame, text="Send Trajectory to the Real Robot", command=self.send_full_trajectory).grid(row=2, column=0, columnspan=2, pady=10)

        self.master.after(100, self.check_joint_states_received)

    # --- GUI ↔️ Slider Sync ---
    def update_entry_from_slider(self, index, value):
        self.entries[index].delete(0, tk.END)
        self.entries[index].insert(0, str(round(float(value), 4)))

    def update_slider_from_entry(self, index, event=None):
        try:
            val = float(self.entries[index].get())
            if JOINT_LIMITS[JOINT_NAMES[index]][0] <= val <= JOINT_LIMITS[JOINT_NAMES[index]][1]:
                self.slider_vars[index].set(val)
        except ValueError:
            rospy.logwarn("Invalid input in joint entry.")

    def set_predefined_pose(self, joint_values):
        for i, val in enumerate(joint_values):
            self.entries[i].delete(0, tk.END)
            self.entries[i].insert(0, str(round(val, 4)))
            self.slider_vars[i].set(val)
        self.send_joint_positions()

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                self.joint_positions[name] = pos
        self.received_joint_states = True

    def check_joint_states_received(self):
        if self.received_joint_states:
            for i, name in enumerate(JOINT_NAMES):
                val = round(self.joint_positions[name], 4)
                self.entries[i].delete(0, tk.END)
                self.entries[i].insert(0, str(val))
                self.slider_vars[i].set(val)
        else:
            self.master.after(100, self.check_joint_states_received)

    # --- Waypoint Handling ---
    def get_current_joint_positions(self):
        try:
            return [float(entry.get()) for entry in self.entries]
        except ValueError:
            rospy.logerr("Failed to get joint positions from entries.")
            return None

    def save_waypoint(self):
        pos = self.get_current_joint_positions()
        if pos:
            self.waypoints.append(pos)
            self.listbox.insert(tk.END, f"Waypoint {len(self.waypoints)}")

    def delete_waypoint(self):
        idx = self.listbox.curselection()
        if idx:
            self.waypoints.pop(idx[0])
            self.refresh_waypoint_listbox()

    def move_waypoint_up(self):
        idx = self.listbox.curselection()
        if idx and idx[0] > 0:
            i = idx[0]
            self.waypoints[i - 1], self.waypoints[i] = self.waypoints[i], self.waypoints[i - 1]
            self.refresh_waypoint_listbox()
            self.listbox.select_set(i - 1)

    def move_waypoint_down(self):
        idx = self.listbox.curselection()
        if idx and idx[0] < len(self.waypoints) - 1:
            i = idx[0]
            self.waypoints[i + 1], self.waypoints[i] = self.waypoints[i], self.waypoints[i + 1]
            self.refresh_waypoint_listbox()
            self.listbox.select_set(i + 1)

    def set_joint_positions_from_waypoint(self):
        idx = self.listbox.curselection()
        if idx:
            waypoint = self.waypoints[idx[0]]
            for i in range(len(JOINT_NAMES)):
                val = waypoint[i]
                self.entries[i].delete(0, tk.END)
                self.entries[i].insert(0, str(round(val, 4)))
                self.slider_vars[i].set(val)
            self.send_joint_positions()
            rospy.loginfo(f"Set joints from Waypoint {idx[0]+1}")

    def refresh_waypoint_listbox(self):
        self.listbox.delete(0, tk.END)
        for i in range(len(self.waypoints)):
            self.listbox.insert(tk.END, f"Waypoint {i+1}")

    # --- Playback & Trajectory Execution ---
    def play_all_waypoints(self):
        if not self.waypoints:
            rospy.logwarn("No waypoints to play.")
            return

        def play_sequence():
            for i, waypoint in enumerate(self.waypoints):
                self.listbox.select_clear(0, tk.END)
                self.listbox.select_set(i)
                self.listbox.activate(i)
                self.master.update_idletasks()

                for j in range(len(JOINT_NAMES)):
                    val = waypoint[j]
                    self.entries[j].delete(0, tk.END)
                    self.entries[j].insert(0, str(round(val, 4)))
                    self.slider_vars[j].set(val)

                self.send_joint_positions()
                rospy.loginfo(f"Playing Waypoint {i+1}")
                time.sleep(1.5)

        threading.Thread(target=play_sequence).start()

    def send_joint_positions(self):
        positions = self.get_current_joint_positions()
        if not positions:
            return
        msg = JointCommand()
        msg.mode = POSITION_MODE
        msg.names = JOINT_NAMES
        msg.position = positions
        self.joint_pub.publish(msg)
        rospy.loginfo(f"Sent JointCommand: {positions}")

    def build_joint_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        time_per_point = 2.0
        for i, waypoint in enumerate(self.waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = rospy.Duration(time_per_point * (i + 1))
            msg.points.append(point)
        return msg

    def send_full_trajectory(self):
        if not self.waypoints:
            rospy.logwarn("No trajectory to send.")
            return
        traj_msg = self.build_joint_trajectory()
        self.trajectory_pub.publish(traj_msg)
        rospy.loginfo("Published full trajectory.")

    # --- Trajectory File Handling ---
    def save_trajectory_to_file(self):
        name = self.trajectory_name_entry.get().strip()
        if not name:
            rospy.logwarn("Trajectory name is empty.")
            return
        if not self.waypoints:
            rospy.logwarn("No waypoints to save.")
            return

        trajectory_data = {
            "name": name,
            "waypoints": self.waypoints
        }

        file_path = filedialog.asksaveasfilename(
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
            title="Save Trajectory As"
        )

        if file_path:
            try:
                with open(file_path, 'w') as f:
                    yaml.dump(trajectory_data, f, default_flow_style=False)
                rospy.loginfo(f"Trajectory saved to {file_path}")
            except Exception as e:
                rospy.logerr(f"Failed to save trajectory: {e}")

    def load_trajectory_from_file(self):
        file_path = filedialog.askopenfilename(
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
            title="Load Trajectory"
        )
        if file_path and os.path.isfile(file_path):
            try:
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f)
                name = data.get("name", "")
                self.trajectory_name_entry.delete(0, tk.END)
                self.trajectory_name_entry.insert(0, name)
                self.waypoints = data.get("waypoints", [])
                self.refresh_waypoint_listbox()
                rospy.loginfo(f"Loaded trajectory from {file_path}")
            except Exception as e:
                rospy.logerr(f"Failed to load trajectory: {e}")

if __name__ == '__main__':
    root = tk.Tk()
    gui = PandaTrajectoryGUI(root)
    root.mainloop()
