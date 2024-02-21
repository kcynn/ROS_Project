#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

class ServoControlApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Servo Control")
        self.switch_pub = rospy.Publisher('switch_topic', Bool, queue_size=10)

        # ROS Initialization
        rospy.init_node('servo_control')
        self.servo1_pub = rospy.Publisher('servo1', UInt16, queue_size=10)
        self.servo2_pub = rospy.Publisher('servo2', UInt16, queue_size=10)
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.msg = JointState()
        self.msg.name = ['joint1', 'joint2']
        self.msg.position = [0, 0]
        
        # Set background color
        self.master.configure(bg='lightsteelblue')
        
        # Create Scale Widget for Servo 1 Angle
        self.angle_label = ttk.Label(master, text="Servo 1 Angle (0-180 degrees):    °")
        self.angle_label.place(x=20, y=80)
        self.angle_scale = ttk.Scale(master, from_=0, to=180, length=500, orient=tk.HORIZONTAL, command=self.update_servo2)
        self.angle_scale.place(x=70, y=130)
        self.angle_scale.set(0)  # Set initial value to 0 degrees
        self.angle_label_value = ttk.Label(master, text="0")  # Label to display current angle
        self.angle_label_value.place(x=225, y=80)

        # Create Scale Widget for Servo 2 Angle
        self.angle_label2 = ttk.Label(master, text="Servo 2 Angle (0-180 degrees):    °")
        self.angle_label2.place(x=20, y=200)
        self.angle_scale2 = ttk.Scale(master, from_=0, to=180, length=500, orient=tk.HORIZONTAL, command=self.update_servo1)
        self.angle_scale2.place(x=70, y=250)
        self.angle_scale2.set(0)  # Set initial value to 0 degrees
        self.angle_label_value2 = ttk.Label(master, text="0")  # Label to display current angle
        self.angle_label_value2.place(x=225, y=200)

        # Create Radiobuttons for functions
        self.radio_var = tk.StringVar()
        self.radio_var.set("reset")
        self.radio_reset = ttk.Radiobutton(master, text="RESET", variable=self.radio_var, command=self.reset)
        self.radio_reset.place(x=200, y=40)
        self.radio_home = ttk.Radiobutton(master, text="HOME POS", variable=self.radio_var, command=self.home_pos)
        self.radio_home.place(x=350, y=40)

        # Create Button for switch mode
        self.mode_state_var = tk.BooleanVar()
        self.switch_mode_button = ttk.Button(master, text="Switch Mode",command=self.toggle_mode)
        self.switch_mode_button.place(x=270, y=300)

    def update_servo1(self, angle):
        angle_int = int(float(angle))
        self.servo1_pub.publish(angle_int)
        self.msg.position[0] = angle_int / 180.0 * 3.14159
        self.msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.msg)
        self.angle_label_value2.config(text=str(angle_int))  # Update label with current angle
    def update_servo2(self, angle):
        angle_int = int(float(angle))
        self.servo2_pub.publish(angle_int)
        self.msg.position[1] = angle_int / 180.0 * 3.14159
        self.msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.msg)
        self.angle_label_value.config(text=str(angle_int))  # Update label with current angle

    def reset(self):
        # Reset both servo motors to 0 degrees
        self.servo1_pub.publish(0)
        self.servo2_pub.publish(0)
        self.msg.position = [0, 0]
        self.msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.msg)
        self.angle_label_value.config(text="0")  # Reset label to show 0 degrees
        self.angle_label_value2.config(text="0")  # Reset label to show 0 degrees

    def home_pos(self):
        # Set both servo motors to home position
        self.servo1_pub.publish(90)
        self.servo2_pub.publish(0)
        self.msg.position = [90 / 180.0 * 3.14159, 0]
        self.msg.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.msg)
        self.angle_label_value.config(text="100")  # Update label to show home position for servo 1
        self.angle_label_value2.config(text="0")  # Update label to show home position for servo 2

    def toggle_mode(self):
        mode_state = not self.mode_state_var.get()
        self.send_mode_to_arduino(mode_state)
        self.mode_state_var.set(mode_state)
    def send_mode_to_arduino(self, mode_state):
        rospy.Publisher('mode', Bool, queue_size=10).publish(mode_state)

def main():
    root = tk.Tk()
    app = ServoControlApp(root)
    root.geometry("650x380+650+300")
    root.mainloop()

if __name__ == "__main__":
    main()
