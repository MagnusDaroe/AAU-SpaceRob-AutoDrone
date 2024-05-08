#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import sys

import time
import threading
import math
import rclpy
from rclpy.node import Node
from drone.msg import DroneStatus, DroneControlData

#todo add color scheme to some of the states
#todo add waypoint
#todo add videofeed

def create_gui(window):
    """Creates the GUI for the ground control station.
    The whole framework and design of the GUI happens here,
    including the events and actions that the GUI performs with buttons and tab change"""
    #*############################ - Tab and window creation - ############################
    #specifies the window title, size, and amount of tabs
    window.title("Ground Control Station")
    x_window = 800
    y_window = 700
    window.geometry(f"{x_window}x{y_window}")
    window.style = ttk.Style()
    tabcontrol = ttk.Notebook(window)

    tab1 = ttk.Frame(tabcontrol)
    tabcontrol.add(tab1, text="Stats")
    
    global clock
    clock = tk.Label(tab1, text="Filler text")
    clock.pack()
    clock.place(x=450, y=10)
    
    tab2 = ttk.Frame(tabcontrol)
    tabcontrol.add(tab2, text="3D plot")

    tab3 = ttk.Frame(tabcontrol)
    tabcontrol.add(tab3, text="2D plot")
    
    tabcontrol.pack(expand=1, fill="both")
    #*############################ - Pose variables - ############################
    # Adds a label with the pose variables, and places them to the right
    poseLabel = tk.Label(tab1, text=f"x:\ny:\nz:\nroll:\npitch:\nyaw:\n", justify="right")
    poseLabel.pack()
    poseLabel.place(x=x_window-150, y=50) 
    # assigns some arbitrary values to the pose variables
    pose_data = np.array([1,2,4,8,16,32])
    # unpacks the pose data into the variables x, y, z, roll, pitch, yaw, and them to a label in the GUI
    global x, y, z, roll, pitch, yaw
    x, y, z, roll, pitch, yaw = pose_data[:]
    global poseVariables
    poseVariables = tk.Label(tab1, text=f"{x}\n{y}\n{z}\n{roll}\n{pitch}\n{yaw}\n", justify="left")
    poseVariables.pack()
    poseVariables.place(x=x_window-110, y=50)
    poseVariables_label = tk.Label(tab1, text="Pose Values", justify="left")
    poseVariables_label.pack()
    poseVariables_label.place(x=x_window-150, y=28)
    #*############################ - Drone Status - ############################
    # Add a label with the drone status, and places them to the right
    droneStatus = tk.Label(tab1, text=f"Connected:\nArmed:\nMode:\nBatteryOK:\nBattery%:\n", justify="right")
    droneStatus.pack()      
    droneStatus.place(x=x_window-193, y=175)
    global droneStates
    # Assign some arbitrary values to the drone status variables
    global Connected, Armed, Mode, BatteryOK, Battery
    Connected, Armed, Mode, BatteryOK, Battery = "TBD", "TBD", "TBD", "TBD", "TBD"
    droneStates = tk.Label(tab1, text=f"{Connected}\n{Armed}\n{Mode}\n{BatteryOK}\n{Battery}\n", justify="left")
    droneStates.pack()
    droneStates.place(x=x_window-110, y=175)

    #*############################ - Console_window - ############################
    #ands a console window to the GUI, which displays some console output
    global console_text

    # Add a label above the console text box
    console_label = tk.Label(tab1, text="Console Output:", justify="left")
    console_label.pack()
    console_label.place(x=10, y=y_window-230)
    # Console window
    console_text = tk.Text(tab1, height=11, width=50)
    console_text.pack(side=tk.BOTTOM, fill=tk.X)
    console_text.place(x=10, y=y_window-200)
    console_text.insert(tk.END, "Welcome to Drone Control Panel x3000:")
    console_text.configure(state='disabled') #makes the console text box read-only

    #*############################ - Waypoints - ##########################
    waypoint_labels_title = tk.Label(tab1, text="Waypoints:", justify="left")
    waypoint_labels_title.pack()
    waypoint_labels_title.place(x=415, y=y_window-230)

    waypoint_labels_next = tk.Label(tab1, text="Heading towards", justify="left")
    waypoint_labels_next.pack()
    waypoint_labels_next.place(x=x_window-150, y=y_window-230)

    global waypoint_labels
    waypoint_labels = tk.Label(tab1, text="1:\n2:\n3:\n", justify="right")
    waypoint_labels.pack()
    waypoint_labels.place(x=x_window-150, y=y_window-200)
    button_waypoint = ttk.Button(tab1, text="Update Waypoint")
    button_waypoint.place(x=x_window-150, y=y_window-280)

    waypoint_list_textbox = tk.Text(tab1, height=11, width=20)
    waypoint_list_textbox.pack(side=tk.BOTTOM, fill=tk.X)
    waypoint_list_textbox.place(x=410, y=y_window-200)

    def refresh_waypoints():
        console_print("Waypoints Updated")
        #import waypoints from .csv
        #update waypoint_labels
        waypoint_list = np.loadtxt("drone/scripts/waypoints.csv", delimiter=",")
        #make a waypoint text
        waypoint_list_textbox.configure(state='normal')
        waypoint_list_textbox.delete('1.0', tk.END)
        for i in range(len(waypoint_list)):
            waypoint_list_textbox.insert(tk.END, f"{waypoint_list[i]}\n")
        waypoint_list_textbox.configure(state='disabled')
    refresh_waypoints()
    button_waypoint.config(command=refresh_waypoints)

    #*############################ - MatplotLib - ############################
    #This section creates the three different plots, which are displayed in the GUI
    #embed a plot into tkinter window
    fig3d = Figure(figsize=(4, 4), dpi=100)  # Set the figure size to 400x400 pixels
    Traject3d = fig3d.add_subplot(111, projection='3d')
    Traject3d.scatter([0], [0], [0], c='r', marker='o')  # initial data
    
    fig3dTab2 = plt.figure()  # Set the figure size to 400x400 pixels
    Traject3dTab2 = fig3dTab2.add_subplot(111, projection='3d')
    Traject3dTab2.scatter([0], [0], [0], c='r', marker='o')  # initial data
    
    fig2d = plt.figure()
    Traject2d = fig2d.add_subplot(111)
    Traject2d.scatter([0], [0], [0], c='r', marker='o')  # initial data

    global canvas3d, canvas2d, canvas3dTab2
    canvas3d = FigureCanvasTkAgg(fig3d, master=tab1)  # Place canvas in tab1
    canvas3d.draw()
    canvas3d.get_tk_widget().pack(side=tk.LEFT, fill=tk.NONE, expand=False, padx=10, pady=10)  # Adjust placement and padding
    canvas3d.get_tk_widget().place(x=10, y=10)


    canvas3dTab2 = FigureCanvasTkAgg(fig3dTab2, master=tab2)
    canvas3dTab2.draw()
    canvas3dTab2.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
    
    canvas2d = FigureCanvasTkAgg(fig2d, master=tab3)
    canvas2d.draw()
    canvas2d.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
    
    #*############################ - Graph activation - ############################
    # This section adds a button to the GUI, which activates the 3D plot, and ensures that on the plots on the current tabs is updated.
    global tabUpdate
    tabUpdate = 0
    # Add graph update button
    button_3dGraph = ttk.Button(tab1, text="Activate 3D Graph")
    # Place button to the right
    button_3dGraph.place(x=10, y=10)
    def button_3dGraph_function():
        global tabUpdate
        if button_3dGraph.cget("text") == "Activate 3D Graph":
            button_3dGraph.configure(text="Deactivate 3D Graph")
            console_print("Graph activated")
            tabUpdate = 1
        else:
            button_3dGraph.configure(text="Activate 3D Graph")
            console_print("Graph deactivated")
            tabUpdate = 0
    button_3dGraph.config(command=button_3dGraph_function)
    

    def on_tab_change(event):
        global tabUpdate
        selected_tab = event.widget.tab(event.widget.select(), "text")
        if selected_tab == "3D plot":
            tabUpdate=2
        elif selected_tab == "2D plot":
            tabUpdate=3
        else:
            tabUpdate=0

    # Bind the tab change event to the on_tab_change function
    tabcontrol.bind("<<NotebookTabChanged>>", on_tab_change)

    #*############################ - Divider Lines - ############################
    #add gray vertical border in the middle of the window
    dividerLine = tk.Canvas(tab1, width=8, height=700) #creates a canvas with a width of 8 pixels and a height of 700 pixels
    dividerLine.create_line(4, 0, 4, 700, fill="gray", width=8) #creates a gray line in the middle of the canvas
    dividerLine.pack()
    dividerLine.place(x=405, y=0)
    #add gray horizontal border in the middle of the window
    dividerLine = tk.Canvas(tab1, width=800, height=8)
    dividerLine.create_line(0, 4, 800, 4, fill="gray", width=8) 
    dividerLine.pack()
    dividerLine.place(x=0, y=400)
    #add gray vertical border on the right of the window
    dividerLine = tk.Canvas(tab1, width=8, height=700)
    dividerLine.create_line(4, 0, 4, 700, fill="gray", width=8)
    dividerLine.pack()
    dividerLine.place(x=800, y=0)
    
    #add gray horizontal border in the bottom of the window
    dividerLine = tk.Canvas(tab1, width=808, height=8)
    dividerLine.create_line(0, 4, 808, 4, fill="gray", width=8)  
    dividerLine.pack()
    dividerLine.place(x=0, y=700)
    global first
    first = 0

    return Traject3d, Traject2d,Traject3dTab2  # Return the trajectory plots for updating plot later



def console_print(text):
    """Simple function to print text to the console text box.
    And ensure its read-only.

    Args:
        text (Any printable types): A variable which are printable
    """
    console_text.configure(state='normal')      #makes the console text box writeable
    console_text.insert(tk.END, "\n" + text)    #inserts the text in the console text box
    console_text.see(tk.END)                    #scrolls to the end of the console text box
    console_text.configure(state='disabled')    #makes the console text box read-only



def update_pose_variables(Traject3d, Traject2d, Traject3dTab2):
    """Updates the pose variables and plots the trajectory in 3D and 2D.

    Args:
        Traject3d (MatPlotLib Subplot): The 3d plot of the main tab
        Traject2d (MatPlotLib Subplot): The 2d plot of the 2d tab
        Traject3dTab2 (MatPlotLib Subplot): The 3d plot of the 3d tab   
    """
    global first
    #pose_data = data_update()   # Get the updated pose data
    global x, y, z, roll, pitch, yaw
    #x, y, z, roll, pitch, yaw = pose_data[:]    # Unpack the pose data
    poseVariables.config(text=f"{x:.2f}\n{y:.2f}\n{z:.2f}\n{roll:.2f}\n{pitch:.2f}\n{yaw:.2f}") # Update the pose variables in the GUI
    
    
    if first == 0:  # Initialize the trajectory arrays
        global trajectory_x, trajectory_y, trajectory_z
        trajectory_x = []
        trajectory_y = []
        trajectory_z = []
        first = 1

    # Append current coordinates to the trajectory
    trajectory_x.append(x)
    trajectory_y.append(y)
    trajectory_z.append(z)
    #axis limits
    lower_limit = -4000
    upper_limit = 4000
    if tabUpdate == 1: #determine which plot should be active
        # Update the 3D plot
        Traject3d.clear()
        Traject3d.scatter(x, y, z, c='r', marker='o')  # Pass arrays of coordinates
        # Plot the trajectory
        Traject3d.plot(trajectory_x, trajectory_y, trajectory_z, c='blue', linewidth=2)
        # Annotate the plotted point with its coordinates
        Traject3d.text(x, y, z, f'({x:.2f}, {y:.2f}, {z:.2f})', color='black', fontsize=10)
        # Set axis limits
        Traject3d.set_xlim(-3000, 3000)
        Traject3d.set_ylim(-1500, 3500)
        Traject3d.set_zlim(0, 3000)
        Traject3d.set_xlabel('X')
        Traject3d.set_ylabel('Y')
        Traject3d.set_zlabel('Z')
        canvas3d.draw()
    elif tabUpdate == 2:
        Traject3dTab2.clear()
        Traject3dTab2.scatter(x, y, z, c='r', marker='o')  # Pass arrays of coordinates
        # Plot the trajectory
        Traject3dTab2.plot(trajectory_x, trajectory_y, trajectory_z, c='blue', linewidth=2)
        # Annotate the plotted point with its coordinates
        Traject3dTab2.text(x, y, z, f'({x:.2f}, {y:.2f}, {z:.2f})', color='black', fontsize=10)
        # Set axis limits
        Traject3dTab2.set_xlim(-3000, 3000)
        Traject3dTab2.set_ylim(-1500, 3500)
        Traject3dTab2.set_zlim(0, 3000)
        Traject3dTab2.set_xlabel('X')
        Traject3dTab2.set_ylabel('Y')
        Traject3dTab2.set_zlabel('Z')
        canvas3dTab2.draw()
    elif tabUpdate == 3:
        # Update the 2D plot
        Traject2d.clear()
        Traject2d.scatter(x, y, c='r', marker='o')  # Pass arrays of coordinates
        # Plot the trajectory
        Traject2d.plot(trajectory_x, trajectory_y, c='blue', linewidth=2)
        # Annotate the plotted point with its coordinates
        Traject2d.text(x, y, f'({x:.2f}, {y:.2f})', color='black', fontsize=10)
        Traject2d.set_xlim(-3000, 3000)
        Traject2d.set_ylim(-1500, 3500)
        Traject2d.set_xlabel('X')
        Traject2d.set_ylabel('Y')
        canvas2d.draw()
        
    clock.config(text=f"Time: {time.strftime('%H:%M:%S', time.localtime())}")
    # Call the update function again after x ms
    window.after(1, update_pose_variables, Traject3d, Traject2d, Traject3dTab2)



class drone_listener(Node):
    def __init__(self):
        super().__init__('drone_listener')
        self.subscription = self.create_subscription(
            DroneStatus,
            '/status_fc',
            self.status_messages,
            10)
        
        self.subscription = self.create_subscription(
            DroneControlData,
            '/DroneControlData',
            self.pose_vicon,
            10)
        


        self.old_message = [0, 0, 0, 0, 0]
        #prints
        self.connectedConsoleDict = {0: "Flight Controller Disconnected", 1: "Flight Controller Connected"}
        self.armedConsoleDict = {0: "Drone Disarmed", 1: "Drone Armed"}
        self.modeConsoleDict = {0: "Manual Mode activated", 1: "Autonomous Mode activated", 2: "Reboot of Flight Controller"}
        self.batteryConsoleOKDict = {0: "Battery Low", 1: "Battery OK"}
        #labels
        self.connectedDict = {0: "FC Disconnected", 1: "FC Connected"}
        self.armedDict = {0: "Disarmed", 1: "Armed"}
        self.modeDict = {0: "Manual", 1: "Autonomous", 2: "Reboot"}
        self.batteryOKDict = {0: "Low", 1: "OK"}
        
    def pose_vicon(self, msg):
        # message type:
        # float64 timestamp
        # float32 vicon_x
        # float32 vicon_y
        # float32 vicon_z
        # float32 vicon_roll
        # float32 vicon_pitch
        # float32 vicon_yaw

        global x, y, z, roll, pitch, yaw
        x = msg.vicon_x
        y = msg.vicon_y
        z = msg.vicon_z
        roll = msg.vicon_roll
        pitch = msg.vicon_pitch
        yaw = msg.vicon_yaw


    def status_messages(self, msg):
        # message type:
        # float64 timestamp
        # int32 mode
        # uint8 battery_ok
        # float32 battery_percentage
        # uint8 fc_connection
        # uint8 armed

        global Connected, Armed, Mode, BatteryOK, Battery
        message = [msg.fc_connection, msg.armed, msg.mode, msg.battery_ok, msg.battery_percentage]#,msg.waypoint]
        #console prints
        ConnectedConsole = self.connectedConsoleDict[message[0]]
        ArmedConsole = self.armedConsoleDict[message[1]]
        ModeConsole = self.modeConsoleDict[message[2]]
        BatteryConsoleOK = self.batteryOKDict[message[3]]
    
        #labels
        Connected = self.connectedDict[message[0]]
        Armed = self.armedDict[message[1]]
        Mode = self.modeDict[message[2]]
        BatteryOK = self.batteryOKDict[message[3]]
        Battery = f"{int(message[4])}"+ "%"
        droneStates.config(text=f"{Connected}\n{Armed}\n{Mode}\n{BatteryOK}\n{Battery}\n")
        #waypoint = f"(message[5])"
        #waypoint_labels.config(text=f"Heading towards:\n{waypoint}")

        console_print(f"Time: {time.strftime('%H:%M:%S', time.localtime())}")
        if message != self.old_message: #checks if the has been an update to the messages, if so it prints the updated message
            if message[0] != self.old_message[0]:
                console_print(f"{ConnectedConsole}")
            if message[1] != self.old_message[1]:
                console_print(f"{ArmedConsole}")
            if message[2] != self.old_message[2]:
                console_print(f"{ModeConsole}")
            if message[3] != self.old_message[3]:
                console_print(f"{BatteryConsoleOK}")
        

        console_print(f"Battery%: {Battery}")

            
        self.old_message = message.copy()
        
        


def ros_spin():
    rclpy.init()
    listener = drone_listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()
    

def main():
    # Start a thread for ROS spin
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    global window
    window = tk.Tk() #creates the main window with tkinter
    Traject3d, Traject2d, Traject3dTab2 = create_gui(window)    # Create the GUI
    update_pose_variables(Traject3d, Traject2d, Traject3dTab2)  # Update pose variables initially
    window.protocol("WM_DELETE_WINDOW", exit_program)  # Call exit_program when window is closed


    
    window.mainloop()   # Start the main loop

def exit_program():
    # Close the window and exit the program
    window.destroy()
    sys.exit()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted by user")
        exit_program(window)