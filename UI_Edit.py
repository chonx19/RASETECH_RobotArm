import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import cv2
from tkinter import StringVar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Create the main window
root = tk.Tk()
root.title("RASETECH")
root.geometry("1200x800")

# Set background color for the main window
root.configure(bg="#1e1e2e")  # Dark background color

# Customize style for the Notebook tabs
style = ttk.Style()
style.theme_use("clam")  # Use a modern theme for better appearance

# Configure the Notebook tabs with dark neon theme
style.configure('TNotebook', background="#1e1e2e", borderwidth=0)
style.configure('TNotebook.Tab', 
                font=('Exo 2', 16, 'bold'),  # Larger font
                padding=[20, 10],  # Increase padding
                background="#3a3a4f",  # Tab background
                foreground="#00ffcc",  # Neon text color
                borderwidth=1,
                relief="flat")
style.map('TNotebook.Tab', 
          background=[('selected', '#00ffcc'), ('active', '#3a3a4f')], 
          foreground=[('selected', '#1e1e2e'), ('active', '#00ffcc')])

# Style for frames and buttons
style.configure('Dark.TFrame', background="#1e1e2e")  # Dark background
style.configure('TButton', 
                font=('Arial', 14, 'bold'), 
                padding=5, 
                background="#3a3a4f", 
                foreground="#00ffcc",
                borderwidth=2,
                relief="flat")
style.map('TButton', 
          background=[('active', '#00ffcc')], 
          foreground=[('active', '#1e1e2e')])
style.configure('TEntry', 
                fieldbackground="#3a3a4f", 
                foreground="#00ffcc", 
                insertcolor="#00ffcc", 
                padding=5)

# Create a Notebook widget
notebook = ttk.Notebook(root)

# Create frames for each tab
tab1 = ttk.Frame(notebook)
tab2 = ttk.Frame(notebook)
tab3 = ttk.Frame(notebook)
tab4 = ttk.Frame(notebook)
tab5 = ttk.Frame(notebook)

# Add tabs to the Notebook
notebook.add(tab1, text="Move")
notebook.add(tab2, text="Program")
notebook.add(tab3, text="Setting")
notebook.add(tab4, text="Admin")
notebook.add(tab5, text="About")

# Center the Notebook at the top
notebook.pack(side='top', pady=20)

# Configure grid layout for Tab 1
tab1.columnconfigure(0, weight=1)  # Left side for controls
tab1.columnconfigure(1, weight=3)  # Right side for simulation

# Create frames for controls and simulation
control_frame = ttk.Frame(tab1, padding="10", style="Dark.TFrame")
control_frame.grid(row=0, column=0, sticky="ns", padx=10, pady=10)

simulation_frame = ttk.Frame(tab1, padding="10", relief="solid", borderwidth=1, style="Dark.TFrame")
simulation_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)


# Dropdown menu for COM ports
com_ports = []
selected_com = tk.StringVar()

# Variables and parameters for the robot arm
OA, AB, BC, CD, DE, EF, FG = 123.2, 53.1, 105.7, 205.5, 155.75, 40, 33
J_vars = [tk.DoubleVar(value=0.0) for _ in range(4)]
inverse_kinematic_vars = [tk.StringVar(value="0.0") for _ in range(4)]

# Matplotlib Figure
fig = plt.Figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=simulation_frame)
canvas.get_tk_widget().pack(expand=True, fill="both")

# Track if Real Mode is active and Home button has been pressed
real_mode_active = False
home_pressed = False



def update_arm():
    """Update the robot arm simulation using forward kinematics."""
    J1, J2, J3, J4 = [np.radians(J_vars[i].get()) for i in range(4)]
    A_angle = J2 + J3 + J4

    # Forward kinematics
    O = np.array([0, 0, 0])
    A = np.array([0, 0, OA])
    B = A + np.array([AB * np.cos(J1), AB * np.sin(J1), 0])
    C = B + np.array([BC * np.cos(np.radians(45)) * np.cos(J1),
                      BC * np.cos(np.radians(45)) * np.sin(J1),
                      BC * np.sin(np.radians(45))])
    D = C + np.array([CD * np.cos(J2) * np.cos(J1),
                      CD * np.cos(J2) * np.sin(J1),
                      CD * np.sin(J2)])
    E = D + np.array([DE * np.cos(J2 + J3) * np.cos(J1),
                      DE * np.cos(J2 + J3) * np.sin(J1),
                      DE * np.sin(J2 + J3)])
    F = E + np.array([EF * np.cos(J2 + J3 + J4) * np.cos(J1),
                      EF * np.cos(J2 + J3 + J4) * np.sin(J1),
                      EF * np.sin(J2 + J3 + J4)])
    G = F + np.array([FG * np.sin(J1), -FG * np.cos(J1), 0])

    # Update inverse kinematics display
    inverse_kinematic_vars[0].set(f"{G[0]:.4f}")
    inverse_kinematic_vars[1].set(f"{G[1]:.4f}")
    inverse_kinematic_vars[2].set(f"{G[2]:.4f}")
    inverse_kinematic_vars[3].set(f"{np.degrees(A_angle):.4f}")

    # Update the 3D plot
    update_3d_plot([O, A, B, C, D, E, F, G])

def update_3d_plot(points):
    ax.clear()
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')

    # Plot links and joints
    points = np.array(points)
    ax.plot(points[:, 0], points[:, 1], points[:, 2], color='orange', linewidth=5)
    ax.plot(points[:, 0], points[:, 1], points[:, 2], 'o', color='black', markersize=5)

    # Projection line for end-effector
    F0 = np.array([points[-1, 0], points[-1, 1], 0])
    dash_end = np.vstack([points[0], F0, points[-1]])
    ax.plot(dash_end[:, 0], dash_end[:, 1], dash_end[:, 2], '--k', linewidth=1)

    # Set limits and update the canvas
    ax.set_xlim3d(-700, 700)
    ax.set_ylim3d(-700, 700)
    ax.set_zlim3d(0, 700)
    canvas.draw()
    
def hold_increment(var_list, idx, step):
    """Continuously increment or decrement variable value while button is held."""
    global real_mode_active, home_pressed

    if real_mode_active and not home_pressed:
        # If in Real Mode and Home is not pressed, block control
        messagebox.showerror("Error", "Please use the Home button before controlling in Real Mode!")
        return

    # Proceed with incrementing or decrementing
    current_value = float(var_list[idx].get())
    var_list[idx].set(f"{current_value + step:.2f}")
    update_arm()
    # Schedule the next increment/decrement
    var_list[idx]._hold = root.after(100, hold_increment, var_list, idx, step)

def stop_increment(var_list, idx):
    """Stop the continuous increment or decrement when button is released."""
    if hasattr(var_list[idx], '_hold'):
        root.after_cancel(var_list[idx]._hold)
        var_list[idx]._hold = None

        
def refresh_com_ports():
    """Refresh the list of available COM ports."""
    global com_ports
    ports = serial.tools.list_ports.comports()
    com_ports = [port.device for port in ports]
    com_menu['values'] = com_ports
    if com_ports:
        selected_com.set(com_ports[0])  # Set default selection

def use_home():
    """Perform homing action for the robot."""
    global home_pressed

    # Update joint variables to home positions
    J_vars[0].set(0)   # J1 = 0
    J_vars[1].set(90)  # J2 = 90
    J_vars[2].set(0)   # J3 = 0
    J_vars[3].set(0)   # J4 = 0
    update_arm()       # Update the arm simulation

    if real_mode_active:
        print("Homing robot in Real Mode...")
        home_pressed = True  # Enable controls in Real Mode
        enable_buttons()     # Enable + and - buttons after homing
    else:
        print("Homing robot in Simulation Mode.")




# Define the Toggle button functionality
def toggle_mode():
    """Toggle between Simulation Mode and Real Mode with all necessary checks."""
    global real_mode_active, home_pressed

    if toggle_button['text'] == "Simulation Mode":
        # Check if a COM port is connected before switching to Real Mode
        if not selected_com.get():
            messagebox.showerror("Error", "Please select and connect to a COM port first!")
            return
        
        # Switch to Real Mode
        toggle_button.config(text="Real Mode", bg="green", fg="white")
        real_mode_active = True
        home_pressed = False  # Reset Home flag when switching to Real Mode
        disable_buttons()  # Disable + and - buttons
        print("Switched to Real Mode. Please use the Home button first.")
    else:
        # Switch back to Simulation Mode
        toggle_button.config(text="Simulation Mode", bg="red", fg="white")
        real_mode_active = False
        enable_buttons()  # Enable + and - buttons in Simulation Mode
        print("Switched to Simulation Mode.")



        
def disable_buttons():
    """Disable all + and - buttons."""
    for button in plus_buttons + minus_buttons:
        button.config(state="disabled")

def enable_buttons():
    """Enable all + and - buttons."""
    if real_mode_active and not home_pressed:
        return  # Do not enable buttons in Real Mode until Home is pressed
    for button in plus_buttons + minus_buttons:
        button.config(state="normal")





def connect_com():
    """Connect to the selected COM port."""
    selected_port = selected_com.get()
    if selected_port:
        print(f"Connecting to {selected_port}...")
        # Add real connection logic if needed
        messagebox.showinfo("Connection", f"Connected to {selected_port}")

        
# Horizontal frame for COM controls
com_controls_frame = ttk.Frame(control_frame, style="Dark.TFrame")
com_controls_frame.pack(pady=5, fill="x")

# COM port controls label
ttk.Label(com_controls_frame, text="COM Port", font=('Exo 2', 12, 'bold'),  # Reduced font size
          background="#1e1e2e", foreground="#00ffcc").pack(side="left", padx=5)

# Dropdown for COM ports
com_menu = ttk.Combobox(com_controls_frame, textvariable=selected_com, font=('Arial', 14),  # Reduced font size
                        state="readonly", width=10)
com_menu.pack(side="left", padx=5)

# Reduced frame for buttons
button_frame = ttk.Frame(com_controls_frame, style="Dark.TFrame", padding=(2, 2))
button_frame.pack(side="left", padx=3)

# Refresh button
refresh_button = tk.Button(
    button_frame,
    text="Refresh",
    font=('Arial', 10, 'bold'),  # Matching font size
    bg="#00cc66",  # Green background
    fg="white",  # White text
    relief="raised",
    command=refresh_com_ports  # Call refresh_com_ports function
)
refresh_button.pack(side="left", padx=5)  # Added consistent padding

# Connect button
connect_button = tk.Button(
    button_frame,
    text="Connect",
    font=('Arial', 10, 'bold'),  # Matching font size
    bg="#0066cc",  # Blue background
    fg="white",  # White text
    relief="raised",
    command=connect_com  # Call connect_com function
)
connect_button.pack(side="left", padx=5)  # Added consistent padding


# Frame for buttons on the same line
action_buttons_frame = ttk.Frame(control_frame, style="Dark.TFrame")
action_buttons_frame.pack(pady=10, fill="x")  # Frame for horizontal alignment

# Toggle button for mode switching
toggle_button = tk.Button(
    action_buttons_frame,
    text="Simulation Mode",
    font=('Arial', 10, 'bold'),  # Reduced font size
    bg="red",
    fg="white",
    relief="raised",
    command=toggle_mode
)
toggle_button.pack(side="left", padx=5)

# Home button
home_button = tk.Button(
    action_buttons_frame,
    text="Home",
    font=('Arial', 10, 'bold'),  # Matching font size
    bg="#00cc66",  # Green background
    fg="white",  # White text
    relief="raised",
    command=use_home  # Call use_home function
)
home_button.pack(side="left", padx=5)


# Stop button
stop_button = tk.Button(
    action_buttons_frame,
    text="Stop",
    font=('Arial', 10, 'bold'),  # Matching font size
    bg="#cc0000",  # Red background
    fg="white",  # White text
    relief="raised",
    command=lambda: print("Stopping robot...")  # Placeholder command
)
stop_button.pack(side="left", padx=5)




# Refresh COM ports on startup
refresh_com_ports()

# Create lists to track + and - buttons
plus_buttons = []
minus_buttons = []

# Create controls for forward kinematics
ttk.Label(control_frame, text="Forward Kinematics (Degrees)", font=('Exo 2', 14, 'bold'), background="#1e1e2e", foreground="#00ffcc").pack(pady=10)
for i, label in enumerate(['J1:', 'J2:', 'J3:', 'J4:']):
    row_frame = ttk.Frame(control_frame, style="Dark.TFrame")
    row_frame.pack(pady=5, fill="x")
    ttk.Label(row_frame, text=label, font=('Arial', 14, 'bold'), background="#1e1e2e", foreground="#00ffcc").pack(side="left", padx=5)
    ttk.Entry(row_frame, textvariable=J_vars[i], width=12, font=('Arial', 14), style='TEntry').pack(side="left", padx=5)

    # Add + button
    plus_button = ttk.Button(row_frame, text="+", width=2, style="TButton")
    plus_button.pack(side="left", padx=5)
    plus_button.bind("<ButtonPress-1>", lambda e, idx=i: hold_increment(J_vars, idx, 1))
    plus_button.bind("<ButtonRelease-1>", lambda e, idx=i: stop_increment(J_vars, idx))
    plus_buttons.append(plus_button)

    # Add - button
    minus_button = ttk.Button(row_frame, text="-", width=2, style="TButton")
    minus_button.pack(side="left", padx=5)
    minus_button.bind("<ButtonPress-1>", lambda e, idx=i: hold_increment(J_vars, idx, -1))
    minus_button.bind("<ButtonRelease-1>", lambda e, idx=i: stop_increment(J_vars, idx))
    minus_buttons.append(minus_button)


# Create controls for inverse kinematics
ttk.Label(control_frame, text="Inverse Kinematics (mm)", font=('Exo 2', 14, 'bold'), background="#1e1e2e", foreground="#00ffcc").pack(pady=10)
for i, label in enumerate(['X:', 'Y:', 'Z:', 'A:']):
    row_frame = ttk.Frame(control_frame, style="Dark.TFrame")
    row_frame.pack(pady=5, fill="x")
    ttk.Label(row_frame, text=label, font=('Arial', 14, 'bold'), background="#1e1e2e", foreground="#00ffcc").pack(side="left", padx=5)  # Increased font size
    ttk.Entry(row_frame, textvariable=inverse_kinematic_vars[i], width=12, font=('Arial', 14), style='TEntry').pack(side="left", padx=5)  # Increased text box width and font size
    # Add hold functionality to + and - buttons
    plus_button = ttk.Button(row_frame, text="+", width=2, style="TButton")  # Reduced size
    plus_button.pack(side="left", padx=5)
    plus_button.bind("<ButtonPress-1>", lambda e, idx=i: hold_increment(inverse_kinematic_vars, idx, 1))
    plus_button.bind("<ButtonRelease-1>", lambda e, idx=i: stop_increment(inverse_kinematic_vars, idx))

    minus_button = ttk.Button(row_frame, text="-", width=2, style="TButton")  # Reduced size
    minus_button.pack(side="left", padx=5)
    minus_button.bind("<ButtonPress-1>", lambda e, idx=i: hold_increment(inverse_kinematic_vars, idx, -1))
    minus_button.bind("<ButtonRelease-1>", lambda e, idx=i: stop_increment(inverse_kinematic_vars, idx))


# Initial simulation update
update_arm()


# Run the main event loop
root.mainloop()
