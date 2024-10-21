import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Define the physical properties of the robot arm
m = 2.0  # Mass of the arm (kg)
l = 1.0  # Length of the arm (m)
I = (1/3) * m * l**2  # Moment of inertia of the arm

b = 0.05  # Damping coefficient
Kp = 5.0  # Proportional gain
Kd = 1.0  # Derivative gain
theta_ref = np.radians(90)  # Reference angle (in radians)

# Define the system of ODEs (dynamics of the robot arm)
def dynamics(state, t, I, b, Kp, Kd, theta_ref):
    theta, theta_dot = state
    # PD controller
    control_torque = Kp * (theta_ref - theta) - Kd * theta_dot
    # Dynamics of the system: I * theta_ddot + b * theta_dot = control_torque
    theta_ddot = (control_torque - b * theta_dot) / I
    return [theta_dot, theta_ddot]

# Time settings for the simulation
t = np.linspace(0, 10, 500)  # Simulate for 10 seconds, 500 time steps

# Initial conditions: [theta(0), theta_dot(0)]
initial_state = [0, 0]  # Start with angle 0 and angular velocity 0

# Solve the ODE
solution = odeint(dynamics, initial_state, t, args=(I, b, Kp, Kd, theta_ref))

# Extract the angular position (theta) and angular velocity (theta_dot)
theta = solution[:, 0]  # Angular position over time
theta_dot = solution[:, 1]  # Angular velocity over time

# Real-time plot with actual theta and reference angle
plt.ion()  # Turn on interactive mode
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

# Set up for angle and reference line plot
ax1.set_xlim(0, 10)
ax1.set_ylim(0, np.degrees(theta_ref) + 10)
line1, = ax1.plot([], [], label='Actual Angle (Degrees)')
line2, = ax1.plot(t, [np.degrees(theta_ref)]*len(t), 'r--', label=f'Reference Angle: {np.degrees(theta_ref):.1f}°')
ax1.set_title('Angular Position vs. Time')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (Degrees)')
ax1.legend()

# Set up for robot arm visualization
ax2.set_xlim(-1.5, 1.5)
ax2.set_ylim(-1.5, 1.5)
robot_arm, = ax2.plot([], [], lw=4, marker='o', markersize=10)
ax2.set_title("Real-time Robot Arm Movement")

# Add reference line for the target position (reference angle)
x_ref = l * np.cos(theta_ref)  # x-coordinate for the reference angle
y_ref = l * np.sin(theta_ref)  # y-coordinate for the reference angle
ax2.plot([0, x_ref], [0, y_ref], 'r--', label=f'Reference Angle: {np.degrees(theta_ref):.1f}°')
ax2.legend()

# Real-time plotting loop
for i in range(len(t)):
    # Update the actual theta plot
    line1.set_data(t[:i], np.degrees(theta[:i]))
    
    # Update robot arm visualization
    x_end = l * np.cos(theta[i])  # x-coordinate of the arm's end
    y_end = l * np.sin(theta[i])  # y-coordinate of the arm's end
    robot_arm.set_data([0, x_end], [0, y_end])  # Plot arm from (0,0) to (x_end, y_end)

    # Redraw the figure
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.01)

# Turn off interactive mode and display the final plot
plt.ioff()
plt.show()
