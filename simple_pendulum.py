#!/usr/local/bin/python3

import numpy as np
import matplotlib.pyplot as plt

#pendulum geometry
g = 9.81
L = 0.33 # meters

#flywheel geometry, modeled as a disc with a hole
R_o = 0.076 # 6in to meters
R_i = 0.064 # 5.5in inner radius -> 0.5" thickness
I = 1/2 * (R_o**2 + R_i**2) # moment of inertia without mass

MAX_FLYWHEEL_ACCEL = 5 # This assumes we can accelerate forever! Need limits on the angular velocity too
MAX_FLYWHEEL_VEL = 1000 * 2*np.pi / 60 # 1000 rpm max
#print(MAX_FLYWHEEL_VEL)


#initial conditions
theta = 150
omega = 0
theta_fly = 0
omega_fly = 0
alpha_fly = 0 # deg/s^2
b = 5
t = 0
dt = 0.01

# define our coordinate system
#0 deg is straight down
# +/-180deg is upright

#time period to simulate
T = 100

# PD controller gains
K_p = 0.5
K_d = 1
target_angle = -180
control_accel = 0
flip_direction = 1 # Cheap and dirty way to do an energy-based swing

#lists to store sim data
times = []
angles = []
velocities = []
flywheel_omegas = []
flywheel_thetas = []
control_accels = []

"""Normalize angle to be within -180 to 180 degrees."""
def normalize_angle(angle):
    normalized_angle = (angle + 180) % 360 - 180
    return normalized_angle

"""Calculate the smallest angle difference with directional information."""
def smallest_angle_diff(target, angle):
    normalized_target = normalize_angle(target)
    normalized_angle = normalize_angle(angle)
    if normalized_angle >= 0:
        return abs(normalized_target) - normalized_angle  # Deal with positive angles
    else:
        return normalized_target + abs(normalized_angle)  # Deal with negative angles

""" Prevent the acceleration of the """
def clip_acceleration(control_accel):
    if(abs(control_accel) >= MAX_FLYWHEEL_ACCEL): print("y")

'''Control the flywheel to spin in the opposite direction if the current speed limit has been reached
def flip_acceleration(current_vel):
    if(abs(current_vel) >= MAX_FLYWHEEL_VEL):
        if(current_vel >= 0):
            return'''

""" Run the numerical integration over the [0,T] with timestep dt """
while t<T:
    omega_new = omega - (g/L) * np.sin(np.deg2rad(theta)) * dt - b * omega * dt - (I/L**2)*control_accel     #alpha_fly
    theta_new = normalize_angle(theta + omega * dt)

    omega_fly_new = omega_fly + alpha_fly * dt
    theta_fly_new = theta_fly + omega_fly * dt

    # update current values
    theta, omega = theta_new, omega_new
    theta_fly, omega_fly = theta_fly_new, omega_fly_new

    # update the error for the PD controller
    #if(theta>=0): error = target_angle-theta
    #else: error = target_angle-abs(theta)

    error = smallest_angle_diff(target_angle, theta)
    error_dot = -omega # the target angular velocity is 0

    # set the control acceleration
    # have to invert otherwise the algorithm does the opposite acceleration 
    control_accel = (K_p * error + K_d * error_dot) * (-1)
    print(control_accel)

    ''' Clipping
    if(abs(control_accel) >= MAX_FLYWHEEL_ACCEL): control_accel = MAX_FLYWHEEL_ACCEL * flip_direction
    # flip the direction of the control acceleration if the flywheel has reached its max velocity
    #print(omega_fly)
    if(abs(omega_fly) >= MAX_FLYWHEEL_VEL): 
        #print('true')
        flip_direction *= -1'''

    times.append(t)
    angles.append(theta)
    velocities.append(omega)
    control_accels.append(control_accel)

    flywheel_thetas.append(theta_fly)
    flywheel_omegas.append(omega_fly)

    # increment time
    t += dt

#print(angles)
    
''' ANIMATION '''

# Set up the figure with subplots
fig = plt.figure(figsize=(20, 10))

# Set up the subplot for the pendulum animation (2 row, 4 columns, first subplot)
ax1 = fig.add_subplot(2, 4, (1,6))
ax1.set_xlim(-L * 1.25, L * 1.25)
ax1.set_ylim(-L * 1.25, L * 1.25)

# Pendulum angle vs. time plot
ax2 = fig.add_subplot(2, 4, 3)
ax2.set_title('Pendulum Angle vs. Time')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle (deg)')
ax2.plot(times, angles)

# Pendulum velocity vs. time plot
ax3 = fig.add_subplot(2, 4, 4)
ax3.set_title('Pendulum Velocity vs. Time')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Velocity (rad/s)')
ax3.plot(times, velocities)

# Flywheel control accel vs. time plot
ax4 = fig.add_subplot(2, 4, 7)
ax4.set_title('Flywheel Control Accel vs. Time')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Accel (rad/s^2)')
ax4.plot(times, control_accels)

# Flywheel velocity vs. time plot
ax5 = fig.add_subplot(2, 4, 8)
ax5.set_title('Flywheel velocity vs. Time')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Velocity (rad/s)')
ax5.plot(times, flywheel_omegas)



# Plot the results
'''plt.plot(times, angles)
plt.xlabel('Time (s)')
plt.ylabel('Angular displacement (rad)')
plt.title('Simple Pendulum Simulation (Euler Method)')
plt.show()'''

from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arc

# define number of animation frames
N = int(T/dt)

# Plotting and animation
'''fig, ax = plt.subplots(figsize=(5, 5))
ax.set_xlim(-L * 1.2, L * 1.2)
ax.set_ylim(-L * 1.2, L * 1.2)'''

flywheel_radius = R_o
arc_size = 10 # Degrees, for example

# Elements to animate
line, = ax1.plot([], [], lw=2, color='black')
trace, = ax1.plot([], [], lw=1, alpha=0.5)
pendulum, = ax1.plot([], [], 'o', color='red', markersize=10)

# Flywheel element animation
flywheel = plt.Circle((0, -L), R_o, fill=False, color='black', lw=4, animated=True)
ax1.add_patch(flywheel)

# Create a wedge on the flywheel to indicate its position
flywheel_arc = Arc((0, 0), flywheel_radius*2, flywheel_radius*2, angle=0, 
                   theta1=-arc_size, theta2=arc_size, color='red', lw=4, animated=True)
ax1.add_patch(flywheel_arc)

# Initialize elements
def init():
    line.set_data([], [])
    #trace.set_data([], [])
    pendulum.set_data([], [])
    return line, trace, pendulum

# Animation update function
def update(frame):
    # Pendulum position
    x = L * np.sin(np.deg2rad(angles[frame]))
    y = -L * np.cos(np.deg2rad(angles[frame]))
    
    # Update pendulum elements
    line.set_data([0, x], [0, y])
    pendulum.set_data(x, y)
    
    # Update flywheel position (centered on pendulum end)
    flywheel.center = (x,y)

    # Update flywheel angular position via an arc
    flywheel_arc.set_center((x,y))
    flywheel_arc.set_angle(flywheel_thetas[frame])

    trace.set_data(L * np.sin(np.deg2rad(angles[:frame])), -L * np.cos(np.deg2rad(angles[:frame])))
    return line, pendulum, flywheel, flywheel_arc, trace

# Create the animation
ani = FuncAnimation(fig, update, frames=N, init_func=init, blit=True, interval=dt*100)
plt.show()