#!/usr/local/bin/python3

import numpy as np
import matplotlib.pyplot as plt

#pendulum geometry
g = 9.81
L = 0.33 # meters

#flywheel geometry, modeled as a disc with a hole
IN_TO_METER = 0.0254
R_o = 6/2*IN_TO_METER # 6in to meters
R_i = 5.5/2*IN_TO_METER # 5.5in inner radius -> 0.5" thickness
I = 1/2 * (R_o**2 + R_i**2) # moment of inertia without mass
print(I)

MAX_FLYWHEEL_ACCEL = 300 # rad/s^2
MAX_FLYWHEEL_VEL = 6000 * 2*np.pi / 60 # 1000 rpm max
#print(MAX_FLYWHEEL_VEL)


#initial conditions
theta = 0
omega = 0
theta_fly = 0
omega_fly = 0
alpha_fly = 0 # deg/s^2
b = 0
t = 0
dt = 1/60 # This works out to 60FPS. If we get numerical errors, we can implement frame skipping

# define our coordinate system
#0 deg is straight down
# +/-180deg is upright

#time period to simulate
T = 100

# PD controller gains
K_p = 50
K_d = 30
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

""" Normalize angle to be within -180 to 180 degrees """
def normalize_angle(angle):
    normalized_angle = (angle + 180) % 360 - 180
    return normalized_angle

""" Calculate the smallest angle difference with directional information """
def smallest_angle_diff(target, angle):
    normalized_target = normalize_angle(target)
    normalized_angle = normalize_angle(angle)
    if normalized_angle >= 0:
        return abs(normalized_target) - normalized_angle  # Deal with positive angles
    else:
        return normalized_target + abs(normalized_angle)  # Deal with negative angles

""" Clipping function to ensure the flywheel acceleration stays within physical limits """    
def clip_acceleration(input_accel):
    if input_accel >= MAX_FLYWHEEL_ACCEL: return MAX_FLYWHEEL_ACCEL
    if input_accel <= -MAX_FLYWHEEL_ACCEL: return -MAX_FLYWHEEL_ACCEL
    else: return input_accel

def is_effectively_zero(velocity, threshold=1e-16):
    return abs(velocity) < threshold

""" because things are discretized, checking if 
    1. the velocity is 0 
    2. the derivative of the the angle is 0
    3. the acceleration 
    won't work. instead, just check if the sign of the velocity has changed
    this indicates that the pendulum is swinging back down"""
'''def keepCoasting(curr_theta, old_theta):
    vel = (curr_theta-old_theta)/dt
    print(vel)
    if vel <= 1e-30:
        return False
    return True'''

# Let the flywheel keep following its momentum until it reaches the peak and starts coming back down
def keepCoasting(curr_vel, old_vel):
    return (curr_vel * old_vel) > 0 # POSITIVE IF KEEP COASTING, NEGATIVE IF NOT

def keepSpinning(omega_fly):
    if omega_fly >= MAX_FLYWHEEL_VEL: return False
    if omega_fly <= -MAX_FLYWHEEL_VEL: return False
    return True

def doPID(theta):
    if abs(target_angle) - abs(theta) <= 45: return True
    return False

control_accel = MAX_FLYWHEEL_ACCEL

flywheel_torques = []
gravity_torques = []

""" Run the numerical integration over the [0,T] with timestep dt """
while t<T:
    """ There's something wrong in how in the flywheel portion of my equation. The flywheel isn't generating enough torque even at 20 rad/s^2"""
    omega_new = omega - (g/L) * np.sin(np.deg2rad(theta)) * dt - b * omega * dt - (I/L**2)*control_accel * dt # Is this in deg/s or rad/s????
    theta_new = normalize_angle(theta + omega * dt)

    omega_fly_new = omega_fly + control_accel * dt
    theta_fly_new = theta_fly + omega_fly * dt

    flywheel_torques.append(I*control_accel)
    gravity_torques.append(g*np.sin(np.deg2rad(theta))*L)

    #if(g*np.sin(np.deg2rad(theta))*L > I*control_accel):
        #print("NOT ENOUGH TORQUE")
    
    if doPID(theta_new):
        error = smallest_angle_diff(target_angle, theta)
        error_dot = -omega
        control_accel = (K_p * error + K_d * error_dot) * -1
        print("RUNNING PID")
    else:
        """ The flywheel has hit its velocity limit but is not switching acceleration directions"""
        # the flywheel has hit its velocity limits
        if keepSpinning(omega_fly_new) != True:
            control_accel = 0 # set the acceleration to zero
            #print("STOP SPINNING!") 

            """ Current issue: the pendulum's velocity changes directions before the flywheel hits its velocity limits
                This means the flywheel is not able to generate enough torque to keep the pendulum going. That's fine, 
                but we need to rewrite the logic so we can spin the flywheel in the opp direction even before we hit the velocity limits"""
            # determine if it makes sense to reactive the flywheel, but in the opposite direction
            if keepCoasting(omega_new, omega) != True:
                flip_direction *= -1
                control_accel = MAX_FLYWHEEL_ACCEL * flip_direction
                #if omega_new < omega: control_accel = -MAX_FLYWHEEL_ACCEL
                #else: control_accel = MAX_FLYWHEEL_ACCEL
                #print("STOP COASTING!")

    # update current values
    theta, omega = theta_new, omega_new
    theta_fly, omega_fly = theta_fly_new, omega_fly_new

    times.append(t)
    angles.append(theta)
    velocities.append(omega)
    control_accels.append(control_accel)

    flywheel_thetas.append(theta_fly)
    flywheel_omegas.append(omega_fly)

    # increment time
    t += dt

#print(angles)
    
plt.plot(times, gravity_torques, label='gravity')
plt.plot(flywheel_torques)
plt.legend()
    
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
ax5.set_ylabel('Velocity (RPM)')

flywheel_omegas = [num * (60/(2*np.pi)) for num in flywheel_omegas]
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

def on_key(event):
    if event.key == 'r':
        # Reset the line and pendulum data to empty
        line.set_data([], [])
        pendulum.set_data([], [])
        # For patches like flywheel and arcs, you might need to reset their positions or other properties explicitly
        # Here's an example of resetting a circle's center, it might not be directly applicable without knowing the full context
        flywheel.center = (0, 0)
        flywheel_arc.center = (0, 0)
        flywheel_arc.theta1 = -arc_size  # Assuming you want to reset the arc's start angle, adjust according to your needs
        flywheel_arc.theta2 = arc_size  # Assuming you want to reset the arc's end angle, adjust according to your needs
        flywheel_arc.radius = flywheel_radius*2
        
        # If 'trace' is a Line2D object, reset its data like this
        trace.set_data([], [])
        
        # Restart the animation
        ani.event_source.stop()  # Stop the animation first
        ani.frame_seq = ani.new_frame_seq()  # Reset the frame sequence to start from the beginning
        ani.event_source.start()  # Start the animation

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
ani = FuncAnimation(fig, update, frames=N, init_func=init, blit=True, interval=dt*1000)
fig.canvas.mpl_connect('key_press_event', on_key)
plt.show()