import matplotlib.pyplot as plt
import numpy as np
import control


def plot_root_locus(K_motor, wheel_diameter, max_rps, dt=0.1):
    """
    Plots the root locus for the robot forward movement control system.

    Parameters:
    - K_motor: Motor gain (default 1.0, unitless because we normalize)
    - wheel_diameter: Wheel diameter in cm (default 6.5 cm)
    - max_rps: Maximum motor RPS at full PWM (default 5 RPS)
    - dt: Time step for discrete simulation (not critical for root locus but included for later)
    """

    # Basic model is still 1/s because physical meaning does not change
    # If needed, K_motor will scale gain later (but location of poles is critical now)

    # Define open-loop system G(s) = K_motor / s
    num = [1]
    den = [1, 0]  # s in denominator

    system = control.TransferFunction(num, den)

    # Plot root locus
    control.root_locus(system, kvect=np.linspace(0, 10, 100), grid=True)
    plt.title(f'Root Locus: G(s) = {K_motor}/s | Wheel Diameter={wheel_diameter} cm, Max RPS={max_rps}')
    plt.xlabel('Real Axis')
    plt.ylabel('Imaginary Axis')
    plt.grid(True)
    plt.show()


# Example usage:
plot_root_locus(K_motor=6.5, wheel_diameter=6.5, max_rps=5, dt=0.1)

# Parameters
target_distance = 100  # cm
wheel_diameter = 6.5   # cm
wheel_circumference = np.pi * wheel_diameter  # cm
max_rps = 5  # rotations per second at full 255 PWM
dt = 0.1  # seconds, simulation time step
Kp = 6 # proportional gain

# Initialize variables
distance = 0.0
time = 0.0
distance_list = []
time_list = []

while distance < target_distance-0.001:
    error = target_distance - distance
    pwm = Kp * error  # proportional control
    pwm = max(0, min(255, pwm))  # saturate between 0 and 255

    rps = (pwm / 255) * max_rps
    speed = rps * wheel_circumference  # cm/sec
    distance += speed * dt
    time += dt


    distance_list.append(distance)
    time_list.append(time)

# Plotting
plt.plot(time_list, distance_list)
plt.xlabel('Time (s)')
plt.ylabel('Distance (cm)')
plt.title('Robot Distance vs Time')
plt.grid(True)
plt.show()
