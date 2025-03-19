from math import *
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Landmarks in the environment
landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0  # Size of the environment

# Robot class
class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)

    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z

    def move(self, turn, forward):
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size
        y %= world_size
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def Gaussian(self, mu, sigma, x):
        return exp(-((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):
        prob = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))


# Create a robot
myrobot = robot()
myrobot.set_noise(0.05, 0.05, 5.0)  # Set motion and sensor noise

# Create a set of particles
N = 1000  # Number of particles
p = [robot() for _ in range(N)]
for particle in p:
    particle.set_noise(0.05, 0.05, 5.0)

# Initialize the plot
fig, ax = plt.subplots()
ax.set_xlim(0, world_size)
ax.set_ylim(0, world_size)
ax.set_xlabel('X')
ax.set_ylabel('Y')

# Plot landmarks
landmark_x = [landmark[0] for landmark in landmarks]
landmark_y = [landmark[1] for landmark in landmarks]
landmark_plot = ax.scatter(landmark_x, landmark_y, s=100, c='green', marker='s', label='Landmarks')

# Initialize scatter plots for particles and robot
particle_plot = ax.scatter([], [], s=5, c='blue', label='Particles')
robot_plot = ax.scatter([], [], s=100, c='red', marker='x', label='Robot')

# Add legend
ax.legend()

# Function to update the animation
def update(frame):
    global myrobot, p

    # Move the robot and particles
    myrobot = myrobot.move(0.1, 5.0)
    p = [particle.move(0.1, 5.0) for particle in p]

    # Robot senses the environment
    Z = myrobot.sense()

    # Update particle weights
    w = [particle.measurement_prob(Z) for particle in p]

    # Resample particles
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for _ in range(N):
        beta += random.random() * 3.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    p = p3

    # Extract particle positions
    particle_x = [particle.x for particle in p]
    particle_y = [particle.y for particle in p]

    # Update scatter plots
    particle_plot.set_offsets(list(zip(particle_x, particle_y)))
    robot_plot.set_offsets([[myrobot.x, myrobot.y]])

    return particle_plot, robot_plot

# Create the animation
ani = FuncAnimation(fig, update, frames=range(50), interval=200, blit=True)

# Save the animation as a video (optional)
# ani.save('mcl_animation.mp4', writer='ffmpeg')

# Show the animation
plt.show()