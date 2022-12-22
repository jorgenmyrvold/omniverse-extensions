import numpy as np
import matplotlib.pyplot as plt

# Set the positions and orientations of the poses
positions = [[0, 0], [1, 0], [1, 1], [0, 1]]
orientations = [np.pi / 2, np.pi, 3 * np.pi / 2, 0]

# Create the figure and the subplot
fig, ax = plt.subplots()

# Loop over the poses and plot them
for i, (position, orientation) in enumerate(zip(positions, orientations)):
    # Create a unit vector in the direction of the orientation
    dx, dy = np.cos(orientation), np.sin(orientation)

    # Plot the position of the pose as a marker
    ax.scatter(position[0], position[1], c='r')

    # Plot the orientation of the pose as an arrow
    ax.quiver(position[0], position[1], dx, dy, width=0.005, color='b')

# Show the plot
plt.show()
