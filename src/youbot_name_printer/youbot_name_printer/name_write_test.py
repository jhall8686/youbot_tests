# Preview the waypoints in matplotlib before sending to the robot
import matplotlib.pyplot as plt
import numpy as np
from youbot_name_printer.name_to_waypoints import name_to_waypoints

waypoints = name_to_waypoints(
    name="Jack",
    z_write=150.0,     # mm — pen touching surface
    z_travel=180.0,    # mm — pen lifted between strokes
    x_offset=200.0,    # mm — adjust to center in your workspace
    y_offset=0.0,
    scale=30.0,        # mm per font unit — tune this
    n_points=200,
)



wps = np.array(waypoints)
fig, ax = plt.subplots()
# Color by z height — blue = pen up, red = pen down
colors = ['red' if w[2] == 150.0 else 'blue' for w in waypoints]
ax.scatter(wps[:,0], wps[:,1], c=colors, s=2)
ax.set_aspect('equal')
plt.show()