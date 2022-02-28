# Just like the world, but fake.
# A sanity check example.

import numpy as np
from gtgraph import Graph
import yaml


def deg_to_rad(deg):
    rad = deg/360.0*2*np.pi
    rad = ((rad + np.pi) % (2 * np.pi) ) - np.pi
    return rad

def quat_to_rad(x,y,z,w):
    import math
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw


# Set up our landmarks.
landmarks = []
landmarks.append([0,0,0])
landmarks.append([1,0,0])
landmarks.append([2,0,0])
landmarks.append([1.2,0.5,-np.pi])

g = Graph()
recent_j = -1
for landmark in landmarks:
    recent_j += 1
    g.add_edge(recent_j,recent_j, landmark[0],landmark[1],landmark[2], 
                prior_xyt=[landmark[0],landmark[1],landmark[2]])

# Add some poses.
# Straight line between start and stop. Each are np.array(2x1).
start = np.array([[0], [-0.1]])
t = 0.1 # Radians.
length = 2
step = 0.1

recent_xy = start
recent_t = t

# Connect the first odom to a landmark.
dx, dy, dt = g.relative_transform(landmarks[0][0],landmarks[0][1],landmarks[0][2], recent_xy[0][0],recent_xy[1][0], recent_t)
recent_j += 1
j = recent_j

g.add_edge(0, j, dx, dy, dt, prior_xyt=[dx, dy, dt])# TODO this is only true if the first landmark is at 000.

for num in range(int(length/step)):
    # Change in local robot frame.
    if num >= 15:
        ut = -0.3
    else:
        ut = 0
    ux = step
    uy = step/3
    new_t = recent_t + ut
    new_xy = recent_xy + np.array([[np.cos(new_t), -np.sin(new_t)], [np.sin(new_t), np.cos(new_t)]]).dot(np.array([[ux, uy]]).T)
    dx, dy, dt = g.relative_transform(recent_xy[0][0], recent_xy[1][0], recent_t, new_xy[0][0], new_xy[1][0], new_t)
    i = recent_j
    j = recent_j + 1
    recent_j = j

    recent_xy = new_xy
    recent_t = new_t 

    g.add_edge(i,j, dx + (np.random.rand()-0.5)/5,
                    dy + (np.random.rand()-0.5)/5, 
                    dt + (np.random.rand()-0.5)/5, 
                    [recent_xy[0][0],recent_xy[1][0], recent_t])

    # If close to a landmark, draw an edge.
    for lix, l in enumerate(landmarks):
        if np.linalg.norm(new_xy - np.array([l[:2]]).T) <= 0.4:
            ldx, ldy, ldt = g.relative_transform(l[0],l[1],l[2], recent_xy[0][0],recent_xy[1][0], recent_t)

            print(lix, "-->", j, " with tf ", ldx, ldy, ldt)
            g.add_edge(lix, j, ldx, ldy, ldt, [recent_xy[0][0],recent_xy[1][0], recent_t]) # Edge fom landmark.
            # g.add_edge(j, j, l[0] + ldx, l[1] + ldy, l[2] + ldt) # Assignment of odom node..

            break

  


g.optimize()
g.visualize() 

    
    



