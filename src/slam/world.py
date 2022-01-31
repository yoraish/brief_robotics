from cv2 import sort
import numpy as np
from graph import Graph
import yaml
import rosbag


def relative_transform(xi, yi, ti, xj, yj, tj):
    # Returns the pose of j in the frame of i.
    R = np.array([[np.cos(ti), -np.sin(ti)], 
                  [np.sin(ti), np.cos(ti)]])
    shifted_j = np.array([[xj], [yj]]) - np.array([[xi], [yi]])
    j_in_i = R.T.dot(shifted_j)
    # print(xi, yi, ti, xj, yj, tj)
    # print("-->", j_in_i[0][0], j_in_i[1][0], tj - ti)
    # import matplotlib.pyplot as plt
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.arrow(xi, yi, 0.1 * np.cos(ti), 0.1*np.sin(ti), color = 'b')
    # ax.arrow(xj, yj, 0.1 * np.cos(tj), 0.1*np.sin(tj), color = 'r')
    # ax.scatter([xi,xj], [yi, yj])
    # ax.set_aspect("equal")
    # plt.show()
    return j_in_i[0][0], j_in_i[1][0], tj - ti

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

'''
g = Graph()
# Add landmarks.
g.add_edge(0, 0, 15, 15, np.pi/2.)
g.add_edge(1, 1, 11, 11, np.pi/2.)
g.add_edge(2, 2, 12, 12, np.pi/2.)
g.add_edge(3, 3, 1, 1, 0)

# Add odometry measurements and landmark observations.
g.add_edge(3, 4, 3, 0, 0.2)
g.add_edge(4, 5, 3, 0, 0.2)
g.add_edge(5, 6, 3, 0, 0.2)
g.add_edge(6, 7, 3, 0, 0.2)
g.add_edge(0, 7, -1, 0, 0)


# # If this is not an assignment edge, then we should compute the j pose in the i frame.
# if i != j:
#     dx, dy, dt = self.get_relative_transform()

g.optimize()
g.visualize()
'''

# ==========================
# RUNNING THE WORLD (girls).
# ==========================
if __name__ == "__main__":
    bagfile_path = "/home/yoraish/Desktop/yorai/data/bagfiles/28-135-production_2022-01-28-09-23-57.bag"
    beacons_yaml_path = '/indoorRobotics/indoor_maps/production/28/beacons.yaml'

    g = Graph()
    # Start with getting the beacons of the zone.
    with open (beacons_yaml_path , "r") as stream:
        try:
            beacons_data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # Sort the beacons by name.
    sorted_beacons = [beacon_key for beacon_key in beacons_data.keys()]
    sorted_beacons.sort( key = lambda x: int(x.split("_")[-1]))
    beacon_num = -1
    for beacon_key in sorted_beacons:
        new_beacon_num = int(beacon_key.split("_")[-1])

        if new_beacon_num > beacon_num + 1:
            while new_beacon_num > beacon_num + 1:
                beacon_num += 1
                g.add_edge(beacon_num, beacon_num, 0, 0, 0)
                print("Adding empty space for skipped beacon at %d." % beacon_num)
            beacon_num += 1
            print("Done adding empty. beacon_num at %d" % beacon_num)
        else:
            beacon_num = new_beacon_num

        x, y, _, t, _ = beacons_data[beacon_key]
        t = deg_to_rad(t)
        print(beacon_num, x,y,t)

        g.add_edge(beacon_num, beacon_num, x, y, t)

    


    # Get poses. 
    # Whenever we get a beacon sighting, if the most recent location pose was received less than 100ms ago, we connect an edge from the landmark to the pose.
    recent_location_xyt = None
    recent_location_time = None
    recent_pose_ix = g.edges[-1].j
    odom_is_rooted = False # We require one landmark observation to be added before adding odom.

    # Landmark recently around.
    ldx = None
    ldy = None
    ldt = None
    recent_landmark_time = 0
    lix = None

    bag = rosbag.Bag(bagfile_path)
    for topic, msg, t in bag.read_messages(topics=['/indoor/monitor/location', '/indoor/beacon/pose', '/indoor/mission/status']):

        if topic == "/indoor/monitor/location":

            # If no landmark seen yet, do not add an odom pose.
            if odom_is_rooted == False:
                recent_location_xyt = [msg.location.x, msg.location.y, deg_to_rad(msg.heading)]
                recent_location_time = t.to_sec()
                # Only root the odometry measurements when a link to a landmark is formed. This happens when a landmark measurement is recorded less than 100ms before an odom measurement.
                if t.to_sec() - recent_landmark_time < 0.1:
                    # Add a new pose with an edge to this landmark. The new pose gets the ix recent_pose_ix + 1
                    j = recent_pose_ix + 1
                    recent_pose_ix = j
                    recent_location_time = t.to_sec()
                    odom_is_rooted = True
                    g.add_edge(lix, j, ldx, ldy, ldt, [msg.location.x, msg.location.y, deg_to_rad(msg.heading)])

            # If we have previous landmark positions and previous poses, then add an edge between the previous location and this new one.
            else:
                i = recent_pose_ix
                j = recent_pose_ix + 1
                recent_pose_ix = j
                recent_location_time = t.to_sec()
                dx, dy, dt = relative_transform(*(recent_location_xyt + [msg.location.x, msg.location.y, deg_to_rad(msg.heading)]))
                g.add_edge(i, j, dx, dy, dt, [msg.location.x, msg.location.y, deg_to_rad(msg.heading)])
                recent_location_xyt = [msg.location.x, msg.location.y, deg_to_rad(msg.heading)]

                if t.to_sec() - recent_landmark_time < 0.01:
                    # Add a new pose with an edge to this landmark. The new pose gets the ix recent_pose_ix + 1
                    g.add_edge(lix, j, ldx, ldy, ldt, [msg.location.x, msg.location.y, deg_to_rad(msg.heading)])

            
        if topic == "/indoor/beacon/pose" and odom_is_rooted == False:
            # Transform from beacon to robot.
            ldx = msg.pose.pose.position.x
            ldy = msg.pose.pose.position.y
            ldt = quat_to_rad(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            recent_landmark_time = t.to_sec()
            lix = int(msg.header.frame_id[len("beacon"):])

            # # If the most recent location was seen less than 100ms ago, then create an edge between this beacon and that most recent location.
            # if recent_location_xyt is not None:
            #     if t.to_sec() - recent_location_time < 0.1:
            #         odom_is_rooted = True

        if len(g.edges) > 26:
            break
    bag.close()

    g.optimize()
    g.visualize()