from cv2 import sort
import numpy as np
from graph import Graph
import yaml
import rosbag


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
    bagfile_path = "/home/yoraish/Desktop/yorai/data/bagfiles/20-74-production_2022-02-10-22-29-11.bag"
    beacons_yaml_path = '/indoorRobotics/indoor_maps/production/20/beacons.yaml'

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

    recent_odom_time = 0
    bag = rosbag.Bag(bagfile_path)
    for topic, msg, t in bag.read_messages(topics=['/indoor/monitor/location', '/indoor/beacon/pose', '/indoor/mission/status', '/tf']):

        # if topic == "/indoor/monitor/location":

        if topic == "/tf":
            if msg.transforms[0].child_frame_id == "base_footprint" and msg.transforms[0].header.frame_id == "odom":
                # if t.to_sec() - recent_odom_time < 2:
                #     continue
                recent_odom_time = t.to_sec()
                x_msg = msg.transforms[0].transform.translation.x
                y_msg = msg.transforms[0].transform.translation.y
                t_msg = quat_to_rad(msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w)

                # If no landmark seen yet, do not add an odom pose.
                if odom_is_rooted == False:
                    recent_location_xyt = [x_msg, y_msg, t_msg]
                    recent_location_time = t.to_sec()
                    # Only root the odometry measurements when a link to a landmark is formed. This happens when a landmark measurement is recorded less than 100ms before an odom measurement.
                    if t.to_sec() - recent_landmark_time < 0.01:
                        # Add a new pose with an edge to this landmark. The new pose gets the ix recent_pose_ix + 1
                        j = recent_pose_ix + 1
                        recent_pose_ix = j
                        recent_location_time = t.to_sec()
                        odom_is_rooted = True
                        g.add_edge(lix, j, ldx, ldy, ldt, [x_msg, y_msg, t_msg])
                        print("Rooting odom.")

                # If we have previous landmark positions and previous poses, then add an edge between the previous location and this new one.
                else:
                    i = recent_pose_ix
                    j = recent_pose_ix + 1
                    recent_pose_ix = j
                    recent_location_time = t.to_sec()
                    dx, dy, dt = g.relative_transform(*(recent_location_xyt + [x_msg, y_msg, t_msg]))
                    print("dt", dt)
                    g.add_edge(i, j, dx, dy, dt, [x_msg, y_msg, t_msg])
                    recent_location_xyt = [x_msg, y_msg, t_msg]


                    if t.to_sec() - recent_landmark_time < 0.01:
                        # Add a new pose with an edge to this landmark. The new pose gets the ix recent_pose_ix + 1
                        g.add_edge(lix, j, ldx, ldy, ldt, [x_msg, y_msg, t_msg])

                        # Remove this landmark from being considered again.
                        recent_landmark_time = 0


                        #######
                        """""
                        R = np.array([[np.cos(g.edges[lix].t), -np.sin(g.edges[lix].t)], 
                                    [np.sin(g.edges[lix].t), np.cos(g.edges[lix].t)]])

                        world_trans_j = np.array([[ g.edges[lix].x ], [ g.edges[lix].y]]) + R.dot(np.array([[ldx], [ldy]]))
                        world_rot_j =  g.edges[lix].t + dt
                        g.add""_edge(j, j, world_trans_j[0][0], world_trans_j[1][0], world_rot_j)""" # Assignment of odom node.
                        ####


                
        if topic == "/indoor/beacon/pose":
            if msg.header.frame_id != "beacon0" and msg.header.frame_id != "beacon1":
                # Transform from beacon to robot.
                ldx = msg.pose.pose.position.x
                ldy = msg.pose.pose.position.y
                ldt = quat_to_rad(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                print("ldt", ldt)

                recent_landmark_time = t.to_sec()
                lix = int(msg.header.frame_id[len("beacon"):])
                print(msg.header.frame_id)

        ###################### STUPID TESTS ####################
        if len(g.edges) > 598:
            # Add one artificial beacon measurement that is just fixing the last node in place.
            j = recent_pose_ix 
            ldx = 0
            ldy = 0
            ldt = 0
            lix = 6
            g.add_edge(lix, j, ldx, ldy, ldt)
            break
        ###################### END STUPID TESTS ####################

    bag.close()

    print("Starting optimization on #edges=", len(g.edges))
    g.optimize()
    print("Starting visualization.")
    g.visualize()