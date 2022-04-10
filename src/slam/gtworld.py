import numpy as np
from gtgraph import Graph, RangeMeasurement
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


def transform_matrix_from_tf_msg(msg):
    """
    Credit for base code: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/ 
    """
    # Extract the values from Q
    q0 = msg.transform.rotation.x
    q1 = msg.transform.rotation.y
    q2 = msg.transform.rotation.z
    q3 = msg.transform.rotation.w
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    matrix = np.array([[r00, r01, r02, msg.transform.translation.x],
                       [r10, r11, r12, msg.transform.translation.y],
                       [r20, r21, r22, msg.transform.translation.z],
                       [  0,   0,   0, 1]])
                            
    return matrix

# ==========================
# RUNNING THE WORLD (girls).
# ==========================
if __name__ == "__main__":
    bagfile_path = "/home/yoraish/Desktop/yorai/data/bagfiles/20-74-production_2022-04-07-01-00-05.bag"
    beacons_yaml_path = '/indoorRobotics/indoor_maps/production/20/beacons.yaml'

    # Keep the recent odom in map transform for computing initial estimates. map -> odom -> base_footprint.
    recent_odom_in_map = None

    # Keep points from the mission location (EKF) estimates.
    ekf_basefootprint_in_map = []

    g = Graph()
    g.map_path = "/indoorRobotics/indoor_maps/staging/20/rviz_map.pgm"
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
                g.add_edge(beacon_num, beacon_num, 0, 0, 0, [0,0,0])
                print("Adding empty space for skipped beacon at %d." % beacon_num)
            beacon_num += 1
            print("Done adding empty. beacon_num at %d" % beacon_num)
        else:
            beacon_num = new_beacon_num

        x, y, _, t, _ = beacons_data[beacon_key]
        t = deg_to_rad(t)
        print(beacon_num, x,y,t)

        g.add_edge(beacon_num, beacon_num, x, y, t, [x,y,t])

    


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
    recent_sonar_right, recent_sonar_left = 0,0

    recent_odom_time = 0
    bag = rosbag.Bag(bagfile_path)
    for topic, msg, t in bag.read_messages(topics=['/indoor/monitor/location', '/indoor/beacon/pose', '/indoor/mission/status', '/tf', '/sonar_right', '/sonar_left']):

        if topic == "/indoor/monitor/location":
            ekf_basefootprint_in_map.append((msg.location.x, msg.location.y, msg.heading * 3.141592653 / 180))


        if topic == "/tf":
            if msg.transforms[0].child_frame_id == "odom" and msg.transforms[0].header.frame_id == "map":
                recent_odom_in_map = transform_matrix_from_tf_msg(msg.transforms[0])

            if msg.transforms[0].child_frame_id == "base_footprint" and msg.transforms[0].header.frame_id == "odom":
                # if t.to_sec() - recent_odom_time < 2:
                #     continue
                recent_odom_time = t.to_sec()
                x_msg = msg.transforms[0].transform.translation.x
                y_msg = msg.transforms[0].transform.translation.y
                t_msg = quat_to_rad(msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z, msg.transforms[0].transform.rotation.w)

                # If no landmark seen yet, do not add an odom pose.
                if odom_is_rooted == False:
                    recent_location_xyt = [x_msg, y_msg, t_msg] # In odom frame.
                    recent_location_time = t.to_sec()
                    # Only root the odometry measurements when a link to a landmark is formed. This happens when a landmark measurement is recorded less than 100ms before an odom measurement.
                    if t.to_sec() - recent_landmark_time < 0.01:
                        # Add a new pose with an edge to this landmark. The new pose gets the ix recent_pose_ix + 1
                        j = recent_pose_ix + 1
                        recent_pose_ix = j
                        recent_location_time = t.to_sec()
                        odom_is_rooted = True

                        # Transform the message pose (in odom frame) to be in map frame.
                        basefootprint_in_odom = transform_matrix_from_tf_msg(msg.transforms[0])
                        basefootprint_in_map = recent_odom_in_map.dot(basefootprint_in_odom)
                        prior_xyt = [basefootprint_in_map[0][3], basefootprint_in_map[1][3], np.arctan2(basefootprint_in_map[2][1],basefootprint_in_map[1][1])]

                        g.add_edge(lix, j, ldx, ldy, ldt, prior_xyt)
                        print("Rooting odom.")

                # If we have previous landmark positions and previous poses, then add an edge between the previous location and this new one.
                else:
                    i = recent_pose_ix
                    j = recent_pose_ix + 1
                    recent_pose_ix = j
                    recent_location_time = t.to_sec()
                    dx, dy, dt = g.relative_transform(*(recent_location_xyt + [x_msg, y_msg, t_msg]))

                    # Transform the message pose (in odom frame) to be in map frame.
                    basefootprint_in_odom = transform_matrix_from_tf_msg(msg.transforms[0])
                    basefootprint_in_map = recent_odom_in_map.dot(basefootprint_in_odom)
                    prior_xyt = [basefootprint_in_map[0][3], basefootprint_in_map[1][3], np.arctan2(basefootprint_in_map[2][1],basefootprint_in_map[1][1])]

                    g.add_edge(i, j, dx, dy, dt, prior_xyt)
                    recent_location_xyt = [x_msg, y_msg, t_msg]

                    # Add a measurement for node j.
                    m = RangeMeasurement(recent_sonar_right, recent_sonar_left)
                    g.add_measurement(j, m)


                    if t.to_sec() - recent_landmark_time < 0.01:
                        # Add a new pose with an edge to this landmark. The new pose gets the ix recent_pose_ix + 1
                        g.add_edge(lix, j, ldx, ldy, ldt, prior_xyt)

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
            # if msg.header.frame_id != "beacon0" and msg.header.frame_id != "beacon1":
            # Transform from beacon to robot.
            ldx = msg.pose.pose.position.x
            ldy = msg.pose.pose.position.y
            ldt = quat_to_rad(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

            recent_landmark_time = t.to_sec()
            lix = int(msg.header.frame_id[len("beacon"):])

        if topic == "/sonar_right":
            if msg.range > 0:
                recent_sonar_right = msg.range
            else:
                recent_sonar_right = 0

        if topic == "/sonar_left":
            if msg.range > 0:
                recent_sonar_left = msg.range
            else: 
                recent_sonar_left = 0

        ###################### STUPID TESTS ####################
        # if len(g.edges) > 600:# 598:
        #     break
            # # Add one artificial beacon measurement that is just fixing the last node in place.
            # j = recent_pose_ix 
            # ldx = 0
            # ldy = 0
            # ldt = 0
            # lix = 6
            # g.add_edge(lix, j, ldx, ldy, ldt)
            # break
        ###################### END STUPID TESTS ####################

    bag.close()

    print("Starting optimization on #edges=", len(g.edges))
    g.optimize()
    print("Starting visualization.")
    g.visualize(vis_odom = False, additional_points=ekf_basefootprint_in_map, title_text=bagfile_path.split("/")[-1])
