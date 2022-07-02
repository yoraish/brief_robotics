import gtsam
import numpy as np
from gtsam.symbol_shorthand import L, X
from trimesh import transform_points


class GTEdge():
    def __init__(self, i, j, xj, yj, tj, noise):
        self.i = i
        self.j = j
        # The following are in i frame.
        self.x = xj 
        self.y = yj
        self.t = tj
        self.noise = noise
    
    def __str__(self):
        return "i: %d, j: %d x: %d y: %d t: %d cost: %d" %(self.i, self.j, self.x, self.y, self.t, self.noise)

class RangeMeasurement():
    def __init__(self, r, l):
        self.right = r
        self.left = l
    
    def __str__(self):
        return "Measurement Left: %d, Right: %d" %(self.left, self.right)

class Graph():

    def __init__(self):
        
        # Noise.
        self.ODOM_NOISE =     gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
        self.LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.03, 0.03, 0.03]))
        self.PRIOR_NOISE =    gtsam.noiseModel.Diagonal.Sigmas(np.array([0.00003, 0.00003, 0.00001]))

        # Landmarks.
        self.landmarks = set()

        # Measurements. Mapping nodes to RangeMeasurement's/
        self.measurements = {}
        
        # Keep track of xy edges and build matrix C only after optimizing for angles.
        self.edges = []

        # Should we optimize after every edge addition?
        self.online = False

        # Optimization.
        self.graph = gtsam.NonlinearFactorGraph()
        self.graph_keys = {} # Maps node id to gtsam symbol.
        self.initial_estimate = gtsam.Values()
        self.prior_xyt = {} # Node id : (x,y,t)

        # Optimized poses.
        self.x_hat, self.t_hat = None, None

        # Visualization.
        self.arrow_length = 0.05
        self.arrow_width = 0.001
        self.map_path =  ""
        self.map_resolution = 0.05 # Meters per pixel.
        self.map_width_pixels  = 0
        self.map_height_pixels = 0

    def normalize_yaw(self, yaw):
        return ( yaw + np.pi ) % (2 * np.pi) - np.pi

    def relative_transform(self, xi, yi, ti, xj, yj, tj):
        # Returns the pose of j in the frame of i.
        R = np.array([[np.cos(ti), -np.sin(ti)], 
                    [np.sin(ti), np.cos(ti)]])
        shifted_j = np.array([[xj], [yj]]) - np.array([[xi], [yi]])
        j_in_i = R.T.dot(shifted_j)
        return j_in_i[0][0], j_in_i[1][0], tj - ti

    def add_measurement(self, j, m):
        # Adds a measurement (RangeMeasurement object) that was observed at node j.
        self.measurements[j] = m

    def add_edge(self, i, j, dx, dy, dt, prior_xyt):

        """Add an edge to the pose graph.

        Args:
            xi_ix (int): the index of the previous node. Edge is from x_i to x_j, all quantities in the reference frame of x_i. If i == j then this is an assignment, and the x,y,t quantities are in the map frame.
            dx (float): x axis displacement between x_i.x to this new observation.
            dy (float): y axis displacement between x_i.y to this new obserevation.
            dt (float): angle radians change between x_i.t to this new observation. In range [-pi, +pi].
        """
        # Decide what noise to use.
        if i == j:
            noise = self.PRIOR_NOISE
            self.landmarks.add(j)
        elif i in self.landmarks:
            noise = self.LANDMARK_NOISE
        else:
            noise = self.ODOM_NOISE

        # Record edge.
        self.edges.append(GTEdge(i, j, dx, dy, dt, noise))

        # Add a visualization prior.
        self.prior_xyt[j] = prior_xyt


    def optimize(self):
        # Build the angles matrix and vector.
        # Initialize.
        if self.edges[0].i != self.edges[0].j:
            raise RuntimeError("First edge should be an assignment edge, not a cross-node edge.")
        
        # First, create all the graph keys.
        for k in range(0, len(self.edges)):
            edge = self.edges[k] # From i to j. xyt are for j and in i frame. If i==j then xyt are in the map frame.
            if edge.i == edge.j:
                self.graph_keys[edge.j] = L(edge.j)
            else:
                self.graph_keys[edge.j] = X(edge.j)
                
        # Second, add the priors for the poses to the graph.
        for j, (x,y,t) in self.prior_xyt.items():
            self.initial_estimate.insert(self.graph_keys[j], gtsam.Pose2(x,y,t))


        # Third, add the odometry and landmark edges.
        for k in range(0, len(self.edges)):
            edge = self.edges[k] # From i to j. xyt are for j and in i frame. If i==j then xyt are in the map frame.

            # If this edge is an assignment (of a landmark), add it with a priorfactor (no self edges here).
            if edge.i == edge.j:
                self.graph.add(
                    gtsam.PriorFactorPose2(self.graph_keys[edge.j], gtsam.Pose2(edge.x, edge.y, edge.t), edge.noise))
            
            # Otherwise, add odom/landmark edges.
            else:
                self.graph.add(
                    gtsam.BetweenFactorPose2(self.graph_keys[edge.i], self.graph_keys[edge.j], gtsam.Pose2(edge.x, edge.y, edge.t), edge.noise))


        # Print graph
        print("Factor Graph:\n{}".format(self.graph))
        # Print
        print("Initial Estimate:\n{}".format(self.initial_estimate))
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate,
                                                    params)
        result = optimizer.optimize()
        print("\nFinal Result:\n{}".format(result))
        print(result.atPose2(self.graph_keys[0]))

        self.x_hat = [ [ result.atPose2(self.graph_keys[j]).x(), result.atPose2(self.graph_keys[j]).y() ] for j in range(len(self.prior_xyt))]
        self.x_hat = [[item] for sublist in self.x_hat for item in sublist]
        self.t_hat = [ [result.atPose2(self.graph_keys[j]).theta() ] for j in range(len(self.prior_xyt))]
        return self.x_hat, self.t_hat

    def transform_measurements(self, poses):
        # With poses being a list of (x,y,t) and observations being a map between index j (in poses): RangeMeasurement object, return the map coordinates of the measurements.
        transformed_measurements = []
        for j in poses:
            # Add measurement to maps figure.
            if j in self.measurements:
                if self.measurements[j].right != 0:
                    # Transformation for j in map frame.
                    j_in_map = np.array([[np.cos(poses[j][2]), -np.sin(poses[j][2]), poses[j][0]], 
                                         [np.sin(poses[j][2]),  np.cos(poses[j][2]), poses[j][1]],
                                         [0                  , 0                   , 1]])


                    right_in_j_homo = np.array([[0, self.measurements[j].right, 1]]).T
                    left_in_j_homo  = np.array([[0, -self.measurements[j].left, 1]]).T
                    right_in_map = j_in_map.dot(right_in_j_homo)
                    left_in_map = j_in_map.dot(left_in_j_homo)
                    # Append (x,y) tuples to rights/lefts depending on the origin of the point.
                    transformed_measurements.append([right_in_map[0][0], right_in_map[1][0]])
                    transformed_measurements.append([left_in_map[0][0], left_in_map[1][0]])
                    # ax_maps.scatter([right_in_map[0][0]], [right_in_map[1][0]], color = c)
                    # ax_maps.scatter([left_in_map[0][0]], [left_in_map[1][0]], color = c)
        return transformed_measurements

    def visualize(self, vis_odom = True, additional_points = [], title_text = ""): # Additional points is a list of tuples (x,y) or (x,y,t).
        import matplotlib.pyplot as plt
        fig = plt.figure(0)
        ax = fig.add_subplot(111)

        # Also create figures for the generated maps.
        fig_maps = plt.figure(1)
        ax_maps = fig_maps.add_subplot(111)

        # Store tof points from right/left for map visualization.
        rights, lefts = [], []

        # Containers for the odom/optimized points in another form. Mostly for visualizing maps. This includes landmarks.
        odom_poses = {} # j:(x,y,t)
        gt_poses = {} # j:(x,y,t)

        for edge in self.edges:
            i, j, dx, dy, dt = edge.i, edge.j, edge.x, edge.y, edge.t

            # If this is an assignment edge (self-edge), just draw it.
            if i == j:
                ax.arrow(dx, dy, self.arrow_length * np.cos(dt), self.arrow_length * np.sin(dt), width=self.arrow_width, color = "b", alpha = 0.2)
                odom_poses[j] = (dx, dy, dt)
                ax.scatter(dx, dy)
                ax.text(dx, dy, "beacon"+str(i))
                

            else:
                # Take the pose of i and transform it to j. If i does not exist, disregard it. This means that the plot will only start after seeing at least one landmark. At indoor this is okay since the tile is a landmark.
                if i not in odom_poses.keys():
                    print("[VIS] Skipping i", i, "j", j, "since there is still no i.")
                    continue
                if i in self.landmarks:
                    c = 'b'
                else:
                    c = 'y'

                # If already have an estimate for j, draw that one.
                if j in odom_poses.keys():
                    if vis_odom:
                        ax.plot([odom_poses[i][0] ,odom_poses[j][0]], [odom_poses[i][1], odom_poses[j][1]], color = c, alpha = 0.2)
                    continue
                
                # If no estimate for j, compute from the relative transform from i.
                else:
                    R = np.array([[np.cos(odom_poses[i][2]), -np.sin(odom_poses[i][2])], 
                                [np.sin(odom_poses[i][2]), np.cos(odom_poses[i][2])]])

                    trans_j = np.array([[odom_poses[i][0]], [odom_poses[i][1]]]) + R.dot(np.array([[dx], [dy]]))
                    rot_j = odom_poses[i][2] + dt
                    odom_poses[j] = (trans_j[0][0], trans_j[1][0], rot_j)

                    # Draw in red and if i is a landmark then draw blue line the landmark.
                    if vis_odom:
                        ax.arrow(trans_j[0][0], trans_j[1][0], self.arrow_length * np.cos(rot_j), self.arrow_length * np.sin(rot_j), width=self.arrow_width, color = "y", alpha = 0.2)

                    if vis_odom:
                        ax.plot([odom_poses[i][0] ,trans_j[0][0]], [odom_poses[i][1], trans_j[1][0]], color = c, alpha = 0.2, label = "Odometry")

                    # Draw prior if it exists.
                    '''if j in self.prior_xyt:
                        x,y,t = self.prior_xyt[j]
                        ax.arrow(x, y, self.arrow_length * np.cos(t), self.arrow_length * np.sin(t), width=self.arrow_width, color = "g", alpha = 0.2)
                        ax.scatter([x],[y])'''

                    # # Add measurement to maps figure.
                    # if j in self.measurements:
                    #     if self.measurements[j].right != 0:
                    #         # Transformation for j in map frame.
                    #         j_in_map = np.zeros((3,3))
                    #         j_in_map[2,2] = 1
                    #         j_in_map[:2, :2] = R
                    #         j_in_map[:2,2:3] = trans_j    
                    
                    #         right_in_j_homo = np.array([[self.measurements[j].right, 0, 1]]).T
                    #         left_in_j_homo  = np.array([[-self.measurements[j].left, 0, 1]]).T
                    #         right_in_map = j_in_map.dot(right_in_j_homo)
                    #         left_in_map = j_in_map.dot(left_in_j_homo)
                    #         # Append (x,y) tuples to rights/lefts depending on the origin of the point.
                    #         rights.append([right_in_map[0][0], right_in_map[1][0]])
                    #         lefts.append([left_in_map[0][0], left_in_map[1][0]])
                            # ax_maps.scatter([right_in_map[0][0]], [right_in_map[1][0]], color = c)
                            # ax_maps.scatter([left_in_map[0][0]], [left_in_map[1][0]], color = c)
        
        odom_measurements_in_map = self.transform_measurements(odom_poses)
        ax_maps.scatter([p[0] for p in odom_measurements_in_map], [p[1] for p in odom_measurements_in_map], color = 'k')
        # ax_maps.scatter([p[0] for p in lefts], [p[1] for p in lefts], color = c)


        ax.set_aspect("equal")
        ax_maps.set_aspect("equal")
    


        # Show the optimized points.
        # Check if optimization has already happened.
        if self.x_hat is not None:
            for i in range(len(self.t_hat)):
                if i in self.landmarks:
                    c = "b"
                else:
                    c = "r"
                ax.arrow(self.x_hat[2*i][0], self.x_hat[2*i + 1][0], self.arrow_length * np.cos(self.t_hat[i][0]), self.arrow_length * np.sin(self.t_hat[i][0]), width=self.arrow_width, color = c)
                
            # Go through all the edges and draw some lines (red between poses and blue for poses-to-landmarks.)
            for edge in self.edges:
                i, j = edge.i, edge.j
                if i in self.landmarks:
                    c = 'b'
                else:
                    c = 'r'
                    
                ax.plot([self.x_hat[2*i][0] ,self.x_hat[2*j][0]], [self.x_hat[2*i + 1][0], self.x_hat[2*j + 1][0]], color = c, label="Smoothed")
                gt_poses[i] = (self.x_hat[2*i][0], self.x_hat[2*i + 1][0], self.t_hat[i][0])

                # Draw a line between the original and the optimized poses.
                # ax.plot( [self.x_hat[2*j][0] ,poses[j][0]], [self.x_hat[2*j + 1][0], poses[j][1]], color = 'k', linestyle = "dashdot", alpha = 0.2)
        
            # Draw the map from the optimized points.
            gt_measurements_in_map = self.transform_measurements(gt_poses)
            ax_maps.scatter([p[0] for p in gt_measurements_in_map], [p[1] for p in gt_measurements_in_map], color = c)


        # Show the additional points, if those exist.
        if additional_points:
            ax.plot([x[0] for x in additional_points], [x[1] for x in additional_points], c = 'k', alpha = 0.2)
            # ax.scatter([x[0] for x in additional_points], [x[1] for x in additional_points], c = 'k')
            for x in additional_points:
                ax.arrow(x[0], 
                         x[1], 
                         self.arrow_length * np.cos(x[2]), 
                         self.arrow_length * np.sin(x[2]), 
                         width=self.arrow_width, color = "k", alpha = 0.2)

        # ax.legend()
        # ax.set_title("""self.ODOM_NOISE = %d
        #                 self.LANDMARK_NOISE = %d
        #                 self.PRIOR_NOISE = %d
        #                 %s""" % (int(self.ODOM_NOISE), int(self.LANDMARK_NOISE), int(self.PRIOR_NOISE), title_text))
        ax.set_title(title_text)

        # Visualize the measurements on the map.
        fig_rviz = plt.figure(2)
        ax_rviz = fig_rviz.add_subplot(111)
        
        # Get the map as an array.
        map_array_pixels = self.read_pgm(self.map_path)
        # self.ax_add_map(ax_rviz, self.map_path)
        for pose in gt_measurements_in_map:
            x_pixels, y_pixels = self.world_to_map(*pose[:2])
            # ax_rviz.scatter([x_pixels], [y_pixels], color = 'r')
            try:
                map_array_pixels[int(y_pixels)][int(x_pixels)] = 150
            except IndexError:
                continue

        for pose in odom_measurements_in_map:
            x_pixels, y_pixels = self.world_to_map(*pose[:2])
            try:
                map_array_pixels[int(y_pixels)][int(x_pixels)] = 200
            except IndexError:
                continue
        ax_rviz.imshow(map_array_pixels)#, cmap="gray")

        plt.show()


   
    def read_pgm(self, path):
        raster = []
        f = open(path, "rb")
        f.readline()
        f.readline()
        (width, height) = [int(i) for i in f.readline().split()]
        depth = int(f.readline())
        assert depth <= 255
        cellX = width
        cellY = height
        self.map_width_pixels = width
        self.map_height_pixels = height

        for y in range(height):
            row = []
            for x in range(width):
                row.append(ord(f.read(1)))
            raster.append(row)
        f.close()

        grey = np.array(raster)
        grey[grey != 0] = 255
        return grey

    def world_to_map(self, x, y):
        return (x / self.map_resolution + self.map_width_pixels/2, self.map_height_pixels/2 - y/self.map_resolution)

    def ax_add_map(self, ax, map_path):
        self.map_array_pixels = self.read_pgm(self.map_path)
        ax.imshow(self.map_array_pixels, cmap="gray")
        return ax