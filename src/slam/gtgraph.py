import gtsam
import numpy as np
from gtsam.symbol_shorthand import L, X


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

class Graph():

    def __init__(self):
        
        # Noise.
        self.ODOM_NOISE =     gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
        self.LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.03, 0.03, 0.03]))
        self.PRIOR_NOISE =    gtsam.noiseModel.Diagonal.Sigmas(np.array([0.00003, 0.00003, 0.00001]))

        # Landmarks.
        self.landmarks = set()
        
        # Keep track of xy edges and build matrix C only after optimizing for angles.
        self.edges = []

        # Should we optimize after every edge addition?
        self.online = False

        # Optimization.
        self.graph = gtsam.NonlinearFactorGraph()
        self.graph_keys = {} # Maps node id to gtsam symbol.
        self.initial_estimate = gtsam.Values()


        # Optimized poses.
        self.x_hat, self.t_hat = None, None

        # Visualization.
        self.arrow_length = 0.05
        self.arrow_width = 0.001
        self.prior_xyt = {} # Node id : (x,y,t)


    def normalize_yaw(self, yaw):
        return ( yaw + np.pi ) % (2 * np.pi) - np.pi

    def relative_transform(self, xi, yi, ti, xj, yj, tj):
        # Returns the pose of j in the frame of i.
        R = np.array([[np.cos(ti), -np.sin(ti)], 
                    [np.sin(ti), np.cos(ti)]])
        shifted_j = np.array([[xj], [yj]]) - np.array([[xi], [yi]])
        j_in_i = R.T.dot(shifted_j)
        return j_in_i[0][0], j_in_i[1][0], tj - ti


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


    def visualize(self):
        import matplotlib.pyplot as plt
        fig = plt.figure(0)
        ax = fig.add_subplot(111)

        # Show the unoptimized poses.
        poses = {}
        for edge in self.edges:
            i, j, dx, dy, dt = edge.i, edge.j, edge.x, edge.y, edge.t

            # If this is an assignment edge (self-edge), just draw it.
            if i == j:
                ax.arrow(dx, dy, self.arrow_length * np.cos(dt), self.arrow_length * np.sin(dt), width=self.arrow_width, color = "b", alpha = 0.2)
                poses[j] = (dx, dy, dt)
                ax.scatter(dx, dy)

            else:
                # Take the pose of i and transform it to j. If i does not exist, disregard it. This means that the plot will only start after seeing at least one landmark. At indoor this is okay since the tile is a landmark.
                if i not in poses.keys():
                    print("[VIS] Skipping i", i, "j", j, "since there is still no i.")
                    continue
                if i in self.landmarks:
                    c = 'b'
                else:
                    c = 'r'

                # If already have an estimate for j, draw that one.
                if j in poses.keys():
                    ax.plot([poses[i][0] ,poses[j][0]], [poses[i][1], poses[j][1]], color = c, alpha = 0.2)
                    continue
                
                # If no estimate for j, compute from the relative transform form i.
                else:
                    R = np.array([[np.cos(poses[i][2]), -np.sin(poses[i][2])], 
                                [np.sin(poses[i][2]), np.cos(poses[i][2])]])

                    trans_j = np.array([[poses[i][0]], [poses[i][1]]]) + R.dot(np.array([[dx], [dy]]))
                    rot_j = poses[i][2] + dt
                    poses[j] = (trans_j[0][0], trans_j[1][0], rot_j)

                    # Draw in red and if i is a landmark then draw blue line the landmark.
                    ax.arrow(trans_j[0][0], trans_j[1][0], self.arrow_length * np.cos(rot_j), self.arrow_length * np.sin(rot_j), width=self.arrow_width, color = "r", alpha = 0.2)

                    ax.plot([poses[i][0] ,trans_j[0][0]], [poses[i][1], trans_j[1][0]], color = c, alpha = 0.2)

                    # Draw prior if it exists.
                    '''if j in self.prior_xyt:
                        x,y,t = self.prior_xyt[j]
                        ax.arrow(x, y, self.arrow_length * np.cos(t), self.arrow_length * np.sin(t), width=self.arrow_width, color = "g", alpha = 0.2)
                        ax.scatter([x],[y])'''
        ax.set_aspect("equal")
    


        # Show the optimized points.
        # Check if optimization has already happened.
        if self.x_hat is not None:
            for i in range(len(self.t_hat)):
                if i in self.landmarks:
                    c = "b"
                else:
                    c = "r"
                print(self.x_hat[2*i][0], self.x_hat[2*i+1][0], self.t_hat[i][0])
                ax.arrow(self.x_hat[2*i][0], self.x_hat[2*i + 1][0], self.arrow_length * np.cos(self.t_hat[i][0]), self.arrow_length * np.sin(self.t_hat[i][0]), width=self.arrow_width, color = c)
                
            # Go through all the edges and draw some lines (red between poses and blue for poses-to-landmarks.)
            for edge in self.edges:
                i, j = edge.i, edge.j
                if i in self.landmarks:
                    c = 'b'
                else:
                    c = 'r'
                    
                ax.plot([self.x_hat[2*i][0] ,self.x_hat[2*j][0]], [self.x_hat[2*i + 1][0], self.x_hat[2*j + 1][0]], color = c)
                ax.plot([self.x_hat[2*i][0] ,self.x_hat[2*j][0]], [self.x_hat[2*i + 1][0], self.x_hat[2*j + 1][0]], color = c)

                # Draw a line between the original and the optimized poses.
                ax.plot( [self.x_hat[2*j][0] ,poses[j][0]], [self.x_hat[2*j + 1][0], poses[j][1]], color = 'k', linestyle = "dotted", alpha = 0.2)


        # ax.set_title("""self.ODOM_COST = %d
        #                 self.LANDMARK_COST = %d
        #                 self.ASSIGNMENT_COST = %d""" % (self.ODOM_NOISE, self.LANDMARK_NOISE, self.PRIOR_NOISE))
        plt.show()