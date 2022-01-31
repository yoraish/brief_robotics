import numpy as np

class Edge():
    def __init__(self, i, j, xj, yj, tj, cost):
        self.i = i
        self.j = j
        # The following are in i frame.
        self.x = xj 
        self.y = yj
        self.t = tj
        self.cost = cost
    
    def __str__(self):
        return "i: %d, j: %d x: %d y: %d t: %d cost: %d" %(self.i, self.j, self.x, self.y, self.t, self.cost)

class Graph():

    def __init__(self):
        # Let there be n observations.
        # For optimizing angles. Find hat_t to min ||A hat_t - b||
        self.A = None # nxn
        self.b = None # nx1, observations t.

        # For x,y optimization. Find hat_x to min ||C hat_x - d||
        self.C = None # 2nx2n
        self.d = None # 2nx1, stacked x1,y1,x2,y2 ... . Observations.

        # Costs.
        self.ODOM_COST = 1
        self.LANDMARK_COST = 300
        self.ASSIGNMENT_COST = 200

        # Landmarks.
        self.landmarks = set()
        
        # Keep track of xy edges and build matrix C only after optimizing for angles.
        self.edges = []

        # Should we optimize after every edge addition?
        self.online = False

        # Optimized poses.
        self.x_hat, self.t_hat = None, None

        # Visualization.
        self.arrow_length = 0.05
        self.arrow_width = 0.001
        self.prior_xyt = {}

    def initialize(self, x, y, t):
        """Adds the first node to the graph whith set value.

        Args:
            x ([type]): [description]
            y ([type]): [description]
            t ([type]): [description]
        """
        if self.A is not None:
            raise RuntimeError("Matrices A, C already initialized, aborting reinitialization.")
        



    def add_edge(self, i, j, dx, dy, dt, prior_xyt = None):

        """Add an edge to the pose graph.

        Args:
            xi_ix (int): the index of the previous node. Edge is from x_i to x_j, all quantities in the reference frame of x_i. If i == j then this is an assignment, and the x,y,t quantities are in the map frame.
            dx (float): x axis displacement between x_i.x to this new observation.
            dy (float): y axis displacement between x_i.y to this new obserevation.
            dt (float): angle radians change between x_i.t to this new observation. In range [-pi, +pi].
        """
        # Decide what cost to use.
        if i == j:
            cost = self.ASSIGNMENT_COST
            self.landmarks.add(i)
        elif i in self.landmarks:
            cost = self.LANDMARK_COST
        else:
            cost = self.ODOM_COST


        # Record edge.
        self.edges.append(Edge(i, j, dx, dy, dt, cost))

        # Optionally add a visualization prior.
        if prior_xyt is not None:
            self.prior_xyt[j] = prior_xyt

        return 
        if self.A is None and i == 0:
            print("FIRST EDGE INITIALIZATION")
            self.A = np.array([[1]]) * self.ASSIGNMENT_COST
            self.b = np.array([[tj]]) * self.ASSIGNMENT_COST

            self.C = np.array([[1, 0],
                            [0, 1]]) * self.ASSIGNMENT_COST
            self.d = np.array([[dx],
                            [dy]]) * self.ASSIGNMENT_COST
            return
        '''
        else:
            
            # Expand the angle association matrix with zeros.
            self.A = np.hstack((self.A, np.zeros((self.A.shape[0], 1))))
            self.A = np.vstack((self.A, np.zeros((1, self.A.shape[1]))))

            # Expand the angle vector.
            self.b = np.vstack((self.b, np.array([tj]) * cost))

            # Expand the angle matrix.
            self.A[-1, i] = -1.0 * cost
            self.A[-1, j] = 1.0  * cost

            # Either optimize now for angles and then also for xy, or leave it for later.
            if self.online:
                # Optimize for angles.
                t_hat = np.linalg.pinv(self.A).dot(self.b)

                # Reconstruct xy matrix with angle estimates.  
                raise NotImplementedError
                # for edge in self.edges:
                    # Expand the xy vector.
                    # self.C = np.hstack((self.C, np.zeros((self.C.shape[0], 1))))
                    # self.C = np.vstack((self.C, np.zeros((1, self.C.shape[1]))))
                    # self.d = np.vstack((self.d, np.array([tj]) * cost))
        '''

    def optimize(self):
        # Build the angles matrix and vector.
        # Initialize.
        if self.edges[0].i != self.edges[0].j:
            raise RuntimeError("First edge should be an assignment edge, not a cross-node edge.")
        
        self.A = np.array([[1]]) * self.edges[0].cost
        self.b = np.array([[self.edges[0].t]]) * self.edges[0].cost

        for j in range(1, len(self.edges)):
            edge = self.edges[j] # From i to j. xyt are for j and in i frame. If i==j then xyt are in the map frame.

            # Expand the angle association matrix with zeros.
            self.A = np.hstack((self.A, np.zeros((self.A.shape[0], 1))))
            self.A = np.vstack((self.A, np.zeros((1, self.A.shape[1]))))

            # Expand the angle vector.
            self.b = np.vstack((self.b, np.array([edge.t]) * edge.cost))

            # Update the angle matrix.
            self.A[-1, edge.i] = -1.0 * edge.cost
            self.A[-1, edge.j] = 1.0  * edge.cost
        
        # Optimize for angles.
        t_hat = np.linalg.pinv(self.A).dot(self.b)

        print(t_hat)

        # Build the xy matrix and vector.
        self.C = np.array([[1, 0],
                            [0, 1]]) * self.edges[0].cost
        self.d = np.array([[self.edges[0].x], [self.edges[0].y]]) * self.edges[0].cost

        for j in range(1, len(self.edges)):
            edge = self.edges[j]

            # Expand the xy matrix with two columns and two rows.
            self.C = np.hstack((self.C, np.zeros((self.C.shape[0], 2))))
            self.C = np.vstack((self.C, np.zeros((2, self.C.shape[1]))))

            # Expand the observation xy vector.
            self.d = np.vstack((self.d, np.array([[edge.x], [edge.y]]) * edge.cost))

            # Update the xy matrix. 
            # If this is an assignment edge, mark ones and zeroes appropriately.
            if edge.i == edge.j:
                self.C[-2, 2*edge.j]     = 1 * edge.cost # The x value.
                self.C[-1, 2*edge.j + 1] = 1 * edge.cost # The y value.
            
            else:
                # If i is not in the graph yet, add two more columns since we have 2 more variables (x_i,y_i, x_j, y_j) as opposed to only x_j, y_j.
                if edge.i >= len(self.C):
                    self.C = np.hstack((self.C, np.zeros((self.C.shape[0], 2))))

                    
                # The equation for x_j.
                self.C[-2, 2*edge.i]     = -np.cos(t_hat[edge.i]) * edge.cost 
                self.C[-2, 2*edge.i + 1] = -np.sin(t_hat[edge.i]) * edge.cost 
                self.C[-2, 2*edge.j]     =  np.cos(t_hat[edge.i]) * edge.cost 
                self.C[-2, 2*edge.j + 1] =  np.sin(t_hat[edge.i]) * edge.cost 

                # The equation for y_j.
                self.C[-1, 2*edge.i]     =  np.sin(t_hat[edge.i]) * edge.cost  
                self.C[-1, 2*edge.i + 1] = -np.cos(t_hat[edge.i]) * edge.cost 
                self.C[-1, 2*edge.j]     = -np.sin(t_hat[edge.i]) * edge.cost
                self.C[-1, 2*edge.j + 1] =  np.cos(t_hat[edge.i]) * edge.cost 


        print(self.C)
        # Solve for the positions.
        x_hat = np.linalg.pinv(self.C).dot(self.d)
        self.x_hat, self.t_hat = x_hat, t_hat
        return x_hat, t_hat


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

                    trans_j = R.dot(np.array([[dx], [dy]])) + np.array([[poses[i][0]], [poses[i][1]]])
                    rot_j = poses[i][2] + dt
                    poses[j] = (trans_j[0][0], trans_j[1][0], rot_j)

                    # Draw in red and if i is a landmark then draw blue line the landmark.
                    ax.arrow(trans_j[0][0], trans_j[1][0], self.arrow_length * np.cos(rot_j), self.arrow_length * np.sin(rot_j), width=self.arrow_width, color = "r", alpha = 0.2)

                    ax.plot([poses[i][0] ,trans_j[0][0]], [poses[i][1], trans_j[1][0]], color = c, alpha = 0.2)

                    # Draw prior if it exists.
                    if j in self.prior_xyt:
                        x,y,t = self.prior_xyt[j]
                        ax.arrow(x, y, self.arrow_length * np.cos(t), self.arrow_length * np.sin(t), width=self.arrow_width, color = "g", alpha = 0.2)
                        ax.scatter([x],[y])
        ax.set_aspect("equal")
    


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
                    
                ax.plot([self.x_hat[2*i][0] ,self.x_hat[2*j][0]], [self.x_hat[2*i + 1][0], self.x_hat[2*j + 1][0]], color = c)
                ax.plot([self.x_hat[2*i][0] ,self.x_hat[2*j][0]], [self.x_hat[2*i + 1][0], self.x_hat[2*j + 1][0]], color = c)

                # Draw a line between the original and the optimized poses.
                ax.plot( [self.x_hat[2*j][0] ,poses[j][0]], [self.x_hat[2*j + 1][0], poses[j][1]], color = 'k', linestyle = "dotted", alpha = 0.2)


        ax.set_title("""self.ODOM_COST = %d
        self.LANDMARK_COST = %d
        self.ASSIGNMENT_COST = %d""" % (self.ODOM_COST, self.LANDMARK_COST, self.ASSIGNMENT_COST))
        plt.show()