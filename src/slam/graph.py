import numpy as np

class Edge():
    def __init__(self, i, j, xj, yj, tj, cost) -> None:
        self.i = i
        self.j = j
        self.x = xj
        self.y = yj
        self.t = tj
        self.cost = cost

class Graph():

    def __init__(self) -> None:
        # Let there be n observations.
        # For optimizing angles. Find hat_t to min ||A hat_t - b||
        self.A = None # nxn
        self.b = None # nx1, observations t.

        # For x,y optimization. Find hat_x to min ||C hat_x - d||
        self.C = None # 2nx2n
        self.d = None # 2nx1, stacked x1,y1,x2,y2 ... . Observations.

        # Costs.
        self.ODOM_COST = 1
        self.LANDMARK_COST = 100
        self.ASSIGNMENT_COST = 10000

        # Landmarks.
        self.assigned = set()
        
        # Keep track of xy edges and build matrix C only after optimizing for angles.
        self.edges = []

        # Should we optimize after every edge addition?
        self.online = False

    def initialize(self, x, y, t):
        """Adds the first node to the graph whith set value.

        Args:
            x ([type]): [description]
            y ([type]): [description]
            t ([type]): [description]
        """
        if self.A is not None:
            raise RuntimeError("Matrices A, C already initialized, aborting reinitialization.")
        


        

    def add_edge(self, i, j, dx, dy, dt):

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
        elif i in self.assigned:
            cost = self.LANDMARK_COST
        else:
            cost = self.ODOM_COST

        # Record edge.
        self.edges.append(Edge(i, j, dx, dy, dt, cost))
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
                self.C[-2, 2*edge.i]     = -np.cos(t_hat[edge.i]) * edge.cost # The x value of i. TODO(yoraish): add the other terms from  j to xi yi.
                self.C[-2, 2*edge.i + 1] = -np.sin(t_hat[edge.i]) * edge.cost # The y value of i.
                xj
                yj

                # The equation for y_j
                self.C[-1, 2*edge.j + 1] = 1 * edge.cost # The y value.




 

        # Expand the 





