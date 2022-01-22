import numpy as np
from graph import Graph

g = Graph()
g.add_edge(0, 0, 10, 10, 0)
g.add_edge(1, 1, 10, 10, 0)
g.add_edge(2, 2, 10, 10, np.pi)
g.add_edge(3, 3, 10, 10, 0)
g.optimize()
