import numpy as np

class Node():
    def __init__(self, x = 0, y = 0, t = 0):
        self.x = 0
        self.y = 0
        self.t = (t + np.pi) % (2*np.pi) - np.pi
    