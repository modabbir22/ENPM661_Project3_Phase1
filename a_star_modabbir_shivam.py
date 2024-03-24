# ENPM661 Project3 Phase1: A_star_algorithm

# Group Project Members: Shivam Dhakad and Modabbir Adeeb

# Github Link: https://github.com/modabbir22/ENPM661_Project3_Phase1



# import Libraries
import numpy as np
import math
from math import dist
import matplotlib.pyplot as plt
import time
import heapq
from matplotlib.animation import FuncAnimation #, PillowWriter
# import matplotlib.patches as patches
from matplotlib.colors import ListedColormap

# Class to represent graph nodes
class node:

    def __init__(self, x_pos, y_pos, orientation, path_cost, parent_id, est_cost_to_goal=0):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.orientation = orientation
        self.path_cost = path_cost
        self.parent_id = parent_id
        self.est_cost_to_goal = est_cost_to_goal

    def __lt__(self, comp):
        return self.path_cost + self.est_cost_to_goal < comp.path_cost + comp.est_cost_to_goal