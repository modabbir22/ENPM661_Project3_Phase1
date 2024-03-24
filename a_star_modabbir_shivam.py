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

# Action Function to move the robot in desired direction
def move_action(x_pos, y_pos, orientation, step_length, path_cost):
    orientation = orientation
    x_pos += step_length * np.cos(np.radians(orientation))
    y_pos += step_length * np.sin(np.radians(orientation))
    return round(x_pos), round(y_pos), orientation, path_cost + 1

def execute_movement(action_code, x_pos, y_pos, orientation, step_length, path_cost):
    if action_code == 0:  # move 60 degree left
        return move_action(x_pos, y_pos, orientation + 60, step_length, path_cost)
    elif action_code == 1: # move 30 degree left
        return move_action(x_pos, y_pos, orientation + 30, step_length, path_cost)
    elif action_code == 2: # move straight 
        return move_action(x_pos, y_pos, orientation + 0 , step_length, path_cost)
    elif action_code == 3: # move 30 degree right
        return move_action(x_pos, y_pos, orientation-30, step_length, path_cost)
    elif action_code == 4: # move 60 degree right
        return move_action(x_pos, y_pos, orientation-60, step_length, path_cost)
    else:
        return None
