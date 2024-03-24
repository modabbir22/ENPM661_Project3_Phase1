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

# create obstacle map

def environment(env_width, env_height, clearance, radius):
    environment = np.full((env_height, env_width), 0)

    for y_pos in range(0 , env_height):
        for x_pos in range(0 , env_width):
            #Using half plane equations to plot the buffer space for obstacles

            #Adding buffer space to avoid collision between the robot and the walls
            wall_clr_1 = (x_pos - (clearance + radius))
            wall_clr_2 = (x_pos + (clearance + radius)) - 1200
            wall_clr_3 = (y_pos + (clearance + radius)) - 500
            wall_clr_4 = (y_pos - (clearance + radius)) 

            #Rectangular Obstacle 1 Buffer
            rectangle11_buffer = (x_pos + (clearance + radius)) - 100
            rectangle12_buffer = (x_pos - (clearance + radius)) - 175
            rectangle13_buffer = (y_pos + (clearance + radius)) - 100
            rectangle14_buffer = (y_pos) - 500

            #Rectangular Obstacle 2 Buffer
            rectangle21_buffer = (x_pos + (clearance + radius)) - 275
            rectangle22_buffer = (x_pos - (clearance + radius)) - 350
            rectangle23_buffer = (y_pos) - 0
            rectangle24_buffer = (y_pos - (clearance + radius)) - 400

            #Hexagon Obstacle 3 Buffer
            hexagon6_buffer = (y_pos + (clearance + radius)) + 0.573 * (
                        x_pos + (clearance + radius)) - 473.533
            hexagon5_buffer = (y_pos + (clearance + radius)) - 0.573 * (
                        x_pos - (clearance + radius)) + 271.367
            hexagon4_buffer = (x_pos - (clearance + radius)) - 770
            hexagon3_buffer = (y_pos - (clearance + radius)) + 0.573 * (
                        x_pos - (clearance + radius)) - 771.367
            hexagon2_buffer = (y_pos - (clearance + radius)) - 0.573 * (
                        x_pos + (clearance + radius)) - 26.467
            hexagon1_buffer = (x_pos + (clearance + radius)) - 530

            #C-shaped Obstacle 4 Buffer
            c1_buffer = (x_pos + (clearance + radius)) - 900
            c2_buffer = (x_pos - (clearance + radius)) - 1100
            c3_buffer = (y_pos + (clearance + radius)) - 50
            c4_buffer = (y_pos - (clearance + radius)) - 450
            c5_buffer = (x_pos + (clearance + radius)) - 900
            c6_buffer = (x_pos + (clearance + radius)) - 1020
            c7_buffer = (y_pos - (clearance + radius)) - 125
            c8_buffer = (y_pos + (clearance + radius)) - 375
            #Setting of constraints for the obstacle with the buffer
            if ((
                    hexagon6_buffer > 0 and hexagon5_buffer > 0 and hexagon4_buffer < 0 and hexagon3_buffer < 0 and hexagon2_buffer < 0 and hexagon1_buffer > 0) or 
                    (wall_clr_1 < 0 or wall_clr_2 > 0 or wall_clr_3 > 0 or wall_clr_4 < 0) or (
                    rectangle11_buffer > 0 and rectangle12_buffer < 0 and rectangle13_buffer > 0 and rectangle14_buffer < 0) or (
                    rectangle21_buffer > 0 and rectangle22_buffer < 0 and rectangle23_buffer > 0 and rectangle24_buffer < 0) or (
                    c1_buffer > 0 and c2_buffer < 0 and c3_buffer > 0 and c4_buffer < 0 and not (c5_buffer > 0 and c6_buffer < 0 and c7_buffer > 0 and c8_buffer < 0))):

                    environment[y_pos, x_pos] = 1

            #Using half plane equation to plot the actual obstacle map

            #Upper Rectangular Obstacle
            rectangle11 = (x_pos) - 100
            rectangle12 = (x_pos) - 175
            rectangle13 = (y_pos) - 100
            rectangle14 = (y_pos) - 500

            #Lower Rectangular Obstacle
            rectangle21 = (x_pos) - 275
            rectangle22 = (x_pos) - 350
            rectangle23 = (y_pos) - 0
            rectangle24 = (y_pos) - 400


            #Hexagonal Obstacle 3
            hexagon6 = (y_pos) + 0.573 * (x_pos) - 473.533
            hexagon5 = (y_pos) - 0.573 * (x_pos) + 271.367
            hexagon4 = (x_pos) - 770
            hexagon3 = (y_pos) + 0.573 * (x_pos) - 771.367
            hexagon2 = (y_pos) - 0.573 * (x_pos) - 26.467
            hexagon1 = (x_pos) - 530

            #Triangular Obstacle
            c1 = (x_pos) - 900
            c2 = (x_pos) - 1100
            c3 = (y_pos) - 50
            c4 = (y_pos) - 450
            c5 = (x_pos) - 900
            c6 = (x_pos) - 1020
            c7 = (y_pos) - 125
            c8 = (y_pos) - 375

            #Setting of line constraint for the obstacle
            if ((hexagon6 > 0 and hexagon5 > 0 and hexagon4 < 0 and hexagon3 < 0 and hexagon2 < 0 and hexagon1 > 0) or (
                    rectangle11 > 0 and rectangle12 < 0 and rectangle13 > 0 and rectangle14 < 0) or (rectangle21 > 0 and rectangle22 < 0 and rectangle23 > 0 and rectangle24 < 0) or (
                    c1 > 0 and c2 < 0 and c3 > 0 and c4 < 0 and not (c5 > 0 and c6 < 0 and c7 > 0 and c8 < 0))):
                    environment[y_pos, x_pos] = 2
    # print("environment_funtion_executed")
    return environment


# check feasibility of move_action()
def check_move_legality(x_coord, y_coord, environment):
    check_move_legality = environment.shape
    if (x_coord > check_move_legality[1] or x_coord < 0 or y_coord > check_move_legality[0] or y_coord < 0):
        return False
    else:
        try:
            if (environment[y_coord][x_coord] == 1 or environment[y_coord][x_coord] == 2):
                return False
        except:
            pass
    return True
# check if goal is reached with thresold range value 1.5
def goal_reached(current_pos, goal_pos):
    goal_threshold = 1.5
    distance = dist((current_pos.x_pos, current_pos.y_pos) , (goal_pos.x_pos , goal_pos.y_pos))   # dist() to calculate euclidian distance between two nodes
    if distance < goal_threshold:
        return True
    else:
        return False
# creating key for each explored node to updated in closed list
def unique_id(node):
    unique_id =  501 * node.x_pos + 1201 * node.y_pos
    return unique_id
