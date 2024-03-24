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
from matplotlib.animation import FuncAnimation
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


# a_star algorithm
def astar_algorithm(start_pos, goal_pos, environment, move_step):
    if goal_reached(start_pos, goal_pos):
        return None
   # declaring start and goal node
    start_node = start_pos
    goal_node = goal_pos

    available_actions = [0, 1, 2, 3, 4]
    open_list = {}
    closed_list = {}
    priority_queue = []
    visited_nodes = []
    # initialize the open list to start exploration
    start_id = unique_id(start_node)
    open_list[(start_id)] = start_node
    heapq.heappush(priority_queue, [start_node.path_cost, start_node])
    # check if open list (prority queue) is not empty
    while (len(priority_queue) != 0):
        current_node = heapq.heappop(priority_queue)[1]
        visited_nodes.append([current_node.x_pos, current_node.y_pos, current_node.orientation])
        current_id = unique_id(current_node)

        if goal_reached(current_node, goal_node):
            goal_node.parent_id = current_node.parent_id
            goal_node.path_cost = current_node.path_cost
            print("Reached Goal")
            return visited_nodes, 1

        if current_id in closed_list:
            continue
        else:
            closed_list[current_id] = current_node
        
        del open_list[current_id]

        for action_code in available_actions:
            x_pos, y_pos, orientation, path_cost = execute_movement(action_code, current_node.x_pos , current_node.y_pos , current_node.orientation , move_step, current_node.path_cost)

            est_cost_to_goal = dist((x_pos, y_pos) , (goal_pos.x_pos , goal_pos.y_pos))

            new_node = node(x_pos, y_pos, orientation, path_cost, current_node, est_cost_to_goal)

            new_node_id = unique_id(new_node)
            if not check_move_legality(new_node.x_pos, new_node.y_pos, environment):
                continue
            elif new_node_id in closed_list:
                continue
            
            if new_node_id in open_list:
                if new_node.path_cost < open_list[new_node_id].path_cost:
                    open_list[new_node_id].path_cost = new_node.path_cost
                    open_list[new_node_id].parent_id = new_node.parent_id
            else:
                open_list[new_node_id] = new_node
            
            heapq.heappush(priority_queue, [(new_node.path_cost + new_node.est_cost_to_goal), new_node])
    # print("a_star_function_executed")
    return visited_nodes, 0

# back tracking the optimal path
def backtracking(goal_pos):
    path_x = []
    path_y = []
    path_x.append(goal_pos.x_pos)
    path_y.append(goal_pos.y_pos)

    parent = goal_pos.parent_id
    while parent != -1:
        path_x.append(parent.x_pos)
        path_y.append(parent.y_pos)
        parent = parent.parent_id
    path_x.reverse()
    path_y.reverse()
    # saving the path coordintes in two arrays for plotting
    x_pos = np.asarray(path_x)
    y_pos = np.asanyarray(path_y)
    # print("plot_tracer_funtion_executed")
    return x_pos, y_pos

# visualizing the explored_nodes and generate_path

#Plotting the workspace and the shortest path
def plot(start_node, goal_node, path_x, path_y, visited_waypoints, environment):
    fig, ax = plt.subplots()
    my_colors = np.array([(150, 210, 230), (255, 128, 0), (130, 130, 130)], dtype=float) / 255  # Normalize values between 0 and 1
    custom_cmap = ListedColormap(my_colors)
    ax.imshow(environment, cmap= custom_cmap)
    ax.invert_yaxis()  # Flip the y-axis to match the coordinate system
    #  Mark start and goal
    ax.plot(start_node.x_pos, start_node.y_pos, "Dw", markersize=4)  # Start
    ax.plot(goal_node.x_pos, goal_node.y_pos, "Dr", markersize=4)  # Goal

    path_line, = ax.plot([], [], 'y', lw=2)  # Initialize the line for the path

    def init():
        path_line.set_data([], [])
        return path_line,

    def update(frame):
        # Update the path line to include up to the current frame
        path_line.set_data(path_x[:frame], path_y[:frame])
        return path_line,

    frames = len(path_x)  # One frame for each step in the path

    anim = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, repeat=False)

    # Save the animation
    anim.save('optimal_path_animation.mp4', writer='ffmpeg', fps=40)
    print("plot_path_funtion_executed")
    plt.show()

def plot_node_exploration(start_node, goal_node, visited_waypoints, environment):
    fig, ax = plt.subplots()
    my_colors = np.array([(150, 210, 230), (255, 128, 0), (130, 130, 130)], dtype=float) / 255  # Normalize values between 0 and 1
    custom_cmap = ListedColormap(my_colors)
    ax.imshow(environment, cmap=custom_cmap)
    ax.invert_yaxis()  # Flip the y-axis to match the coordinate system
    print("length of all_node:", len(visited_waypoints))
    # Mark start and goal
    ax.plot(start_node.x_pos, start_node.y_pos, "Dw", markersize=4, label='Start')  # Start
    ax.plot(goal_node.x_pos, goal_node.y_pos, "Dr", markersize=4, label='Goal')  # Goal

    # explored_nodes, = ax.plot([], [], "ob-", alpha=0.6, markersize=3, label='Explored Nodes')  # Initialize the line for explored nodes
    explored_nodes, = ax.plot([], [], "ob", alpha=0.8, markersize=3, label='Explored Nodes')  # Initialize the line for explored nodes

    def init():
        explored_nodes.set_data([], [])
        # explored_nodes.set_data(path_x[:frames], path_y[:frames])

        return explored_nodes,

    def update(frame):
         # Determine the range of points to display in this frame
        if frame >= len(visited_waypoints):
            return explored_nodes,
        # Calculate the end_index for the current frame
        # end_index = min((frame + 1) * 200, len(visited_waypoints))
        end_index = min((frame + 1) * max_nodes_per_frame, len(visited_waypoints))

    # Extract coordinates up to end_index
        x_coords, y_coords = zip(*[(wp[0], wp[1]) for wp in visited_waypoints[:end_index]])

        explored_nodes.set_data(x_coords, y_coords)
        return explored_nodes,

    if len(visited_waypoints)> 200:
        max_nodes_per_frame = 1000
    else:max_nodes_per_frame = 100
    # frames = len(path_x)  # Use total path length for frames
    frames = (len(visited_waypoints) + 99) // max_nodes_per_frame
    first_plot_frame_T_no = frames
    total_frames = first_plot_frame_T_no + len(path_x)
    print("creating video file")
    print("total_frames:", total_frames)

    # total_points = len(visited_waypoints)
    # points_per_frame = 200
    # frames = math.ceil(total_points / points_per_frame)
    anim = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, repeat=False, interval=100)

    # Save the animation
    anim.save('node_exploration_animation.mp4', writer='ffmpeg', fps=30)
    print("plot_node_explore_funtion_executed")
    plt.legend()
    plt.show()
    
# code to generate common visualization video for the code 
# def combined_plot_animation(start_node, goal_node, path_x, path_y, visited_waypoints, environment):
#     fig, ax = plt.subplots()
#     # defining custom color code
#     my_colors = np.array([(150, 210, 230), (255, 128, 0), (130, 130, 130)], dtype=float) / 255  # Normalize values between 0 and 1
#     custom_cmap = ListedColormap(my_colors)
#     ax.imshow(environment, cmap=custom_cmap)
#     ax.invert_yaxis()  # Flip the y-axis to change the coordinate system (frame)

#     # Mark start and goal
#     ax.plot(start_node.x_pos, start_node.y_pos, "Dw", markersize=4, label='Start')  # plot start node
#     ax.plot(goal_node.x_pos, goal_node.y_pos, "Dr", markersize=4, label='Goal')  # plot goal node

#     # Initialize path line (yellow, initially hidden) and explored nodes line (blue)
#     path_line, = ax.plot([], [], 'y', lw=2, label='Path', visible=False)  # Initially hidden path
#     explored_nodes, = ax.plot([], [], "ob", alpha=0.8, markersize=2, label='Explored Nodes')

#     def init():
#         explored_nodes.set_data([], [])
#         path_line.set_data([], [])
#         return explored_nodes, path_line

#     def update(frame):
#         if frame < total_frames - len(path_x) + 1:  # Show explored nodes until path_line starts
#             end_index = min((frame + 1) * max_nodes_per_frame, len(visited_waypoints))
#             x_coords, y_coords = zip(*[(wp[0], wp[1]) for wp in visited_waypoints[:end_index]])
#             explored_nodes.set_data(x_coords, y_coords)
#             print("frame_no.:", frame)

#         if frame >= total_frames - len(path_x) + 1:
#             path_line.set_data(path_x[:frame - (total_frames - len(path_x))], path_y[:frame - (total_frames - len(path_x))])
#             path_line.set_visible(True)  # Make path visible starting from the appropriate frame
#             print("frame_no.for path_line:", frame)
#         return explored_nodes, path_line

#     total_points = len(visited_waypoints)  # Not used in this version
#     if len(visited_waypoints)> 200:
#         max_nodes_per_frame = 1000
#     else:max_nodes_per_frame = 100
#     # frames = len(path_x)  # Use total path length for frames
#     frames = (len(visited_waypoints) + 99) // max_nodes_per_frame
#     first_plot_frame_T_no = frames
#     total_frames = first_plot_frame_T_no + len(path_x)
#     print("creating video file")
#     print("total_frames:", total_frames)
#     # anim = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, repeat=False, interval=100)
#     anim = FuncAnimation(fig, update, frames=total_frames, init_func=init, blit=True, repeat=False, interval=100)

#     # Save the animation
#     anim.save('exploration_and_path_animation.mp4', writer='ffmpeg', fps=20)
#     print("video file saved!","\n","showing the exploration...")
#     # print("combined_plot_animation_executed")
#     plt.legend()
#     plt.show()

   
#Main function
if __name__ == '__main__':
    clearance = int(input("Enter the clearance: "))  #Robot's clearance
    radius = int(input("Enter the radius of the robot: ")) #Robot's radius
    while True:
        try:
            robot_step = int(input("Enter the robot step size: "))  #Robot's step size
            if 1 <= robot_step <= 10:
                break  # Exit the loop if the input is within bounds
            else:
                print("Step size is not within bounds. Please enter a value between 1 and 10.")
        except ValueError:
            print("Invalid input. Please enter an integer value between 1 and 10.")

    env_width = 1200 #Width of the workspace
    env_height = 500 #Height of the workspace
    environment = environment(env_width, env_height, clearance, radius) #Generating the workspace with the advised buffer
    est_cost_to_goal = 0 #Initial cost to go

    #Asking start co-ordinates and orientation of the robot from the user
    startpoint_x = int(input("Enter X coordinate of the start node (between 0 and 1200): "))
    startpoint_y = int(input("Enter Y coordinate of the start node (between 0 and 500): "))
    start_orientation = int(input("Enter the orientation of the robot at initial position (multiple of 30 degree): "))

    #Rounding off the start orientation value to the nearest multiple of 30
    rounding_int = int(start_orientation)
    remainder_int = rounding_int % 30
    if remainder_int < 15:
        start_orientation = rounding_int - remainder_int
    else:
        start_orientation = rounding_int + (30 - remainder_int)

    #Checking the validity of the given starting point in the workspace
    if not check_move_legality(startpoint_x, startpoint_y, environment):
        print("Start node is either out of bounds or in the obstacle")
        exit(-1)

    #Asking goal co-ordinates and orientation of the robot from the user
    goalpoint_x = int(input("Enter X coordinate of the goal node (between 0 and 1200): "))
    goalpoint_y = int(input("Enter Y coordinate of the goal node (between 0 and 500): "))
    goal_orientation = int(input("Enter the orientation of the robot at final position (multiple of 30 degree): "))
    
    #Rounding off the goal orientation value to the nearest multiple of 30
    rounding_int = int(goal_orientation)
    remainder_int = rounding_int % 30
    if remainder_int < 15:
        goal_orientation = rounding_int - remainder_int
    else:
        goal_orientation = rounding_int + (30 - remainder_int)

    #Checking the validity of the given goal point in the workspace
    if not check_move_legality(goalpoint_x, goalpoint_y, environment):
        print("The goal is in the way of obstacles or out of bounds")
        exit(-1)

    start_timer = time.time() #Initialising timer to calculate the total computational time

    #Forming the start node and the goal node objects
    start_node = node(startpoint_x, startpoint_y, start_orientation, 0.0, -1, est_cost_to_goal)
    goal_node = node(goalpoint_x, goalpoint_y, goal_orientation, 0.0, -1, est_cost_to_goal)
    visited_nodes, flag = astar_algorithm(start_node, goal_node, environment, robot_step)

    #Plotting the most optimal path after verifying that the goal node has been reached
    if (flag) == 1:
        path_x, path_y = backtracking(goal_node)
        path_cost = goal_node.path_cost #Total cost to reach the goal node from the start node
        print("Total cost for the path:", path_cost)

        plot_node_exploration(start_node, goal_node, visited_nodes, environment)
        # Wait for 5 seconds
        time.sleep(5)
        plot(start_node, goal_node, path_x, path_y, visited_nodes, environment)
        # combined_plot_animation(start_node, goal_node, path_x, path_y, visited_nodes, environment)

        stop_timer = time.time() #Stopping the timer and displaying the time taken to reach the goal node from the start node while exploring all the nodes
        total_time = stop_timer - start_timer
        print("Total execution time  ", total_time)

    else:
        print("Path not found")
