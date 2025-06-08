import sys
import os
import math
import heapq

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from superutilities import MySuperCustomRobot
from collections import deque


robot = MySuperCustomRobot(verbose=False)
robot.initialize_devices()

timestep = int(robot.getBasicTimeStep())


# Access pedestrian nodes
ped1_node = robot.getFromDef("PEDESTRIAN1")  # Make sure your pedestrian has DEF name in world file
ped2_node = robot.getFromDef("PEDESTRIAN2")
ped1_trans_field = ped1_node.getField("translation")
ped2_trans_field = ped2_node.getField("translation")


translation_field = robot.getFromDef("WARDEN").getField("translation")
start_world = translation_field.getSFVec3f()

def world_to_grid(pos, arena_size=0.8, grid_size=16):
    x, _, z = pos  # Only X and Z are relevant (Y is not used)

    # Calculate the cell size in world units
    cell_size = arena_size / grid_size

    # Shift origin from center of the world (0,0) to top-left corner of the grid (0,0)
    shifted_x = x + (arena_size / 2)
    shifted_z = z + (arena_size / 2)

    # Convert world coordinates to grid coordinates
    grid_x = int(shifted_x / cell_size)
    grid_y = grid_size - 1 - int(shifted_z / cell_size)  # Flip Z to match top-left grid origin

    # Ensure the grid coordinates stay within bounds
    grid_x = max(0, min(grid_x, grid_size - 1))
    grid_y = max(0, min(grid_y, grid_size - 1))

    return grid_x, grid_y


def grid_to_world(row, col):
    x = MIN_X + col * GRID_RESOLUTION + GRID_RESOLUTION / 2
    z = MIN_Z + row * GRID_RESOLUTION + GRID_RESOLUTION / 2
    return [x, 0.0, z]

def get_position(node_field):
    return node_field.getSFVec3f()

def distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[2] - pos2[2])**2)



world_map = [
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
]
#print(start_world)
# print(start_cell)

stuck_threshold = 0.001
stuck_time_1 = 0
stuck_time_2 = 0
prev_ped1_pos = get_position(ped1_trans_field)
prev_ped2_pos = get_position(ped2_trans_field)
ped1_helped = False
ped2_helped = False


#start = start_cell

def is_valid_position(position):
    """ Returns True if the specified position is within the map and unoccupied. """
    map_size = 8
    if 0 <= position[0] < map_size and 0 <= position[1] < map_size:
        if world_map[position[0]][position[1]] == 0:
            return True
    return False

def get_neighbors(map, position):
    #print(position)
    """ Returns the valid neightbors of a cell in the map. """
    north = (position[0]-1, position[1])
    east = (position[0], position[1]+1)
    south = (position[0]+1, position[1])
    west = (position[0], position[1]-1)
    output = []
    for direction in [north, east, south, west]:
        if is_valid_position(direction):
            output.append(direction)
    return output


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

# def path_planning_BFS(grid, start, goal):
#     rows, cols = len(grid), len(grid[0])
#     open_set = []
#     heapq.heappush(open_set, (0, start))
#     came_from = {}
#     g_score = {start: 0}

#     while open_set:
#         _, current = heapq.heappop(open_set)
#         if current == goal:
#             # Reconstruct path
#             path = []
#             while current in came_from:
#                 path.append(current)
#                 current = came_from[current]
#             path.reverse()
#             return path

#         neighbors = [
#             (current[0] + dx, current[1] + dy)
#             for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]
#         ]
#         for neighbor in neighbors:
#             if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0):
#                 tentative_g = g_score[current] + 1
#                 if neighbor not in g_score or tentative_g < g_score[neighbor]:
#                     g_score[neighbor] = tentative_g
#                     f_score = tentative_g + heuristic(neighbor, goal)
#                     heapq.heappush(open_set, (f_score, neighbor))
#                     came_from[neighbor] = current
#     return []



def path_planning_BFS(map, start, goal):
    
    frontier = deque()
    frontier.append(start)
    came_from = dict()
    came_from[start] = None
    while frontier:
        current = frontier.popleft()
        if current == goal:
            break    
        for neighbor in get_neighbors(world_map, current):
            if neighbor not in came_from:
                frontier.append(neighbor)
                came_from[neighbor] = current
    # Build the ordered list
    step = came_from[goal]
    path = [goal]
    while step is not None:
        path.append(step)
        step = came_from[step]
    path.reverse()
    return path

def move_from_to(current, next):
    """ Decide in which cardinal direction to move, based on the current and
    next positions. """
    #print(current)
    #print(next)
    if next[0] < current[0]:
        robot.turn_north()
        robot.move_forward()
    elif next[0] > current[0]:
        robot.turn_south()
        robot.move_forward()
    if next[1] < current[1]:
        robot.turn_west()
        robot.move_forward()
    elif next[1] > current[1]:
        robot.turn_east()
        robot.move_forward()
 

def move_to_help_pedestrian(ped1_pos,ped):

    if ped == "ped1":
        #start = world_to_grid(start_world)
        #goal = world_to_grid(ped1_pos)

        start = (7, 1)  
        goal = (1, 1)
    if ped == "ped2":
        start = (1, 1)  
        goal = (2, 3)
    if ped == "back":
        start = (2, 3)  
        goal = (7, 1)

    #print(start)
    #print(goal)

    path = path_planning_BFSg_BFS(world_map, start, goal)

    current = start
    for next in path[1:]:
        print("Next step: moving from {0} to {1}".format(current, next))
        move_from_to(current, next)
        current = next
    print("Goal reached!")
    



    # goal_cell = world_to_grid(ped1_pos)
    # path = bfs(grid, start_cell, goal_cell)
    # for cell in path:
    #     target_pos = grid_to_world(*cell)
    #     move_to_position(robot, target_pos, translation_field)



while robot.step(timestep) != -1:


    light_node = robot.getFromDef("RESCUE_LIGHT")
    shape_node = light_node.getField("children").getMFNode(0)
    geometry_field = shape_node.getField("geometry")
    geometry_node = geometry_field.getSFNode()
    light_size_field = geometry_node.getField("size")


    ped1_pos = get_position(ped1_trans_field)
    ped2_pos = get_position(ped2_trans_field)

    # Check if pedestrian 1 is stuck

    if not ped1_helped:
        if distance(prev_ped1_pos, ped1_pos) < stuck_threshold:
            stuck_time_1 += 1
        else:
            stuck_time_1 = 0
        prev_ped1_pos = ped1_pos

        if stuck_time_1 > 30:
            print("Pedestrian 1 is stuck, moving to help...")
            light_size_field.setSFVec3f([0.1, 0.1, 0.1])
            move_to_help_pedestrian(ped1_pos,"ped1")
            ped1_node = robot.getFromDef("PEDESTRIAN1")  # or whatever the DEF name is
            ped1_node_field = ped1_node.getField("translation")
            ped1_node_field.setSFVec3f([0.4, 0.2, -6.39568e-05])  # new position
            ped1_helped = True
            light_size_field.setSFVec3f([0.001, 0.001, 0.001])


    elif not ped2_helped:

        if distance(prev_ped2_pos, ped2_pos) < stuck_threshold:
            stuck_time_2 += 1
        else:
            stuck_time_2 = 0
        prev_ped2_pos = ped2_pos

        if stuck_time_2 > 30:
            print("Pedestrian 2 is stuck, moving to help...")
            light_size_field.setSFVec3f([0.1, 0.1, 0.1])
            move_to_help_pedestrian(ped2_pos,"ped2")
            ped2_node = robot.getFromDef("PEDESTRIAN2")  # or whatever the DEF name is
            ped2_node_field = ped2_node.getField("translation")
            ped2_node_field.setSFVec3f([0.4, 0.151715, -6.39568e-05])  # new position
            ped2_helped = True
            light_size_field.setSFVec3f([0.001, 0.001, 0.001])
                

    # Stop when both are helped
    if ped1_helped and ped2_helped:
        print("Both pedestrians helped. Warden finished.")
        move_to_help_pedestrian(ped2_pos,"back")
        break




# pedestrian_pos = [-0.1, 0.0, 0.3]  # You can dynamically read this too
# pedestrian_pos = get_position(ped1_trans_field)
