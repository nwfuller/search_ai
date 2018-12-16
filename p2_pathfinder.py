from heapq import heappush, heappop

from math import sqrt

import random



def find_path (source_point, destination_point, mesh):
    """ Searches for a path from source_point to destination_point through the mesh
    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:
        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = []
    back_pointer = {}

    #Our code start

    boxes = crude_path(source_point, destination_point, mesh)
    if boxes is None:
        print("No path!")
        return [], []

    path = points(source_point, destination_point, boxes, mesh)
    return path, boxes



def crude_path(source_point, destination_point, mesh):

    boxes = {}

    back_pointer_forward = {}
    back_pointer_backward = {}
    visited = []


    for box in mesh['boxes']:



        if (box[0] <= source_point[0] <= box[1]) and (box[2] <= source_point[1] <= box[3]):
            boxes[source_point] = box
        if (box[0] <= destination_point[0] <= box[1]) and (box[2] <= destination_point[1] <= box[3]):
            boxes[destination_point] = box



    source_box = boxes[source_point]
    dest_box = boxes[destination_point]
    back_pointer_forward[source_box] = None
    back_pointer_backward[dest_box] = None
    queue = []

    heappush(queue, (0, source_box,'destination'))
    heappush(queue, (0,dest_box,'source'))

    found = False
    midway = None
    while len(queue) > 0:
        priority, current, goal = heappop(queue)

        if current in back_pointer_backward.keys() and goal == 'destination':
            midway = current
            found = True
            break
        elif current in back_pointer_forward.keys() and goal == 'source':
            midway = current
            found = True
            break
        neighbors = mesh['adj'][current]
        for neighbor in neighbors:
            if neighbor not in back_pointer_forward.keys() and goal == 'destination':
                    back_pointer_forward[neighbor] = current
                    mid = midpoint(neighbor)
                    heappush(queue,(euclid(mid, destination_point), neighbor, goal))
            elif neighbor not in back_pointer_backward.keys() and goal == 'source':
                    back_pointer_backward[neighbor] = current
                    mid = midpoint(neighbor)
                    heappush(queue,(euclid(mid, source_point), neighbor, goal))


    box_list = []
    if found:
        box = midway
        while back_pointer_forward[box]:
            box_list.append(box)
            box = back_pointer_forward[box]
        box = midway
        while back_pointer_backward[box]:
            box_list.append(box)
            box = back_pointer_backward[box]
        box_list.append(source_box)
        box_list.append(dest_box)
        return box_list
    else:
        return None

def points(source_point, destination_point, boxes, mesh):
    point = []
    for box in boxes:
        if (box[0] <= source_point[0] <= box[1]) and (box[2] <= source_point[1] <= box[3]):
            source_box = box
        if (box[0] <= destination_point[0] <= box[1]) and (box[2] <= destination_point[1] <= box[3]):
            dest_box = box

    if source_box == dest_box:
        point.append((source_point, destination_point))
        return point

    link = False
    curr_box = source_box
    visited = []
    curr_point = source_point
    while link is False:
        for box in boxes:
            if box in mesh['adj'][curr_box] and box not in visited:
                visited.append(curr_box)
                temp = shortest_line_to_box(curr_point, box)
                point.append((curr_point,temp))
                curr_point = temp
                curr_box = box
                break
        if curr_box == dest_box:
            point.append((curr_point, destination_point))
            break
    return pointgoo

def euclid(start, end):

    return sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)



def midpoint(box):

    return ((box[0] + box[1])/2, (box[2] + box[3])/2)
