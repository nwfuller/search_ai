import math, sys, pygame, random
from colour import Color
from math import *
from pygame import *
from pygame.locals import *
img1 = pygame.image.load('button1.png')
img2 = pygame.image.load('button2.png')

WINDOW_X = 400
WINDOW_Y = 500
#WINDOW_X = 720
#WINDOW_Y = 500
GOAL_RADIUS = 10
window = [WINDOW_X, WINDOW_Y]
pygame.init()
screen = pygame.display.set_mode(window)
status = "gen_obstacles"
theta_limits = (-2*pi, 2*pi)
theta_goal = None
distance_limit = 15
nodes = []
obstacles = []
fpsClock = pygame.time.Clock()
goal = None
population = []
children = []
deltaX = WINDOW_X
deltaY = WINDOW_Y
oldDeltas = [deltaX, deltaY]
first_gen = True
N = 60



def generate_obstacle(coordinates):  # generates rect obstacles at onclick positions
    global obstacles
    obstacles.append(pygame.Rect((coordinates[0]-25, coordinates[1]-25), (50, 50)))


def goal_direction(start, goal):  # to draw a vector from start -> goal, this finds the angle in standard position of such a vector with start as the origin
    return atan2((goal[1]-start[1]), (goal[0]-start[0]))
    
    
    
"""Possible Genetic traits: When estimated distance from goal is large (begin by keeping track of closest node to goal), focus expansion on all four corners of the screen,
                            and as the estimated distance narrows,reduce expansion to be within a narrowing space relative to the goal.
                            Have varying values for x_min,x_max,y_min,y_max and find the most effective ones given the current amount of obstacles. """


def random_point():  # generates a random point within the window such that the point is not within an obstacle

    if deltaX <= goal.point[0]:
        x_min = goal.point[0] - deltaX
    else:
        #print("reset xMin")
        x_min = 0
    
    if deltaX + goal.point[0] <= WINDOW_X:
        x_max = deltaX+goal.point[0]
    else:
        #print("reset xMan")
        x_max = WINDOW_X

    if deltaY <= goal.point[1]:
        y_min = goal.point[1] - deltaY
    else:
        #print("reset yMin")
        y_min = 0
        
    if deltaY+goal.point[1] <= WINDOW_Y:
        y_max = deltaY+goal.point[1]
    else:
        #print("reset yMax")
        y_max = WINDOW_Y
    
    while True:
        new_node_coordinates = (random.randint(x_min, x_max), random.randint(y_min, y_max))
        if not collides(new_node_coordinates):
            # pygame.draw.circle(screen, Color("black"), new_node_coordinates, 1, 0)
            
            return new_node_coordinates
    # while True:
    #     new_node_coordinates = (random.randint(0, WINDOW_X), random.randint(0, WINDOW_Y))
    #     if not collides(new_node_coordinates):
    #         pygame.draw.circle(screen, Color("black"), new_node_coordinates, 1, 0)
    #         return new_node_coordinates


def redraw():  # used for clearing button off screen once desired number of obstacles have been added
    screen.fill(Color("white"))
    for obs in obstacles:
        pygame.draw.rect(screen, Color("black"), obs)  # redraws all obstacles on screen


def button(x,y,w,h,ic,ac,action=None):  # button to be used once the desired number of obstacles have been added. onclick changes status from "gen_obstacles" to "set_coordinates"
    global status
    mouse = pygame.mouse.get_pos()  # used for tracking mouse position to identify button mouseovers
    click = pygame.mouse.get_pressed()  # identifies button click events
    # print(click)
    if x+w > mouse[0] > x and y+h > mouse[1] > y:
        screen.blit(img2, (0, 0))
        # pygame.draw.rect(screen, ac,(x,y,w,h))#button is colored differently if mouse is hovering

        if click[0] == 1:
            status = "set_coordinates"
            redraw()
    else:
        screen.blit(img1, (0, 0))
        # pygame.draw.rect(screen, ic,(x,y,w,h))


def collides(point):
    for obs in obstacles:
        if obs.collidepoint(point):  # checks given point against all obstacles, if a point is within an obstacle (on edge does not count) it will return true
            return True
    return False 


def euclid(point_a, point_b):
    return sqrt((point_b[0]-point_a[0])**2 + (point_b[1]-point_a[1])**2)


def goal_reached(p1, p2, radius):
    distance = euclid(p1,p2)
    if (distance <= radius):
        return True
    return False


def add_to_tree(existing_point, new_point):
    if euclid(existing_point, new_point) <= distance_limit:  # if the generated point is within distance_limit of any point on the tree, no changes are necessary
        return new_point
        
    else: 
        theta = atan2((new_point[1]-existing_point[1]), (new_point[0]-existing_point[0]))  # consider existing_point as the origin, theta is the angle in standard position of the vector that ends at new_point
        coords = (existing_point[0] + distance_limit*cos(theta)), (existing_point[1] + distance_limit*sin(theta))  # modifying the positioning of new_point to be within distance_limit of existing_point and in same direction
        return coords


def init():  # redraws screen
    global status
    pygame.display.set_caption("RRT")
    status = "gen_obstacles"
    screen.fill(Color("white"))


def calculate_fitness(time_length, distance, ideal_distance):  # method for calculating the fitness of a delta value
    distance_offset = 100/(abs(distance - ideal_distance))  # how far from the ideal euclidian distance
    print("distance: ", distance, "Ideal Distance: ", ideal_distance)
    time_value = distance/(time_length*.1)  # how long it took vs the distance it had to travel
    return distance_offset + time_value
    

def generate_child(parent_1, parent_2):
    global first_gen  # first gen is the initial population
    first_gen = False
    num = int((parent_1.deltaX + parent_2.deltaX)/2)
    num2 = int((parent_1.deltaY + parent_2.deltaY)/2)
    return (num, num2) # averages the delta value of two parents


def tournament_selection():
    best = None
    random_order = population.copy()
    random.shuffle(random_order)

    for i in range(2):
        individual = random_order[i]
        if best is None or individual.fitness < best.fitness:
            best = individual
    return individual
        

def reset(start, goal):
    global status
    global nodes
    status = "traverse"
    screen.fill(Color("white"))  
    pygame.draw.circle(screen, Color("red"), start, 10, 0)  # Surface, color, pos, radius, width=0
    pygame.draw.circle(screen, Color("green"), goal, 10, 0)  # Surface, color, pos, radius, width=0
    nodes.clear()
    nodes.append(Node(start, None))
     

def rrt():
    global status
    global theta_goal
    global nodes
    global goal
    global population
    global deltaX
    global deltaY
    global oldDeltas
    global first_gen
    global WINDOW_X
    global WINDOW_Y
    init()
    start = Node(None, None)  # beginning node
    start_set = False
    goal = Node(None, None)  # ending node
    goal_set = False
    clock = pygame.time.Clock()
    show_path = False
    first_gen = True
    stuck = False
    r = 155
    g = 155
    b = 155
    farthest_node = 0

    done = False
    while not done:
    
        if status == "gen_obstacles":
            button(0, 0, 100, 50, Color("blue"), Color("cyan"))  # button creation
        
        elif status == "traverse":  # obstacles, start, and goal have all been set. RRT goes here.
            # print("How many times does it go here?")
            node_added = False
            while not node_added:
                print(len(nodes))
                point = random_point()
                parent = start
                for node in nodes:
                    if euclid(node.point, point) <= euclid(parent.point, point): # finding node along existing tree that is closest to the new point, will look through all existing nodes in tree
                    
                        new_point = add_to_tree(node.point, point)
                        if not collides(new_point):
                            if len(nodes) > 1500:
                                lower_bound = int((len(nodes)*.99))
                                latest_distances = []
                                for i in range(lower_bound, len(nodes)-2):
                                    latest_distances.append(euclid(nodes[i].point, nodes[i+1].point))

                                if max(latest_distances) - min(latest_distances) < 2000:

                                    if not stuck:
                                        oldDeltas = [deltaX, deltaY]
                                    stuck = True
                                    deltaX = int(deltaX * 5.0)
                                    deltaY = int(deltaY * 5.0)
                                    if deltaX > WINDOW_X: deltaX = WINDOW_X
                                    if deltaY > WINDOW_Y: deltaY = WINDOW_Y
                                else:
                                    stuck = False
                                    deltaX = oldDeltas[0]
                                    deltaY = oldDeltas[1]
                            elif len(nodes) > 1000:
                                lower_bound = int((len(nodes)*.99))
                                latest_distances = []
                                for i in range(lower_bound, len(nodes)-2):
                                    latest_distances.append(euclid(nodes[i].point, nodes[i+1].point))

                                if max(latest_distances) - min(latest_distances) < 200:

                                    if not stuck:
                                        oldDeltas = [deltaX, deltaY]
                                    stuck = True
                                    deltaX = int(deltaX * 5.0)
                                    deltaY = int(deltaY * 5.0)
                                    if deltaX > WINDOW_X: deltaX = WINDOW_X
                                    if deltaY > WINDOW_Y: deltaY = WINDOW_Y
                                else:
                                    stuck = False
                                    deltaX = oldDeltas[0]
                                    deltaY = oldDeltas[1]
                            elif len(nodes) > 500:
                                lower_bound = int((len(nodes)*.95))
                                latest_distances = []
                                for i in range(lower_bound, len(nodes)-2):
                                    latest_distances.append(euclid(nodes[i].point, nodes[i+1].point))

                                if max(latest_distances) - min(latest_distances) < 100:

                                    if not stuck:
                                        oldDeltas = [deltaX, deltaY]
                                    stuck = True
                                    deltaX = int(deltaX * 1.5)
                                    deltaY = int(deltaY * 1.5)
                                    if deltaX > WINDOW_X: deltaX = WINDOW_X
                                    if deltaY > WINDOW_Y: deltaY = WINDOW_Y
                                else:
                                    stuck = False
                                    deltaX = oldDeltas[0]
                                    deltaY = oldDeltas[1]
                            elif len(nodes) > 100:
                                lower_bound = int((len(nodes)*.95))
                                latest_distances = []
                                for i in range(lower_bound, len(nodes)-2):
                                    latest_distances.append(euclid(nodes[i].point, nodes[i+1].point))

                                if max(latest_distances) - min(latest_distances) < 50:

                                    if not stuck:
                                        oldDeltas = [deltaX, deltaY]
                                    stuck = True
                                    deltaX = int(deltaX * 1.2)
                                    deltaY = int(deltaY * 1.2)
                                    if deltaX > WINDOW_X: deltaX = WINDOW_X
                                    if deltaY > WINDOW_Y: deltaY = WINDOW_Y
                                else:
                                    stuck = False
                                    deltaX = oldDeltas[0]
                                    deltaY = oldDeltas[1]



                            node_added = True
                            parent = node
            # This takes rgb and alters them every loop so that the colors change
            r += random.randint(-5, 5)
            g += random.randint(-5, 5)
            b += random.randint(-5, 5)
            # limit to rgb size
            if r > 255: r = 255
            if r < 0:   r = 0
            if g > 255: g = 255
            if g < 0:   g = 0
            if b > 255: b = 255
            if b < 0:   b = 0

            new_node = add_to_tree(parent.point, new_point)
            nodes.append(Node(new_node, parent))
            pygame.draw.line(screen, Color(r, g, b, 0), parent.point, new_node)
            
            if goal_reached(new_node, goal.point, GOAL_RADIUS):
                status = 'goalFound'

                time_since_enter = pygame.time.get_ticks() - start_time
                start_time = pygame.time.get_ticks()
                print(time_since_enter)
                goalNode = nodes[len(nodes)-1]
        elif status == "goalFound":
            # end = time.time()
            currNode = goalNode.parent
            distance = 0
            while currNode.parent != None:
                distance += euclid(currNode.point, currNode.parent.point)
                pygame.draw.line(screen,Color("blue"),currNode.point,currNode.parent.point, 3)
                currNode = currNode.parent
            show_path = True
            
            if len(population) < N and first_gen:  # for genetating the initial population, we cycle through delta values from large to small (delta is the difference between the min x,y values and the goal x,y values)
                deltaX = int(deltaX * .5)
                deltaY = int(deltaY * .5)
                if deltaX < 20: deltaX = 10
                if deltaY < 20: deltaY = 10
                fitness = calculate_fitness(time_since_enter, distance, (euclid(start.point, goal.point)-30))
                population.append(Member(deltaX, deltaY, fitness, distance))
                print(fitness)
            
            elif len(population) < N and not first_gen:  # once we have a set of delta values for children, we need to find their fitness values
                delta_values = children.pop()
                deltaX = delta_values[0]
                deltaY = delta_values[1]
                if deltaX < 20: deltaX = 10
                if deltaY < 20: deltaY = 10
                fitness = calculate_fitness(time_since_enter, distance, (euclid(start.point, goal.point)-30))
                population.append(Member(deltaX, deltaY, fitness, distance))
                print(fitness)
            else:  # generating new children
                while len(children) < N:
                    parent_1 = tournament_selection()
                    parent_2 = tournament_selection()
                    
                    child = generate_child(parent_1, parent_2)
                    
                    children.append(child)
                population.clear()  # the older generation is discarded and will be replaced by its children

        for event in pygame.event.get():  # User did something

            # print(event.type)
        
            if event.type == 5 and status == "gen_obstacles":  # if the user has clicked, and the current status is gen_obstacles, a rect will be drawn at onclick location
                if pygame.mouse.get_pos()[0] > 100 or pygame.mouse.get_pos()[1] > 50:
                    generate_obstacle(pygame.mouse.get_pos())

            elif event.type == 5 and status == "set_coordinates":  # user has clicked, status is set_coordinates, start and goal points will be chosen based on next two onclick actions
                mouse_pos = pygame.mouse.get_pos()
                if not start_set:
                    if not collides(mouse_pos):
                        start = Node(mouse_pos, None)
                        start_set = True
                        nodes.append(start)
                        pygame.draw.circle(screen, Color("red"), mouse_pos, 10, 0)  # Surface, color, pos, radius, width=0
                elif not goal_set:
                    if not collides(mouse_pos):
                        goal = Node(mouse_pos, None)
                        goal_set = True
                        pygame.draw.circle(screen, Color("green"), mouse_pos, 10, 0)  # Surface, color, pos, radius, width=0
                        theta_goal = goal_direction(start.point, goal.point)
                        start_time = pygame.time.get_ticks()
                        status = "traverse"
                        deltaX = WINDOW_X
                        deltaY = WINDOW_Y

            elif event.type == pygame.QUIT: # exit has been clicked
                done = True #breaks out of pygame loop and quits program

        for obs in obstacles: #drawing obstacles
            pygame.draw.rect(screen, Color("black"), obs)

        pygame.display.update() #updating display to show drawn items, must come after all draw actions
        
        if show_path: 
            pygame.time.delay(1500)
            reset(start.point,goal.point)
            show_path = False

        # fpsClock.tick(10000)
        
    pygame.quit()


class Member(object): #member stores rrt characteristics for genetic modification
    def __init__(self, deltaX, deltaY, fitness, distance):
        super(Member, self).__init__()
        self.deltaX = deltaX
        self.deltaY = deltaY
        self.fitness = fitness
        self.distance = distance


class Node(object): #node object stores its coordinates and its parent node
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent
        

if __name__ == '__main__':
    rrt()