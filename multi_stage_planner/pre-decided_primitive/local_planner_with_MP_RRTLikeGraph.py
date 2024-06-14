import numpy as np
import heapq
import cv2
import time
import motion_primitive_discrete
import pygame
import matplotlib.pyplot as plt
from math import cos,sin
# dx1 = 5
# dy1 = 5
# dx2 = 3
# dy2 = 4
# dtheta = 15
# def neighbors_ mp(x,y,theta):
#     forward = ( x + dx1*cos(theta), y + dy2*sin(theta), theta )
#     fwd_t =  ( x+dx2*cos(theta) ,y , theta + dtheta)
#     return None #should return neighbour in the form (x,y,theta)
XDIM = 795
YDIM = 661
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 5000

def obstacle_check():
    pass

def astar(map_array, start, goal):
    #Pygame for graph visualization:
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    img = pygame.image.load('map_new.jpeg')
    img.convert()

    # pygame.display.set_caption('RRT      S. LaValle    May 2011')
    white = 255, 240, 200
    black = 20, 20, 40
    RED = (255, 0, 0)
    screen.fill(black)
    rect = img.get_rect()
    rect.center = XDIM // 2, YDIM // 2
    screen.blit(img, rect)
    pygame.draw.rect(screen, RED, rect, 1)
    pygame.display.update()

    #A*
    start_t = time.time_ns()
    height, width = map_array.shape

    def heuristic(node):
        return np.linalg.norm(np.array(node) - np.array(goal[:2]))



    # Initializing
    queue = []
    cost = {start: 0}
    heapq.heappush(queue, (0, start))
    parent = {start: None}

    while queue:
        _, current = heapq.heappop(queue)
        if parent[current] == None:
            pass
        else:
            #pass
            pygame.draw.line(screen, black, parent[current][:2], current[:2])
            pygame.display.update()

        if current == goal :
            break

        if heuristic(current[:2]) < 10 and current[2] == goal[2]:
            goal = current
            print('thresholded')
            break

        neighbors = []

        x, y, theta = current

        neighbors = motion_primitive_discrete.value_intialisation(x,-y,theta)


        for neighbor in neighbors:
            # if map_array[neighbor[0]][neighbor[1]] == 0:
            #     continue
            # Calculate the cost of reaching the neighbor
            if 0 <= int(neighbor[0]) < height and 0 <= int(neighbor[1]) < width:
                #Check if obstacle
                if map_array[int(neighbor[1])][int(neighbor[0])] < 215:
                    continue
            #print(neighbor)
            neighbor_cost = cost[current] + np.linalg.norm(np.array(neighbor[:2]) - np.array(current[:2])) + (
                        255 - map_array[int(neighbor[1])][int(neighbor[0])]) * 4 / 255

            if neighbor not in cost or neighbor_cost < cost[neighbor]:
                # g
                cost[neighbor] = neighbor_cost
                # f = g+h
                priority = neighbor_cost + heuristic(neighbor[:2])
                # adding new nodes to the q
                heapq.heappush(queue, (priority, neighbor))
                # connect node to its parent
                parent[neighbor] = current
                #print('queue',queue)

    # Retrieve the path from the goal to the start
    path = []
    current = goal
    flag = 1
    while current:
        path.append(current)
        current = parent.get(current)
    path.reverse()
    print(time.time_ns() - start_t)
    return path

def mouse_callback(event, x, y, flags, param):
    global start_point, goal_point

    if event == cv2.EVENT_LBUTTONDOWN:
        if start_point is None:
            start_point = (x, y, theta1)
            print("Start point selected: ({}, {})".format(x, y))

        else:
            goal_point = (x, y, theta2)
            print("Goal point selected: ({}, {})".format(x, y))


if __name__ == '__main__':
    image = cv2.imread('map_new.jpeg')
    # Use the cvtColor() function to grayscale the image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # cv2_imshow( gray_image)
    # Apply binary thresholding to create a binary image
    threshold_value = 210
    max_value = 255
    ret, binary_image = cv2.threshold(gray_image, threshold_value, max_value, cv2.THRESH_BINARY)
    im = np.asarray(binary_image)
    im_gray = np.asarray(gray_image)
    # box_blur_ker = (1/25)*np.array([[1,1,1,1,1],[1,1,1,1,1],[1,1,1,1,1],[1,1,1,1,1],[1,1,1,1,1]])
    box_blur_ker = (1 / 225) * np.ones((15, 15))

    Box_blur = cv2.filter2D(src=im, ddepth=-1, kernel=box_blur_ker)
    # cv2_imshow(Box_blur)
    imblur = np.asarray(Box_blur)

    map_array = imblur

    theta1 = 0
    theta2 = 270



    map_image = cv2.imread('map_new.jpeg')

    # Create a window to display the image
    cv2.namedWindow('Map')
    cv2.imshow('Map', map_image)

    # Initialize the start and goal points as None
    start_point = None
    goal_point = None

    # Set the mouse callback function
    cv2.setMouseCallback('Map', mouse_callback)

    # Wait for the user to select the start and goal points
    while start_point is None or goal_point is None:
        cv2.waitKey(1)

    # Close the image window
    cv2.destroyAllWindows()

    # Print the selected start and goal points
    # (195, 577,theta1),(111, 577,theta2)

    start_point = (127, 223, theta1)
    goal_point = (475, 259, theta2)
    path = astar(map_array, start_point, goal_point)
    print(path)
    img = plt.imread("map_new.jpeg")
    fig, ax = plt.subplots()
    x = range(300)
    ax.imshow(img, extent=[0, 795, -661, 0])
    X_final_path = []
    Y_final_path = []
    for i, j, k in path:
        X_final_path.append(i)
        Y_final_path.append(-j)
    plt.scatter(X_final_path, Y_final_path)
    plt.show()

# Load the map image
