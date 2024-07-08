#!/usr/bin/env python

from lib_interfaz import *
from geometry_msgs.msg import Polygon, Point32
import rospy
import numpy as np 
import pygame, sys

# Initializing ros node
rospy.init_node("interface_node")
num_robots = int(rospy.get_param("~n_robots"))

# Publisher for sending goal points
goal_publisher = rospy.Publisher('/goal_points', Polygon, queue_size=5)

# Publishers for sending desired positions to each turtle
desired_pos_publishers = [rospy.Publisher(f'/robot_{i}/desired_r', Point32, queue_size=10) for i in range(1, num_robots+1)]

# Initializing pygame
pygame.init()

# Initializing the main window
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption(interface_caption)
pygame.display.set_icon(pygame.image.load(interface_icon))

# Creating the panel for the buttons
panel   = pygame.Rect(PANEL_L, PANEL_T, PANEL_WIDTH, PANEL_HEIGHT)
border  = pygame.Rect(BORDER_L,BORDER_T,BORDER_WIDTH,BORDER_HEIGHT)

# Initializing the form and titles of the main window
title_rect = pygame.image.load(interface_title).get_rect()
robot_title_rect = pygame.image.load(interface_robot_title).get_rect()
info_rect  = pygame.image.load(interface_info).get_rect()
coordinate_rect = pygame.image.load(interface_axes).get_rect()

# Creating the buttons
btn_delete  = Button("REMOVE",  190,50,(15,520), 3,pygame.font.Font(None,20),superlightgrey,grey)
btn_draw    = Button("DRAW", 190,50,(230,520),3,pygame.font.Font(None,20),superlightgrey,grey)
btn_send    = Button("SEND",  190,50,(445,520),3,pygame.font.Font(None,20),superlightgrey,grey)

# Declaration and definition of the functions of the buttons and others
def initialize_grid():
    global grid_state
    for i in range(0,SUPER_GRID_WIDTH):
        row = []
        for j in range(0,SUPER_GRID_HEIGHT):
            row.append(False)
        grid_state.append(row)

def clear_grid():
    global grid_state, solution, grid_dict
    for i in range(len(grid_state)):
        for j in range(len(grid_state[i])):
            grid_state[i][j] = False
    solution = []
    grid_dict = {}

def draw_solution():
    global solution
    global path
    global grid_dict

    # Preprocessing of the grid dictionary
    keys = list(grid_dict.keys())
    key_points = []
    for i in keys:
        x = int(i/100)
        y = i%100
        key_points.append(Point(x,y))
    
    for k in key_points:
        print(k.x, k.y)

    solution = []
    if len(key_points) <= num_robots:
        solution = key_points.copy() 
    else:
        i = aux = (len(key_points) - (len(key_points) / num_robots)) / (num_robots - 2)
        solution.append(key_points[0])
        while(i <= len(key_points)):
            aux2 = int(i)
            solution.append(key_points[aux2])
            i = i + aux
        solution.append(key_points[-1])
        
    print("-----")
    print(len(solution), len(key_points), len(path))
    for i in solution:
        print(i.x, i.y)

def send_solution():
    poly = Polygon()
    for s in solution:
        poly.points.append(Point32(s.x*2/10,s.y*2/10,0))
        
    goal_publisher.publish(poly)

    # Calculate relative positions
    num_goals = len(solution)
    desired_pos = np.zeros((num_goals,2))
    rel_pos_matrix = np.zeros((num_goals, num_goals, 2))
    
    for i in range(num_goals):
        for j in range(num_goals):
            if i != j:
                rel_pos_matrix[i, j, 0] = (solution[j].x - solution[i].x) * 2/10
                rel_pos_matrix[i, j, 1] = (solution[j].y - solution[i].y) * 2/10
        
        desired_pos[i,0] = -np.sum(rel_pos_matrix[i,:,0])
        desired_pos[i,1] = -np.sum(rel_pos_matrix[i,:,1])
        desired_pos_publishers[i].publish(Point32(desired_pos[i,0], desired_pos[i,1], 0))
                
    rospy.loginfo("Desired relative positions in x: {}".format(rel_pos_matrix[:,:,0]))
    rospy.loginfo("Desired relative positions in y: {}".format(rel_pos_matrix[:,:,1]))

# Initializing the solution grid
initialize_grid()

# Infinite loop where the screen updates
while True:
    # First, check the event list
    for event in pygame.event.get():
        # Check if there's any QUIT event
        if event.type == pygame.QUIT: sys.exit()

    # Fill screen with background color, panel for buttons and border
    screen.fill(black)
    pygame.draw.rect(screen,lightgrey,panel, PANEL_BORDER_WIDTH)
    pygame.draw.rect(screen,lightgrey,border,BORDER_BORDER_WIDTH)

    # Placing the titles
    title_rect.x, title_rect.y    =   65, 10
    robot_title_rect.x, robot_title_rect.y    =   70, 55
    info_rect.x,  info_rect.y     =   20, 465
    coordinate_rect.x, coordinate_rect.y    =   -2, -2

    screen.blit(pygame.image.load(interface_axes),coordinate_rect)
    screen.blit(pygame.image.load(interface_title), title_rect)
    screen.blit(pygame.image.load(interface_info),  info_rect)
    
    # Fill the screen with the widgets and configuration
    btn_delete.draw(screen, clear_grid)
    btn_draw.draw(screen, draw_solution)
    btn_send.draw(screen, send_solution)

    # Drawing the grid
    for i in range(GRID_L,GRID_W,CELL_W):
        for j in range(GRID_T,GRID_H,CELL_H):
            # Drawing the main grid
            grid_rect = pygame.Rect(i,j,CELL_W,CELL_H)
            pygame.draw.rect(screen,lightgrey, grid_rect,CELL_BORDER_WIDTH)

            # Getting the mouse events in order to draw or delete the path - this path is saved at grid_state
            left_click = pygame.mouse.get_pressed()[0]
            right_click = pygame.mouse.get_pressed()[2] 
            if left_click or right_click:
                x,y = pygame.mouse.get_pos()
                if x > i and x < i+CELL_W and y > j and y < j+CELL_H:
                    if left_click:                  
                        grid_state[int((i-GRID_L)/CELL_W)][int((j-GRID_T)/CELL_H)] = True
                        grid_dict.update({int((i-GRID_L)/CELL_W)*100 + int((j-GRID_T)/CELL_H):True})
                    if right_click:
                        grid_state[int((i-GRID_L)/CELL_W)][int((j-GRID_T)/CELL_H)] = False
                        try:
                            grid_dict.pop(int((i-GRID_L)/CELL_W)*100 + int((j-GRID_T)/CELL_H))
                        except KeyError:
                            continue
                        solution = []

    # Drawing the selected grids (Done when right button of the mouse is pressed)
    path = []
    for i in range(0,SUPER_GRID_WIDTH):
        for j in range(0,SUPER_GRID_HEIGHT):
            if grid_state[i][j] == True: 
                rect = pygame.Rect(i*CELL_W+GRID_L,j*CELL_H+GRID_T,CELL_W,CELL_W)
                pygame.draw.rect(screen,white,rect,0)
                path.append(Point(i,j))

    # Drawing the points where the robots will move
    for s in solution:
        rect = pygame.Rect(s.x*CELL_W+GRID_L,s.y*CELL_H+GRID_T,CELL_W,CELL_H)
        pygame.draw.rect(screen,lightblue,rect,0)
    
    mx, my = pygame.mouse.get_pos()
    if mx > 20 and mx < 35 and my > 465 and my < 480:
        info_panel = pygame.Rect(25, 330, 520, 115)
        pygame.draw.rect(screen, light_white, info_panel,0)

        triangle_points = [(25, 445),(25, 462),(40, 445)]
        pygame.draw.polygon(screen, light_white, triangle_points, 0)

        for t in range(len(info_text)):
            font = pygame.font.Font(current_file_path + "/fonts/mytype.ttf",12)
            if t == 0:

                font.set_underline(True)
                font_surface = font.render(info_text[t],True,'#000000')
                font_text_rect = font_surface.get_rect()
                font_text_rect.x, font_text_rect.y = 200, 336 + t * 20

            else:

                font_surface = font.render(info_text[t],True,'#000000')
                font_text_rect = font_surface.get_rect()
                font_text_rect.x, font_text_rect.y = 29, 336 + t * 20

            screen.blit(font_surface, font_text_rect)
    # Updating the app
    pygame.display.flip()
