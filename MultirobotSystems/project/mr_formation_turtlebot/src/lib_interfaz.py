#!/usr/bin/env python

# Importing necessary libraries
import pygame
import os

# Path to current file directory
current_file_path = os.path.dirname(os.path.abspath(__file__))

# Window and grid dimensions
GRID_SIZE   = GRID_WIDTH,  GRID_HEIGHT    = [600,400]
WINDOW_SIZE = WINDOW_WIDTH, WINDOW_HEIGHT = [650,600]

# Panel dimensions and position
PANEL_L             = 0
PANEL_T             = 500
PANEL_WIDTH         = WINDOW_WIDTH
PANEL_HEIGHT        = WINDOW_HEIGHT - PANEL_T
PANEL_BORDER_WIDTH  = 0

# Cell dimensions and border width
CELL_W              = 10
CELL_H              = 10
CELL_BORDER_WIDTH   = 1

# Border dimensions and position
BORDER_L            = 0
BORDER_T            = 0
BORDER_WIDTH        = WINDOW_WIDTH
BORDER_HEIGHT       = WINDOW_HEIGHT - PANEL_HEIGHT
BORDER_BORDER_WIDTH = 15

# Grid position and size
GRID_L              = 75 #125
GRID_W              = 575 #525
GRID_T              = 80
GRID_H              = 480

# Super grid dimensions
SUPER_GRID_WIDTH    = 50
SUPER_GRID_HEIGHT   = 40

# Number of turtles
NUM_TURTLES = 10

# Configuration files paths
config_loc      = current_file_path + "/icons/"
interface_caption  = "Formation creation"
interface_icon     = config_loc + "icon.png"
interface_title    = config_loc + "title.png"
interface_robot_title   = config_loc + "numbers.png"
interface_info     = config_loc + "instructions.png"
interface_axes     = config_loc + "coordinates.png"

# Color definitions
white           = [255,255,255]
light_white     = [200,200,200]
superlightgrey  = [50,50,50]
lightgrey       = [30,25,25]
grey            = [18,18,18]
black           = [0,0,0]
lightblue       = [100,100,255]

# Variables for grid state, solution, and dictionary
grid_state      = []
path            = []
solution        = []
grid_dict       = {}

# Information text
info_text       = ["Instructions:", 
                    "Left-click to draw",  
                    "Right-click to erase", 
                    "Press 'Draw' to generate robot points", 
                    "Press 'Send' to start robot movement"]

# Point class definition
class Point: 
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Button class definition
class Button:
    def __init__(self, text, width, height, pos, elevation, font, color_on, color_off):
        # Core attributes 
        self.pressed = False
        self.elevation = elevation
        self.dynamic_elevation = elevation
        self.original_y_pos = pos[1]
        self.color_on = color_on
        self.color_off = color_off

        # Top rectangle 
        self.top_rect = pygame.Rect(pos,(width,height))
        self.top_color = self.color_off

        # Bottom rectangle 
        self.bottom_rect = pygame.Rect(pos,(width,height))
        self.bottom_color = '#354B5E'

        # Text
        self.text_surf = font.render(text,True,'#FFFFFF')
        self.text_rect = self.text_surf.get_rect(center=self.top_rect.center)

    def draw(self, screen, function):
        # Elevation logic 
        self.top_rect.y = self.original_y_pos - self.dynamic_elevation
        self.text_rect.center = self.top_rect.center 

        self.bottom_rect.midtop = self.top_rect.midtop
        self.bottom_rect.height = self.top_rect.height + self.dynamic_elevation

        pygame.draw.rect(screen, self.bottom_color, self.bottom_rect, border_radius=12)
        pygame.draw.rect(screen, self.top_color, self.top_rect, border_radius=12)
        screen.blit(self.text_surf, self.text_rect)
        self.check_click(function)

    def check_click(self, function):
        mouse_pos = pygame.mouse.get_pos()
        if self.top_rect.collidepoint(mouse_pos):
            self.top_color = self.color_on
            if pygame.mouse.get_pressed()[0]:
                self.dynamic_elevation = 0
                self.pressed = True
            else:
                self.dynamic_elevation = self.elevation
                if self.pressed == True:
                    function()
                    self.pressed = False
        else:
            self.dynamic_elevation = self.elevation
            self.top_color = self.color_off
