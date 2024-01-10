# TODO: Find a graphics lib that would allow me to render all the pixels from the format.
# Additionally, we want to be able to add the option to zoom, pull up a grid, and bring up 
# a number on the bottom and side to indiciate what pixel we're looking at.
#
# Arguably, we'll need a library that is very flexible and isn't specialized on only one specific
# type of data visualization (e.g. MatLab). 
#
# Potential canidates:
# - PyGame (Handles Mouse and Keyboard Input, Graphics, Sound, etc...) 
# 

# TODO: Write a requirements.txt and move the data visualizer into it's own separate folder.
# Also, maybe write up quick little documentation that provides details on how to use it, 
# what features it includes, etc.

import pygame
import pygame_widgets as pw
from pygame_widgets.button import Button
import numpy as np
import random
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog as fd

filetypes = (
    ('map files', '*.map_data'),
    ('All files', '*.*')
)

filename = fd.askopenfilename(
    title='Open a file',
    initialdir='.',
    filetypes=filetypes)

files_loaded = []

map_data = []
map_width = 2
map_height = 2
old_width = map_width
old_height = map_height

pixels = np.array([[(0, 0, 0), (0, 0, 0)], [(0, 0, 0), (0, 0, 0)]], dtype=np.uint8)
pixel_color = (random.randint(128, 255), random.randint(128, 255), random.randint(128, 255))

colors_used = []

# This function sucks, but I don't care.
def write_to_pixels(file_path: str):
    global pixel_color
    global pixels
    global colors_used
    global map_width
    global map_height
    global old_width
    global old_height
    
    pixel_color = (random.randint(128, 255), random.randint(128, 255), random.randint(128, 255))
    
    unique_color_picked = False
    
    while not unique_color_picked:
        found_match = False
        for color in colors_used:
            found_match = found_match and color == pixel_color
        
        if not found_match:
            unique_color_picked = True
        else:
            pixel_color = (random.randint(128, 255), random.randint(128, 255), random.randint(128, 255))
    
    colors_used.append(pixel_color)
    
    old_width = map_width
    old_height = map_height
    
    map_data = []
    
    with open(file_path) as f:
        full_map_data = f.read()
        
        map_width = int(full_map_data[5:(full_map_data.find("x"))])
        map_height = int(full_map_data[(full_map_data.find("x")+1):(full_map_data.find("\n"))])
        
        str_map_data = full_map_data[(full_map_data.find("\n")+1):]
        
        map_data = np.frombuffer(str_map_data.encode(), dtype=np.uint8)
        
        print("The map width is", map_width, "and the map height is", map_height)
    
    if old_height < map_height:
        for _ in range(0, map_height-old_height):
            pixels = np.append(pixels, [[(0, 0, 0)] for i in range(0, old_width)], 1)
    
    if old_width < map_width:
        for _ in range(0, map_width-old_width):
            pixels = np.append(pixels, [[(0, 0, 0) for i in range(0, map_height)]], 0)
    
    for i in range(0, map_height):
        for j in range(0, map_width):
            if bool(map_data[i * map_width + j]): 
                pixels[j, i] = pixel_color  

def clear_pixels():
    global pixels
    
    pixels[:] = (0, 0, 0)

# clear_pixels()

# choosen_map_file_to_load = input("Please choose a file to visualize (e.g. 'example.map_data'):").strip()

write_to_pixels(filename)

files_loaded.append(filename)

# clear_pixels()
# write_to_pixels("example.map_data")

# clear_pixels()
# write_to_pixels("example.map_data")

# clear_pixels()

WINDOW_WIDTH  = 1280
WINDOW_HEIGHT = 720

pygame.init()
window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Path Util Map Visualizer")

image = pygame.surfarray.make_surface(pixels)

rect = image.get_rect()

# rect.width = WINDOW_WIDTH - map_width 
# rect.height = WINDOW_HEIGHT - map_height
rect.center = (WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2)

speed = 5

clock = pygame.time.Clock()

buttons_to_run = []

def load_file():
    global filename
    global files_loaded
    global filetypes
    global updated_files
    global rect
    global image
    
    filename = fd.askopenfilename(
        title='Open a file',
        initialdir='.',
        filetypes=filetypes)
    
    if filename != "":
        write_to_pixels(filename)
        
        files_loaded.append(filename)
        image = pygame.surfarray.make_surface(pixels)
    
        previous_x = rect.x
        previous_y = rect.y
    
        rect = image.get_rect()
        
        rect.x = previous_x
        rect.y = previous_y

buttons_to_run.append(Button(
    window,
    30, 30, 120, 75,
    text="Load Map",
    inactiveColour=(255, 0, 0),
    pressedColour=(0, 255, 0),
    radius = 20,
    onClick = load_file,
))

running = True

while running:
    clock.tick(60)

    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            running = False
            break
    pw.update(events)
    
    for button in buttons_to_run:
        button.listen(events)

    keys = pygame.key.get_pressed()
    
    # Keeping these just incase we want a zoom feature.
    if keys [pygame.K_LEFT]:
        rect.x += speed
    if keys [pygame.K_RIGHT]:
        rect.x -= speed
    if keys [pygame.K_UP]:
        rect.y += speed
    if keys [pygame.K_DOWN]:
        rect.y -= speed
        
    if keys [pygame.K_ESCAPE]:
        running = False
        break

    # rect.x = max (0, min (rect.x, window.get_width () - rect.width))
    # rect.y = max (0, min (rect.y, window.get_height () - rect.height))

    window.fill((123, 123, 123))
    
    window.blit(image, rect)

    for button in buttons_to_run:
        button.draw()
    
    pygame.display.flip()
    pygame.display.update()

pygame.quit()
