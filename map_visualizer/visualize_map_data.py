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

# Maybe we should limit the amount of layers to something like 10? 
# That would allow me to make unique colors manually that are distinct and mix well.


import pygame
import pygame_widgets as pw
from pygame_widgets.button import Button, ButtonArray
from pygame_widgets.dropdown import Dropdown
import numpy as np
import random
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog as fd
import copy

filetypes = (
    ('map files', '*.map_data'),
    ('All files', '*.*')
)

# filename = fd.askopenfilename(
#     title='Open a file',
#     initialdir='.',
#     filetypes=filetypes)

filename = ""

files_loaded = []
files_blobbed = []

map_data = []
map_width = 2
map_height = 2
old_width = map_width
old_height = map_height

pixels = np.array([[(0, 0, 0), (0, 0, 0)], [(0, 0, 0), (0, 0, 0)]], dtype=np.uint8)
pixel_color = (random.randint(128, 255), random.randint(128, 255), random.randint(128, 255))

colors_used = []

def clamp(n: int, smallest: int, largest: int) -> int: 
    return max(smallest, min(n, largest))

# This function sucks, but I don't care.
def write_to_pixels(file_path: str, blob: bool = False):
    global pixel_color
    global pixels
    global colors_used
    global map_width
    global map_height
    global old_width
    global old_height
    
    pixel_color = (random.randint(128, 255), random.randint(128, 255), random.randint(128, 255))
    
    if file_path not in files_loaded:
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
    else:
        pixel_color = colors_used[files_loaded.index(file_path)]
    
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
                if blob:
                    # A stupid piece of code that makes the pixels wider.
                    pixels[clamp(j-1, 0, map_width), i] += pixel_color
                    pixels[clamp(j+1, 0, map_width), i] += pixel_color
                    pixels[j, clamp(i-1, 0, map_height)] += pixel_color
                    pixels[j, clamp(i+1, 0, map_height)] += pixel_color
                    pixels[clamp(j-1, 0, map_width), clamp(i-1, 0, map_height)] += pixel_color
                    pixels[clamp(j+1, 0, map_width), clamp(i-1, 0, map_height)] += pixel_color
                    pixels[clamp(j-1, 0, map_width), clamp(i+1, 0, map_height)] += pixel_color
                    pixels[clamp(j+1, 0, map_width), clamp(i+1, 0, map_height)] += pixel_color
                    pixels[clamp(j-2, 0, map_width), i] += pixel_color
                    pixels[clamp(j+2, 0, map_width), i] += pixel_color
                    pixels[j, clamp(i-2, 0, map_height)] += pixel_color
                    pixels[j, clamp(i+2, 0, map_height)] += pixel_color
                    
                # might not want to add the pixel colors together.
                pixels[j, i] += pixel_color  

def clear_pixels():
    global pixels
    
    pixels[:] = (0, 0, 0)

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
layer_buttons = []
blob_buttons = []

files_enabled = []

def reload_pixels():
    global files_enabled
    global files_loaded
    global image
    global rect
    
    clear_pixels()
    
    for idx, enabled in enumerate(files_enabled):
        layer_buttons[idx].setText(f"Layer: {idx} X")
    
        if enabled:
            layer_buttons[idx].setText(f"Layer: {idx} O")
            write_to_pixels(files_loaded[idx], files_blobbed[idx])
    
    for idx, blobbed in enumerate(files_blobbed):
        blob_buttons[idx].setText("Blob X")
        
        if blobbed:
            blob_buttons[idx].setText("Blob O")
                    
    image = pygame.surfarray.make_surface(pixels)
    
    previous_x = rect.x
    previous_y = rect.y

    rect = image.get_rect()
    
    rect.center = (WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2)
    
    rect.x = previous_x
    rect.y = previous_y

def enable_or_disable_filename(idx: int):
    global files_enabled
    
    files_enabled[idx] = not files_enabled[idx]
    
    reload_pixels()

def blob_filename(idx: int):
    global files_blobbed
    
    files_blobbed[idx] = not files_blobbed[idx]
    
    reload_pixels()

def load_file():
    global filename
    global files_loaded
    global files_enabled
    global files_blobbed
    global filetypes
    global updated_files
    global rect
    global image
    global layer_buttons
    global blob_buttons
    
    filename = fd.askopenfilename(
        title='Open a file',
        initialdir='.',
        filetypes=filetypes)
    
    if filename != "" and filename not in files_loaded:
        write_to_pixels(filename)
        
        files_loaded.append(filename)
        files_enabled.append(True)
        files_blobbed.append(False)
        image = pygame.surfarray.make_surface(pixels)
    
        previous_x = rect.x
        previous_y = rect.y
    
        rect = image.get_rect()
        
        rect.center = (WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2)
        
        rect.x = previous_x
        rect.y = previous_y
        
        layer_buttons = []
        blob_buttons = []
        
        for idx, file_path in enumerate(files_loaded):
            hover_color = colors_used[idx]
            
            hover_color = (hover_color[0] - 23, hover_color[1] - 23, hover_color[2] - 23)
             
            pressed_color = (hover_color[0] - 23, hover_color[1] - 23, hover_color[2] - 23) 
            
            layer_buttons.append(Button(
                window,
                15, 105 + 45 * (idx), 95, 35,
                text=f"Layer: {idx} O",
                inactiveColour=colors_used[idx],
                hoverColour=hover_color,
                pressedColour=pressed_color,
                onClick = lambda i=idx: enable_or_disable_filename(i)
            ))
            
            blob_buttons.append(Button(
                window,
                115, 105 + 45 * (idx), 95, 35,
                text = "Blob X",
                inactiveColour=colors_used[idx],
                hoverColour=hover_color,
                pressedColour=pressed_color,
                onClick = lambda i=idx: blob_filename(i)
            ))

button = Button(
    window,
    15, 15, 95, 35,
    text="Load Map",
    inactiveColour=(255, 0, 0),
    hoverColour=(128, 0, 0),
    pressedColour=(0, 255, 0),
    radius = 2,
    onClick = load_file,
)

def clear_layers():
    global files_loaded
    global files_enabled
    global files_blobbed
    global colors_used
    global layer_buttons
    global blob_buttons
    
    colors_used = []
    files_loaded = []
    files_enabled = []
    files_blobbed = []
    layer_buttons = []
    blob_buttons = []
    
    reload_pixels()

clear_layers_button = Button(
    window,
    WINDOW_WIDTH - 110, 15, 95, 35,
    text = "Clear layers",
    inactiveColour=(255, 255, 255),
    hoverColour=(175, 175, 175),
    pressedColour=(128, 128, 128),
    onClick = clear_layers,
)

running = True

while running:
    clock.tick(60)

    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            running = False
            break

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

    pw.update(events)
    pygame.display.flip()
    pygame.display.update()

pygame.quit()
