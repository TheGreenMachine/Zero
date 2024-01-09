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

print("Sorry, the visualizer is not operational as of now!")

# import pygame and numpy modules
import pygame
import numpy as np

map_data = []
map_width = 0
map_height = 0

choosen_map_file_to_load = input("Please choose a file to visualize (e.g. 'example.map_data'):").strip()

with open(choosen_map_file_to_load) as f:
    full_map_data = f.read()
    
    map_width = int(full_map_data[5:(full_map_data.find("x"))])
    map_height = int(full_map_data[(full_map_data.find("x")+1):(full_map_data.find("\n"))])
    
    str_map_data = full_map_data[(full_map_data.find("\n")+1):]
    
    map_data = np.frombuffer(str_map_data.encode(), dtype=np.uint8)
    
    print("The map width is", map_width, "and the map height is", map_height)

# create a 2x2 array of white pixels
pixels = np.full ((map_width, map_height, 3), 0, dtype=np.uint8)

for i in range(0, map_height):
    for j in range(0, map_width):
        pixel_color = map_data[i * map_width + j] * 255
        pixels[j, i] = (pixel_color, pixel_color, pixel_color)

WINDOW_WIDTH  = 1280
WINDOW_HEIGHT = 720

# initialize pygame and create a window
pygame.init ()
window = pygame.display.set_mode ((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption ("Pygame Example")

# create a surface from the pixel array
image = pygame.surfarray.make_surface (pixels)

# create a rectangle object for the image
rect = image.get_rect ()

rect.center = (WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2)
rect.size = (WINDOW_WIDTH, WINDOW_HEIGHT)

speed = 5

clock = pygame.time.Clock ()

running = True

while running:
    clock.tick (60)

    for event in pygame.event.get ():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed ()

    if keys [pygame.K_LEFT]:
        rect.x -= speed
    if keys [pygame.K_RIGHT]:
        rect.x += speed
    if keys [pygame.K_UP]:
        rect.y -= speed
    if keys [pygame.K_DOWN]:
        rect.y += speed
        
    if keys [pygame.K_ESCAPE]:
        running = False
        break

    rect.x = max (0, min (rect.x, window.get_width () - rect.width))
    rect.y = max (0, min (rect.y, window.get_height () - rect.height))

    window.fill ((123, 123, 123))

    window.blit (image, rect)

    pygame.display.flip ()

# quit pygame and exit the program
pygame.quit ()
