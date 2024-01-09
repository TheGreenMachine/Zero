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

import pygame

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

player_pos = pygame.Vector2(screen.get_width() / 2, screen.get_height() / 2)

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("purple")

    pygame.draw.circle(screen, "red", player_pos, 40)

    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        player_pos.y -= 300 * dt
    if keys[pygame.K_s]:
        player_pos.y += 300 * dt
    if keys[pygame.K_a]:
        player_pos.x -= 300 * dt
    if keys[pygame.K_d]:
        player_pos.x += 300 * dt

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()