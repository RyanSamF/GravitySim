import pygame
import random
import numpy as np
import math

pygame.init()
SCREEN_WIDTH = 1500.
SCREEN_HEIGHT = 800.
white = (255, 255, 255)
green = (0, 0, 255)
black = (0, 0, 0)
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
run = True
G = 1


class particle:
    def __init__(self, mass, pos, color, vel, size):
        self.mass = mass
        self.pos = np.array(pos)
        self.color = color
        self.a = np.array([0., 0.])
        self.v = np.array(vel)
        self.r = size
    def get_forces(self, particles):
        self.a = 0
        for cur_par in particles:
            if cur_par != self:
                distance = abs(np.linalg.norm((self.pos - cur_par.pos)))
                u_vec = (cur_par.pos - self.pos) / distance
                self.a += (G * cur_par.mass / distance ** 2) * u_vec

def update(particles, delta):
    for cur_par in particles:
        cur_par.get_forces(particles)
        cur_par.v += cur_par.a * delta
        cur_par.pos += cur_par.v * delta
        if (cur_par.pos[0] >= -1 * cur_par.r and
        cur_par.pos[0] <= SCREEN_WIDTH + cur_par.r and 
        cur_par.pos[1] >= -1 * cur_par.r and
        cur_par.pos[1] <= SCREEN_HEIGHT + cur_par.r):
            pygame.draw.circle(screen, cur_par.color, cur_par.pos, cur_par.r)
            
time = pygame.time.Clock()  
p1 = particle(500000, [750., 400.], [255, 0, 0], [0.,0.], 30)
p2 = particle(1000, [750., 700.], [0, 255, 0], [(G*p1.mass / (700 - p1.pos[1])) ** 0.5,0.], 10)  
p3 = particle(0, [770., 700.], [0, 0, 255], [p2.v[0], p2.v[1] + (G*p2.mass / (750 - p2.pos[1])) ** 0.5], 3)
p4 = particle(0, [750., 300.], [0, 255, 255], [-70.7106,0.], 8)      
while run:
    delta = time.tick_busy_loop() / 1000
    screen.fill(black)
    update([p1, p2, p3, p4], delta)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    pygame.display.update()
