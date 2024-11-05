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
scale_factor = 10 ** -6
G = 6.673 * 10 ** -20

def get_accel(pos, particles, par):
    a = 0 
    for cur_par in particles:
        if cur_par != par:
            distance = abs(np.linalg.norm((pos - cur_par.pos)))
            pygame.time.wait(10)
            u_vec = (cur_par.pos - pos) / distance
            a += (G * cur_par.mass / distance ** 2) * u_vec
    return(a)

class particle:
    def __init__(self, mass, pos, color, vel, size):
        self.mass = mass
        self.pos = np.array(pos)
        self.color = color
        self.a = np.array([0., 0.])
        self.v = np.array(vel)
        self.r = size
   
def runge_kutta(cur_par, particles, a):
    k1v = get_accel(cur_par.pos, particles, cur_par)                   #use RK4 method
    k1r =  cur_par.v
    k2v = get_accel(cur_par.pos + (a/2) * k1r, particles, cur_par)            
    k2r =     cur_par.v + (a/2) * k1v            
    k3v = get_accel(cur_par.pos + (a/2) * k2r, particles, cur_par)            
    k3r =     cur_par.v + (a/2) * k2v          
    k4v = get_accel(cur_par.pos +  a    * k3r, particles, cur_par)                
    k4r =     cur_par.v +  a    * k3v
    dv = (a/6) * (k1v + 2*k2v + 2*k3v + k4v)
    dr = (a/6) * (k1r + 2*k2r + 2*k3r + k4r) 
    return(dv, dr)

def update(particles, delta):
    n = 3
    a = delta / n
    for cur_par in particles:
        for i in range(n):
            (dv, dr) = runge_kutta(cur_par, particles, a)
            cur_par.v += dv             #update v
            cur_par.pos += dr              #update r
        if (cur_par.pos[0] * scale_factor >= -1 * cur_par.r and
        cur_par.pos[0] * scale_factor <= SCREEN_WIDTH + cur_par.r and 
        cur_par.pos[1] * scale_factor >= -1 * cur_par.r and
        cur_par.pos[1] * scale_factor <= SCREEN_HEIGHT + cur_par.r):
            pygame.draw.circle(screen, cur_par.color, cur_par.pos * scale_factor, cur_par.r)
            
time = pygame.time.Clock()  
a_earth = abs(np.linalg.norm(1.496 * 10 ** 8))
a_venus = abs(np.linalg.norm(1.08 * 10 ** 8))
a_sat = abs(np.linalg.norm(6571))
sun = particle(1.989 * 10 ** 30, [750 * 10 ** 6., 400 * 10 ** 6.], [255, 0, 0], [0.,0.], 30)
earth = particle(5.972 * 10 ** 24, [sun.pos[0], sun.pos[1] + a_earth], [0, 255, 0], [(G*sun.mass / (a_earth)) ** 0.5,0.], 5)  
sat = particle(0, [earth.pos[0] + a_sat, earth.pos[1]], [0, 0, 255], [earth.v[0], earth.v[1] - (G*earth.mass / (a_sat)) ** 0.5], 2)
venus = particle(4.867 * 10 ** 24, [sun.pos[0], sun.pos[1] + a_venus], [0, 255, 255], [(G*sun.mass / (a_venus)) ** 0.5,0.], 8) 
burn = pygame.Rect(0,0,4,3)

t_trans = math.pi * ((((a_earth + a_venus) / 2) ** 3) / (sun.mass * G)) ** 0.5
t_earth = 2 * math.pi * (((a_earth) ** 3) / (sun.mass * G)) ** 0.5
t_venus = 2 * math.pi * (((a_venus) **  3) / (sun.mass * G)) ** 0.5
t_sat = 2 * math.pi * (((a_sat) **  3) / (earth.mass * G)) ** 0.5
sat_speed = 2 * math.pi / t_sat
nf = 2 * math.pi /  t_venus
theta = math.pi - nf * t_trans - 0.28
theta += 2 * math.pi
timer = 0
timerstart = 0
burn1 = 0
t_launch = theta / ((2 * math.pi / t_venus) - (2 * math.pi / t_earth))
theta = t_launch * 2 * math.pi / t_earth
print(sun.pos * scale_factor)
sat_s_theta = (t_launch * 2 * math.pi / t_sat) % (2 * math.pi)
opt_s_theta = theta - sat_s_theta + math.pi
sat.pos = np.array([earth.pos[0]+ (sat.pos[0] - earth.pos[0]) * math.sin(opt_s_theta), earth.pos[1] + (sat.pos[0] - earth.pos[0]) * math.cos(opt_s_theta)])
#sat_s_theta -= theta
sat_vel =  (G*earth.mass / (a_sat)) ** 0.5
sat.v = np.array([earth.v[0] + sat_vel * math.cos(opt_s_theta), earth.v[1] - sat_vel * math.sin(opt_s_theta)])
clr = [255,255,255]

v_inj = ((2 * sun.mass * G / a_earth) - (2 * G * sun.mass / (a_venus + a_earth))) ** 0.5 - (G * sun.mass / a_earth) ** 0.5
v_escape = (v_inj ** 2 + (2 * G * earth.mass / a_sat)) ** 0.5
v_burn1 = v_escape - (G * earth.mass / a_sat) ** 0.5
print(v_burn1)
start_time = 10
start = 0
while run:
    if timerstart == 0 and np.linalg.norm((earth.v / np.linalg.norm(earth.v)) - venus.v / np.linalg.norm(venus.v)) < 0.01:
        timerstart = 1
        clr = [0,255,0]
    delta = time.tick_busy_loop() / 1000
    print(delta)
    screen.fill(black)
    point = [earth.pos[0]+100 * math.sin(opt_s_theta), earth.pos[1] + 100 * math.cos(opt_s_theta)]
    point2 = [sun.pos[0]-1000 * math.sin(theta), sun.pos[1] - 1000 * math.cos(theta)]
    pygame.draw.line(screen, green, earth.pos, point)
    pygame.draw.line(screen, green, sun.pos, point2)
    update([sun, earth, sat, venus], delta)
    pygame.draw.line(screen, clr, earth.pos, sun.pos)
    if timerstart == 1:
        timer += delta
    if timer >= t_launch and burn1 == 0:
        sat.v -= v_burn1 * earth.v / np.linalg.norm(earth.v) 
        burn1 = 2
    if burn1 > 1:
        burn1 -= delta
        burn.centery = sat.pos[1]
        burn.right=sat.pos[0]
        pygame.draw.rect(screen, [255, 40, 40], burn)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    pygame.display.update()
