# @author: Matheus Kunnen Ledesma
from math import pow
import numpy as np

class Body:
    def __init__(self, b_radius, b_pos, b_vel):
        self.b_radius = b_radius
        self.b_pos = b_pos
        self.b_vel = b_vel
        self.b_collisioned = False

    def get_k_v(self):
        k_v = np.array([0., 0., 0.])
        for i in range(0, len(k_v)):
            k_v[i] += pow(self.b_vel[i], 2)/2
        return k_v
    
    def get_k(self):
        k = 0.
        for val in self.b_vel:
            k += pow(val, 2)
        return k

    def update(self, dt):
        self.b_collisioned = False
        self.b_pos += self.b_vel*dt

    def draw(self, g_manager, normal_color, collision_color):
        g_manager.draw_solid_sphere(self.b_pos, self.b_radius, normal_color if not self.b_collisioned else collision_color)


