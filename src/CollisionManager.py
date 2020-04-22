# @author: Matheus Kunnen Ledesma
import numpy as np
import random as rand
import math
import time

from Body import Body

class CollisionManager:

    # General parameters
    CUBE_LENGHT = 10.
    SPHERE_NORMAL_COLOR = np.array([.1, .1, .1, 1.])
    SPHERE_NORMAL_DIFFUSE_COLOR = np.array([.5, .5, .5, 1.])
    SPHERE_COLLISION_COLOR = np.array([.1, .01, .01, 1.])
    SPHERE_COLLISION_DIFFUSE_COLOR = np.array([.5, .25, .25, 1.])

    def __init__(self):
        self.n_bodies = 100
        self.b_radius = (self.CUBE_LENGHT / (2. * self.n_bodies))*2
        self.vel_limits = np.array([-3., 3.])
        self.init_bodies()
    
    def init_bodies(self):
        self.bodies_l = []
        # def __init__(self, b_id, b_radius, b_pos, b_vel):
        # self.bodies_l.append(Body(1, .1, [0., 0., 0.], [0., 0., 0.,]))
        count = 0
        #b_radius = CollisionManager.CUBE_LENGHT/self.n_bodies
        self.min_d = self.b_radius * 2.

        pos_aux = -CollisionManager.CUBE_LENGHT/2. + self.min_d
        K = 5
        p = [pos_aux, pos_aux, pos_aux]
        dp = [self.min_d*K, self.min_d*K, self.min_d*K]
        while count < self.n_bodies:
            pos = np.array(p)
            vel = np.array([rand.randrange(self.vel_limits[0], self.vel_limits[1]),
                              rand.randrange(self.vel_limits[0], self.vel_limits[1]),
                              rand.randrange(self.vel_limits[0], self.vel_limits[1])])
            print("POS ", pos, " VEL", vel)
            # def __init__(self, b_id, b_radius, b_pos, b_vel):
            self.bodies_l.append(Body(count, self.b_radius, pos, vel, CollisionManager.CUBE_LENGHT/2.))

            p[0] += dp[0]
            if abs(p[0] + dp[0]) > abs(CollisionManager.CUBE_LENGHT/2 - 3*self.b_radius):
                dp[0] *= -1
                p[1] += dp[1] 
                if abs(p[1]) > abs(CollisionManager.CUBE_LENGHT/2 - 3*self.b_radius):
                    dp[1] *= -1
                    p[2] += dp[2]
            count += 1

    def update(self, dt):
        for body in self.bodies_l:
            body.update(dt)

    def draw(self, g_manager):
        g_manager.draw_cube_frame(CollisionManager.CUBE_LENGHT)
        for body in self.bodies_l:
            print("POS ", body.b_pos, " VEL ", body.b_vel)
            body.draw(g_manager, CollisionManager.SPHERE_NORMAL_COLOR, CollisionManager.SPHERE_COLLISION_COLOR, CollisionManager.SPHERE_NORMAL_COLOR)