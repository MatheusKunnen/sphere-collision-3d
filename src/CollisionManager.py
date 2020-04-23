# @author: Matheus Kunnen Ledesma
import numpy as np
import random as rand
import math
import time

from Body import Body

class CollisionManager:

    # General parameters
    CUBE_LENGHT = 10.
    D_K = .5
    MB_ROUND = 0
    SPHERE_NORMAL_COLOR = np.array([.1, .1, .1, 1.])
    SPHERE_NORMAL_DIFFUSE_COLOR = np.array([.5, .5, .5, 1.])
    SPHERE_COLLISION_COLOR = np.array([.1, .01, .01, 1.])
    SPHERE_COLLISION_DIFFUSE_COLOR = np.array([.5, .25, .25, 1.])

    def __init__(self):
        self.n_bodies = 250
        self.b_radius = math.sqrt(self.CUBE_LENGHT**2 /self.n_bodies) * CollisionManager.D_K
        print(self.b_radius)
        self.min_d = self.b_radius * 2.
        self.vel_limits = np.array([-3., 3.])
        self.init_bodies()
    
    def init_bodies(self):
        self.bodies_l = []
        count = 0
        pos_aux = -CollisionManager.CUBE_LENGHT/2. + self.min_d
        K = 2.
        p = [pos_aux, pos_aux, pos_aux]
        dp = [self.min_d*K, self.min_d*K, self.min_d*K]
        while count < self.n_bodies:
            pos = np.array(p)
            vel = np.array([rand.randrange(self.vel_limits[0], self.vel_limits[1]),
                              rand.randrange(self.vel_limits[0], self.vel_limits[1]),
                              rand.randrange(self.vel_limits[0], self.vel_limits[1])])
            self.bodies_l.append(Body(self.b_radius, pos, vel))

            p[0] += dp[0]
            if abs(p[0] + dp[0]) > abs(CollisionManager.CUBE_LENGHT/2 - 3*self.b_radius):
                dp[0] *= -1
                p[1] += dp[1] 
                if abs(p[1]) > abs(CollisionManager.CUBE_LENGHT/2 - 3*self.b_radius):
                    dp[1] *= -1
                    p[2] += dp[2]
            count += 1
    
    def check_collisions(self):
        for a in range(0, len(self.bodies_l)):
            body_a = self.bodies_l[a]
            # Checks collision with walls of the cube
            self.check_wall_collision(body_a)
            for b in range(a+1, len(self.bodies_l)):
                body_b = self.bodies_l[b]
                r_ab = body_b.b_pos - body_a.b_pos
                if r_ab[0] <= self.min_d:
                    if self.norm_2(r_ab) <= self.min_d*self.min_d:
                        self.on_collision(body_a, body_b)
                else:
                    break

    def check_wall_collision(self, body):
        self.sort_bodies()
        for i in range(0, len(body.b_pos)):
            if abs(body.b_pos[i]) + body.b_radius >= CollisionManager.CUBE_LENGHT/2.:
                body.b_collisioned = True
                body.b_vel[i] *= -1
                body.b_pos[i] = CollisionManager.CUBE_LENGHT/2. - body.b_radius if body.b_pos[i] > 0 else - CollisionManager.CUBE_LENGHT/2. + body.b_radius

    def on_collision(self, sphere_a, sphere_b):
        # Set Spheres status
        sphere_a.b_collisioned = True
        sphere_b.b_collisioned = True
        # Collision direction
        r_ab = sphere_a.b_pos - sphere_b.b_pos
        r_normal = self.direction(r_ab)
        # Vel in collision direction
        v_ab = sphere_a.b_vel - sphere_b.b_vel
        v_normal = np.dot(v_ab, r_normal) * r_normal
        # Updates velocities
        sphere_a.b_vel = sphere_a.b_vel - v_normal
        sphere_b.b_vel = sphere_b.b_vel + v_normal
        # Separate spheres
        r_sep = r_normal * (self.norm(r_ab) - self.min_d) * 0.5
        sphere_a.b_pos = sphere_a.b_pos - r_sep 
        sphere_b.b_pos = sphere_b.b_pos + r_sep 

    def sort_bodies(self):
        self.bodies_l.sort(key=lambda body: body.b_pos[0])

    def update(self, dt):
        for body in self.bodies_l:
            body.update(dt)
        self.check_collisions()

    def draw(self, g_manager):
        g_manager.draw_cube_frame(CollisionManager.CUBE_LENGHT)
        for body in self.bodies_l:
            body.draw(g_manager, CollisionManager.SPHERE_NORMAL_COLOR, CollisionManager.SPHERE_COLLISION_COLOR)
    
    def get_v_rms(self):
        v_rms = 0.
        for body in self.bodies_l:
            v_rms += self.norm_2(body.b_vel)
        return math.sqrt(v_rms/self.n_bodies)
    
    def get_v_med(self):
        v_med = np.array([0., 0., 0.])
        for body in self.bodies_l:
            v_med = v_med + body.b_vel
        return v_med/self.n_bodies

    def get_k_v(self):
        k_v = np.array([0., 0., 0.])
        for body in self.bodies_l:
            k_v = k_v + body.get_k_v()
        return k_v
    
    def get_mb_dist(self):
        vel_l = []
        max_vel = int(self.norm(self.bodies_l[0].b_vel))
        for body in self.bodies_l:
            vel = int(self.norm(body.b_vel))
            vel_l.append(vel)
            max_vel = max_vel if max_vel > vel else vel
        distribution = []
        distribution.append(np.array([0, 0]))
        for i in range(1, max_vel+1):
            count = 0
            for j, vel in enumerate(vel_l):
                if vel == i:
                    count+= 1
                    vel_l.pop(j)
            distribution.append(np.array([i, count]))
        return distribution

    def direction(self, vec):
        return np.array(vec)/self.norm(vec) if self.norm(vec) != 0 else np.array([0., 0., 0.])

    def norm_2(self, vec):
        return (vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])
    
    def norm(self, vec):
        return math.sqrt(self.norm_2(vec))