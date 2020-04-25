# @author: Matheus Kunnen Ledesma
import numpy as np
import random as rand
import math
import time

from Body import Body

class SimulationManager:

    # Constants
    K_BOLTZMANN = 1.3806488 # e-23 (Corrige ordem de grandeza para compensat velocidades "baixas")

    # General parameters
    D_CUBE_LENGHT = 10.
    D_K = .5
    MB_ROUND = 0
    SPHERE_NORMAL_COLOR = np.array([.1, .1, .1, 1.])
    SPHERE_NORMAL_DIFFUSE_COLOR = np.array([.5, .5, .5, 1.])
    SPHERE_COLLISION_COLOR = np.array([.1, .01, .01, 1.])
    SPHERE_COLLISION_DIFFUSE_COLOR = np.array([.5, .25, .25, 1.])

    def __init__(self, n_bodies = 4000):
        # Ambient Parameters
        self.cube_lenght = SimulationManager.D_CUBE_LENGHT
        self.cube_area = math.pow(self.cube_lenght, 2)
        self.cube_vol = math.pow(self.cube_lenght, 3)
        self.n_bodies = n_bodies
        self.b_radius = math.sqrt(self.D_CUBE_LENGHT**2 /self.n_bodies) * SimulationManager.D_K
        self.limit_d = self.D_CUBE_LENGHT/2. - self.b_radius
        self.min_d = self.b_radius * 2.
        self.vel_limits = np.array([-6., 6.])
        # self.max_vel = self.norm(np.array([self.vel_limits[1], self.vel_limits[1], self.vel_limits[1]]))
        # Simulation Variables
        self.n_wall_collisions = 0
        self.n_bodies_collision = 0
        self.dp_wall_bodies = 0.
        self.k_t = 0
        self.max_vel = 0.
        self.init_bodies()
    
    def init_bodies(self):
        self.bodies_l = []
        count = 0
        pos_aux = -SimulationManager.D_CUBE_LENGHT/2. + self.min_d
        K = 2.
        p = [pos_aux, pos_aux, pos_aux]
        dp = [self.min_d*K, self.min_d*K, self.min_d*K]
        while count < self.n_bodies:
            # Estructured initial position
            # pos = np.array(p)

            # Random initial position
            pos = np.array([rand.uniform(-self.limit_d, self.limit_d),
                              rand.uniform(-self.limit_d, self.limit_d),
                              rand.uniform(-self.limit_d, self.limit_d)])
            
            # Random initial vel
            vel = np.array([rand.uniform(self.vel_limits[0], self.vel_limits[1]),
                              rand.uniform(self.vel_limits[0], self.vel_limits[1]),
                              rand.uniform(self.vel_limits[0], self.vel_limits[1])])
            self.bodies_l.append(Body(self.b_radius, pos, vel))
            self.max_vel += self.norm_2(vel)/2.
            p[0] += dp[0]
            if abs(p[0] + dp[0]) > abs(SimulationManager.D_CUBE_LENGHT/2 - 3*self.b_radius):
                dp[0] *= -1
                p[1] += dp[1] 
                if abs(p[1]) > abs(SimulationManager.D_CUBE_LENGHT/2 - 3*self.b_radius):
                    dp[1] *= -1
                    p[2] += dp[2]
            count += 1
        self.max_vel = math.sqrt(self.max_vel/2.)*.10
        print("N BODIES", len(self.bodies_l), "MAX VEL", self.max_vel)
    
    def check_collisions(self):
        # Sort bodies (in the "x" axis) for better performance in collision detection
        self.sort_bodies()
        # Reset collision counters
        self.n_wall_collisions = 0
        self.n_bodies_collision = 0
        self.dp_wall_bodies = 0.
        # Loop through bodies
        for a in range(0, len(self.bodies_l)):
            body_a = self.bodies_l[a]
            # Checks collision with walls of the cube
            self.check_wall_collision(body_a)
            # Check collision with other bodies
            for b in range(a+1, len(self.bodies_l)):
                body_b = self.bodies_l[b]
                r_ab = body_b.b_pos - body_a.b_pos
                if r_ab[0] <= self.min_d:
                    if self.norm_2(r_ab) <= self.min_d*self.min_d:
                        self.on_collision(body_a, body_b)
                else:
                    # No more posible collision ("x" axis sorted)
                    break

    def check_wall_collision(self, body):
        for i in range(0, len(body.b_pos)):
            if abs(body.b_pos[i]) + body.b_radius >= SimulationManager.D_CUBE_LENGHT/2.:
                # Change status of body
                body.b_collisioned = True
                # Reverse vel in the axis
                body.b_vel[i] *= -1
                # Fix distance
                body.b_pos[i] = SimulationManager.D_CUBE_LENGHT/2. - body.b_radius if body.b_pos[i] > 0 else - SimulationManager.D_CUBE_LENGHT/2. + body.b_radius
                # Ref http://hyperphysics.phy-astr.gsu.edu/hbase/Kinetic/kinthe.html#c2
                # dp = 2.mass.vel
                # Calculate momentum transfered to the wall (mass = 1)
                self.dp_wall_bodies += 2*abs(body.b_vel[i])

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

    # Comparation function for sorting 
    def sort_bodies(self):
        self.bodies_l.sort(key=lambda body: body.b_pos[0])

    def update(self, dt):
        for body in self.bodies_l:
            body.update(dt)
        self.check_collisions()

    def draw(self, g_manager):
        g_manager.draw_cube_frame(SimulationManager.D_CUBE_LENGHT)
        for body in self.bodies_l:
            body.draw(g_manager, SimulationManager.SPHERE_NORMAL_COLOR, SimulationManager.SPHERE_COLLISION_COLOR)

    def get_simulation_m(self, dt):
        # Init dictionary
        s_magnitudes = { "v_rms":0., 
                      "v_med_v":np.array([0., 0., 0.]), 
                      "v_med":0.,
                      "k_t_axes":np.array([0., 0., 0.]),
                      "k_t":0.,
                      "k_med":0.,
                      "p_ideal":0.,
                      "p_calc":0.,
                      "temperature":0.
                      }
        # Calculate magnitudes
        for body in self.bodies_l:
            s_magnitudes["v_med_v"] += body.b_vel
            s_magnitudes["v_rms"] += self.norm_2(body.b_vel)
            s_magnitudes["k_t_axes"] += body.get_k_v()
            s_magnitudes["k_t"] += body.get_k()

        self.k_t = s_magnitudes["k_t"]

        s_magnitudes["v_med_v"] /= self.n_bodies
        s_magnitudes["v_med"] = self.norm(s_magnitudes["v_med_v"])
        s_magnitudes["v_rms"] = math.sqrt(s_magnitudes["v_rms"]/self.n_bodies)
        s_magnitudes["k_med"] = s_magnitudes["k_t"] / self.n_bodies

        # Ref http://hyperphysics.phy-astr.gsu.edu/hbase/Kinetic/kinthe.html#c2
        # Ideal pressure P = N.m.(V_rms)^2/3.V
        s_magnitudes["p_ideal"] = self.n_bodies * pow(s_magnitudes["v_rms"], 2) / (3. * self.cube_vol)
        # Calculated pressure 
        s_magnitudes["p_calc"] = self.dp_wall_bodies / (6 * self.cube_area * dt)

        # Ref http://hyperphysics.phy-astr.gsu.edu/hbase/Kinetic/eqpar.html#c1
        # Temperature
        self.k_temp = s_magnitudes["temperature"] = s_magnitudes["k_med"] * 2 / (3*SimulationManager.K_BOLTZMANN)

        return s_magnitudes
    
    def get_mb_dist_ideal(self):
        # Ref http://hyperphysics.phy-astr.gsu.edu/hbase/Kinetic/maxspe.html#c4
        f = lambda v: 4. * math.pi * math.pow(v, 2.) * math.pow(1. / (2 * math.pi * self.K_BOLTZMANN * self.k_temp), 1.5) * math.exp(-1. * math.pow(v, 2.) / (2. * self.K_BOLTZMANN * self.k_temp))
        distribution = []
        distribution.append(np.array([0, 0]))
        for i in range(1, math.ceil(self.max_vel)+1):
            distribution.append(np.array([i, f(i)*self.n_bodies]))
        # print(distribution) # DEBUG
        return distribution

    def get_mb_dist(self):
        vel_l = []
        #self.max_vel = int(self.norm(self.bodies_l[0].b_vel))
        for body in self.bodies_l:
            vel = self.norm(body.b_vel)
            vel_l.append(int(vel))
            #self.max_vel = self.max_vel if self.max_vel > vel else vel
        distribution = []
        distribution.append(np.array([0, 0]))
        for i in range(1, math.ceil(self.max_vel)+1):
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