# @desc: Sphere Elastic Collisions Simulator
# @autor: Matheus Kunnen Ledesma - matheusl.2000@alunos.utfpr.edu.br

import numpy as np
import time
import math

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from OpenGLManager import OpenGLManager
from Graph import Graph
from SimulationManager import SimulationManager


class Main:
    # General parameters
    ROT_K = .5
    MOVE_K = .1
    DT_K = .1

    # Simulator parameters
    cube_lenght = 10. # Cube lenght
    bodies_l = []     # Bodies list

    def __init__(self):
        # Init General Parameters
        self.is_running = False
        self.is_paused = True
        self.dt = 1/60.0
        self.dt_k = 1.
        self.target_dt = 1/20.0
        self.t_total = 0
        self.hud_enabled = True
        self.graphs_enabled = True
        self.vector_field_enabled = False
        self.spheres_g_enabled = True
        
        # Init Graphics Manager
        self.g_manager = OpenGLManager("3D Sphere Collision Simulator | Matheus Kunnen ")
        self.move_vector = np.array([0., 0., 0.])
        self.rot_vector = np.array([0., 0., 0.])
        self.cam_pos = np.array([5., 26., -5.])
        self.cam_rot = np.array([-40., 0., 0.])
        self.g_manager.cam_pos = self.cam_pos
        self.g_manager.cam_rot = self.cam_rot

        # Init Collision Manager & Bodies
        self.s_manager = SimulationManager()

        # Init Graphs
        self.init_graphs()


    def init_bodies(self):
        pass

    def init_graphs(self):
        graphs_s = [600, 300]
        # graphs_offset = [20, -self.g_manager.display_size[1] + graphs_s[1] + 20]
        n_points = 1000
        n = 1
        self.graph_mb_distrib = Graph("Maxwell-Boltzmann Distribution (n x V)", ["n", "V"], n_points, 
                            np.array(graphs_s), 
                            np.array([20, 20]), 2)
        self.graph_mb_distrib_ideal = Graph("", ["n", "V"], n_points, 
                            np.array(graphs_s), 
                            np.array([20, 20]), 2, np.array([0., 1., 0., 10.]))

    def run(self):
        self.is_running = self.g_manager.init_display()
        self.t_total = 0
        while self.is_running:
            t1 = time.time()
            self.g_manager.clear_buffer()
            self.check_events()
            if not self.is_paused:
                self.update()
            self.draw()
            self.g_manager.swap_buffers()
            t2 = time.time()
            self.update_dt(t1, t2)

    def draw(self):
        self.draw_hud()
        self.draw_graphs()
        if self.spheres_g_enabled:
            self.s_manager.draw(self.g_manager)

    def draw_graphs(self):
        if not self.graphs_enabled:
            return
        # self.graph_mb_distrib.update_edge_values()
        # self.graph_mb_distrib.update_scale()

        # self.graph_mb_distrib.draw(self.g_manager)
        # self.graph_mb_distrib_ideal.g_scale = self.graph_mb_distrib.g_scale 
        # self.graph_mb_distrib_ideal.max_v = self.graph_mb_distrib.max_v 
        # self.graph_mb_distrib_ideal.min_v = self.graph_mb_distrib.min_v 
        # self.graph_mb_distrib_ideal.draw(self.g_manager, False)

        self.graph_mb_distrib_ideal.draw(self.g_manager)
        self.graph_mb_distrib.g_scale = self.graph_mb_distrib_ideal.g_scale 
        self.graph_mb_distrib.max_v = self.graph_mb_distrib_ideal.max_v 
        self.graph_mb_distrib.min_v = self.graph_mb_distrib_ideal.min_v 
        self.graph_mb_distrib.draw(self.g_manager, False)

    def update(self):
        self.s_manager.update(self.get_sim_dt())
        self.update_graphs()

    def update_graphs(self):
        self.s_manager.get_mb_dist()
        self.graph_mb_distrib_ideal.points_queue.set_elements(self.s_manager.get_mb_dist_ideal())
        self.graph_mb_distrib.points_queue.set_elements(self.s_manager.get_mb_dist())

    def draw_hud(self):
        if not self.hud_enabled:
            return
        txt_status = "Paused" if self.is_paused else "Running"
        magnitudes = self.s_manager.get_simulation_m(self.get_sim_dt())
        p_i = magnitudes["p_ideal"]
        p_c = magnitudes["p_calc"]
        v_med_v = magnitudes["v_med_v"]
        v_med = magnitudes["v_med"]
        v_rms = magnitudes["v_rms"]
        k_v = magnitudes["k_t_axes"]
        k = magnitudes["k_t"]
        k_med = magnitudes["k_med"]
        temperature = magnitudes["temperature"]
        mean_free_path = magnitudes["mean_free_path"]
        collision_freq_teoric = magnitudes["collision_freq_teoric"]
        collision_freq = (self.s_manager.n_body_body_collisions + self.s_manager.n_wall_collisions)/(self.get_sim_dt()*self.s_manager.cube_vol)
        self.g_manager.captions = [
            "-> General Parameters",
            f"    FPS: {round(1/self.dt,0)} ({round(self.dt*1000, 1)}ms)",
            f"Runtime: {round(self.t_total, 3)}s.",
            f"   Play: x{round(self.dt_k,1)}",
            f"Cam. dP: {np.round(self.cam_pos, 2)}",
            f"Cam. dR: {np.round(self.cam_rot, 2)}",
            f" Status: {txt_status}", "",
            "-> Simulation Parameters",
            f"N. Spheres: {self.s_manager.n_bodies}",
            f"V_med: {np.round(v_med_v, 3)}",
            f"V_med: {np.round(v_med, 3)}",
            f"V_rms: {round(v_rms, 3)}",
            f"K_t_v: {np.round(k_v, 3)}",
            f"  K_t: {round(k, 3)} <{round(k_med, 3)}>",
            f"   Pi: {round(p_i, 3)}",
            f"    P: {round(p_c, 3)}",
            f"Temp.: {round(temperature, 3)}",
            f"  FMP: {round(mean_free_path, 3)}",
            f"   Ni: {round(collision_freq_teoric, 3)}",
            f"    N: {round(collision_freq, 3)}"]

        self.g_manager.draw_captions()

    def update_dt(self, t1, t2):
        self.dt = t2 - t1
        self.t_total += self.dt if not self.is_paused else 0.
        if self.dt > self.target_dt:
            pass
            #print(f"FPS {round(1/self.dt,1)} | {round((self.dt - self.target_dt)/self.target_dt,1)} frames missed")
        else:
            self.g_manager.wait(self.target_dt - self.dt)

    def get_sim_dt(self):
        return self.target_dt * self.dt_k

    def check_events(self):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT:
                    self.move_vector[0] = Main.MOVE_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_RIGHT:
                    self.move_vector[0] = - Main.MOVE_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_DOWN:
                    self.move_vector[1] = Main.MOVE_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_UP:
                    self.move_vector[1] = - Main.MOVE_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_q:
                    self.move_vector[2] = Main.MOVE_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_e:
                    self.move_vector[2] = - Main.MOVE_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_a:
                    self.rot_vector[1] = - Main.ROT_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_d:
                    self.rot_vector[1] = Main.ROT_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_w:
                    self.rot_vector[0] = - Main.ROT_K if event.type == pygame.KEYDOWN else 0.
                elif event.key == pygame.K_s:
                    self.rot_vector[0] = Main.ROT_K if event.type == pygame.KEYDOWN else 0.  
                elif event.key == pygame.K_h and event.type == pygame.KEYDOWN:
                    self.hud_enabled = not self.hud_enabled
                elif event.key == pygame.K_v and event.type == pygame.KEYDOWN:
                    self.vector_field_enabled = not self.vector_field_enabled
                elif event.key == pygame.K_PAGEUP and event.type == pygame.KEYDOWN:
                    self.dt_k += Main.DT_K
                elif event.key == pygame.K_PAGEDOWN and event.type == pygame.KEYDOWN and round(self.dt_k - Main.DT_K,1) > 0:
                    self.dt_k -= Main.DT_K
                elif event.key == pygame.K_p and event.type == pygame.KEYDOWN:
                    self.is_paused = not self.is_paused
                elif event.key == pygame.K_g and event.type == pygame.KEYDOWN:
                    self.graphs_enabled = not self.graphs_enabled
                elif event.key == pygame.K_r and event.type == pygame.KEYDOWN:
                    self.spheres_g_enabled = not self.spheres_g_enabled
            if event.type == pygame.QUIT:
                #is_running = False
                pygame.quit()
                quit()
        self.cam_pos = [self.cam_pos[0] + self.move_vector[0], self.cam_pos[1] +
                        self.move_vector[1], self.cam_pos[2] + self.move_vector[2]]
        self.cam_rot = [self.cam_rot[0] + self.rot_vector[0], self.cam_rot[1] +
                        self.rot_vector[1], self.cam_rot[2] + self.rot_vector[2]]
        self.g_manager.move_cam(self.move_vector)
        self.g_manager.rotate_cam(self.rot_vector)

if __name__ == "__main__":
    main = Main()
    main.run()

exit()