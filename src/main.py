# @desc: Sphere Elastic Collisions Simulator
# @autor: Matheus Kunnen Ledesma - matheusl.2000@alunos.utfpr.edu.br

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from Simulator import Simulator
from Body import Body
import time
import concurrent.futures
from multiprocessing import Process
""" General parameters """
length = 16                 # Largo do lado do cubo
n_balls = 200              # Numero de bolas no cubo
b_radius = .25              # Radio minimo das bolas
max_v0 = 3                  # Velocidade inicial maxima das bolas
dt = 0.05                   # Variacao do tempo
display_size = (1200, 800)  # Tamanho da janela a abrir
sphere_slices = 15          # Divisioes das bolas (> -> Maior Qualidade)
running = False             # Indicador de estado
pause = False               # Indicador de estado do renderizado
text_pos = (10, 750)        # Posicao inicial do texto
text_dP = 15                # Distancia entre linhas do texto
real_dt = 1                 # Tempo de renderizado/procesado

""" Ilumination parameters """
lightZeroPosition = [.5*length, 1.*length, -.1 *
                     length, 1.]  # Posicao da fonte de iluminacao
# Cor da fonte de iluminacao
lightZeroColor = [1., 1., 1., 1.]
# Posicao da fonte de iluminacao ambiente
lightZeroAmbient = [.5, .5, .5, 1.]
# Cor da fonte de direta?
lightZeroSpecular = [10., 10., 10., 1.]

""" Camera parameters """
camera_pos_0 = (0, length / 6, -length * 2)     # Posicao da camera (visao)
# Rotacao da camera no eixo x (visao)
camera_rot_x = 30.
# Rotacao da camera no eixo y (visao)
camera_rot_y = 30.

""" Colors """
cube_color = [1., 0., 0., 1.]                       # Cor do frame do Cubo
sphere_diffuse_color = [1., .8, 0., 1.]             # Cor das bolas
sphere_ambient_color = [.5, .4, 0., 1.]             # Cor ambiente? das bolas
sphere_collision_diffuse_color = [1., .0, 0., 1.]   # Cor das bolas
sphere_collision_ambient_color = [.5, .0, 0., 1.]   # Cor ambiente? das bolas
text_color = [10., 10., 10., 1.]                    # Cor do texto

""" Collision class instance """
c_simulator = Simulator(n_balls, b_radius, length, dt, max_v0)


def init_display():
    if running:
        return True

    pygame.init()
    pygame.display.set_mode(display_size, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("3D Sphere Collision - Matheus Kunnen")
    pygame.display.gl_set_attribute(GL_ACCELERATED_VISUAL, True)
    glClearColor(0., 0., 0., 1.)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glCullFace(GL_BACK)

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightZeroAmbient)
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightZeroSpecular)

    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.5)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.01)
    glEnable(GL_LIGHT0)

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (display_size[0] / display_size[1]), 0.1, 50.0)

    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()

    glTranslatef(camera_pos_0[0], camera_pos_0[1], camera_pos_0[2])
    glRotatef(camera_rot_x, 1., 0., 0.)
    glRotatef(camera_rot_y, 0., 1., 0.)

    return True


def check_events():
    global running, pause, camera_rot_x, camera_rot_y, c_simulator
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                camera_rot_x = camera_rot_x + 5
            if event.key == pygame.K_DOWN:
                camera_rot_x = camera_rot_x - 5
            if event.key == pygame.K_RIGHT:
                camera_rot_y = camera_rot_y + 5
            if event.key == pygame.K_LEFT:
                camera_rot_y = camera_rot_y - 5
            if event.key == pygame.K_PAGEUP and c_simulator.dt < .2:
                c_simulator.dt = c_simulator.dt + .01
            if event.key == pygame.K_PAGEDOWN and c_simulator.dt > .01:
                c_simulator.dt = c_simulator.dt - .01
            if event.key == pygame.K_SPACE:
                pause = not pause
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
            quit()
            return False
    return True


def draw_cube_frame():
    glPushMatrix()
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cube_color)
    glTranslatef(0, 0, 0)
    glutWireCube(length)
    glPopMatrix()


def draw_balls():
    for body in c_simulator.l_bodies:
        glPushMatrix()
        if not body.b_collisioned:
            glMaterialfv(GL_FRONT, GL_DIFFUSE, sphere_diffuse_color)
            glMaterialfv(GL_FRONT, GL_AMBIENT, sphere_ambient_color)
        else:
            glMaterialfv(GL_FRONT, GL_DIFFUSE, sphere_collision_diffuse_color)
            glMaterialfv(GL_FRONT, GL_AMBIENT, sphere_collision_ambient_color)
        # glMaterialfv(GL_FRONT, GL_DIFFUSE, [ball[0] + .5, ball[1] + .5, ball[2] + .5, 1.])  # Cor Especial 01
        # glMaterialfv(GL_FRONT, GL_AMBIENT, [ball[0] + .5, ball[1] + .5, ball[2] + .5, 1.])  # Cor Especial 01
        glTranslatef(
            body.b_pos[0], body.b_pos[1], body.b_pos[2])
        glutSolidSphere(body.b_radius, sphere_slices, sphere_slices)
        glPopMatrix()


def draw_hud(real_dt):
    """ Set 2D mode"""
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0.0, display_size[0], 0.0, display_size[1])
    glMatrixMode(GL_MODELVIEW)
    glPushMatrix()
    glLoadIdentity()

    captions = [f'FPS: {round(1/real_dt,0)} ({round(real_dt*1000,0)}ms)',
                ''
                'N. Bodies: {}'.format(c_simulator.n_balls),
                'dt: {0:.2f}'.format(c_simulator.dt),
                ' ',
                'Momentum: {0:.3f}'.format(c_simulator.momentum),
                ' ',
                f'Vmed: ({round(c_simulator.v_med[0],3)}, {round(c_simulator.v_med[1],3)}, {round(c_simulator.v_med[2],3)})',
                ' ',
                f'Rcm: ({round(c_simulator.r_cm[0],3)}, {round(c_simulator.r_cm[1],3)}, {round(c_simulator.r_cm[2],3)})']

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, text_color)
    for i in range(len(captions)):
        glRasterPos2d(text_pos[0], text_pos[1] - i*text_dP)
        for j in range(len(captions[i])):
            glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24,
                                ord(captions[i][j]))

    """ Making sure we can render 3d again """
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glTranslatef(camera_pos_0[0], camera_pos_0[1], camera_pos_0[2])
    glRotatef(camera_rot_x, 1., 0., 0.)
    glRotatef(camera_rot_y, 0., 1., 0.)
    glPopMatrix()


def adjust_camera():
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()
    glPushMatrix()

    glTranslatef(camera_pos_0[0], camera_pos_0[1], camera_pos_0[2])
    glRotatef(camera_rot_x, 1., 0., 0.)
    glRotatef(camera_rot_y, 0., 1., 0.)


def init_main_loop():
    global running
    running = init_display()
    real_dt = 1
    while running:
        t1 = time.time()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        draw_cube_frame()
        draw_balls()
        if not pause:
            c_simulator.update()
        draw_hud(real_dt)
        adjust_camera()
        glutSwapBuffers()

        pygame.display.flip()
        pygame.time.wait(1)
        check_events()
        t2 = time.time()
        real_dt = t2 - t1
    # print(round(1/real_dt, 2), " FPS", f"{round(real_dt,3)}s.")


init_main_loop()  # Inicia Loop da simulacao
