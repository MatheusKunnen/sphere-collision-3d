class Body:
    def __init__(self, b_id, b_radius, b_pos, b_vel, c_limit = 1.):
        self.b_id = b_id
        self.b_radius = b_radius
        self.b_pos = b_pos
        self.b_vel = b_vel
        self.b_collisioned = False
        self.c_limit = c_limit

    def check_wall_collision(self):
        for i in range(0, len(self.b_pos)):
            if abs(self.b_pos[i]) >= self.c_limit:
                self.b_collisioned = True
                self.b_vel[i] *= -1
                self.b_pos[i] = self.c_limit if self.b_pos[i] > 0 else -self.c_limit

    def update(self, dt):
        self.b_collisioned = False
        self.b_pos += self.b_vel*dt
        self.check_wall_collision()

    def draw(self, g_manager, normal_color, collision_color, diffuse_color):
        g_manager.draw_solid_sphere(self.b_pos, self.b_radius, normal_color if not self.b_collisioned else collision_color)


