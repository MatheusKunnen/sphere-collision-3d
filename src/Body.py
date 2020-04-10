class Body:
    def __init__(self, b_id, b_radius, b_pos, b_vel):
        self.b_id = b_id
        self.b_radius = b_radius
        self.b_pos = b_pos
        self.b_vel = b_vel
        self.b_collisioned = False

    def move(self, dt):
        self.b_collisioned = False
        #print(self.b_pos, ' ', self.b_vel, ' ', dt)
        self.b_pos += self.b_vel*dt

    def check_wall_collision(self, c_limit):
        for i in range(0, len(self.b_pos)):
            if abs(self.b_pos[i]) >= c_limit:
                self.b_collisioned = True
                self.b_vel[i] *= -1
                self.b_pos[i] = c_limit if self.b_pos[i] > 0 else -c_limit
