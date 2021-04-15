import math
import time
import random as rand
import pygame

pygame.init()

s_width = 1000
s_height = 600

screen = pygame.display.set_mode((s_width, s_height))
pygame.display.set_caption('Gravity Simulator')

weights = {
    "real_hydrogen": (0.00000000000000000000000166058, 1),
    "hydrogen": (0.00000000000000166058, 3)
}

g_constant = 6.67408e-11
t = 0.05
mag = 10000000000
speed_cap = 0.000000008

alive = []


class Particle:
    def __init__(self, coords, force, type, id):
        alive.append(self)
        # 1 pixel = 1 angstrom
        self.coords = coords
        # Convert to meters for calculations
        self.rcoords = []
        for coord in coords:
            self.rcoords.append(coord / mag)
        self.mass = type[0]
        self.radius = type[1]
        self.rradius = self.radius / mag
        self.rarea = math.pi * self.rradius ** 2
        self.id = 'p' + str(id)
        self.force = force
        self.collided = False
        self.x_forces = []
        self.y_forces = []
        self.x_force = 0
        self.y_force = 0

    def update(self):
        # Calculate forces on every other particle
        forces = []
        for particle2 in particles:
            if particle2.rcoords != self.rcoords and particle2 in alive:
                # Get dist from object
                x_diff = abs(self.rcoords[0] - particle2.rcoords[0])
                y_diff = abs(self.rcoords[1] - particle2.rcoords[1])
                dist = math.sqrt((x_diff ** 2) + (y_diff ** 2))

                # Check for collision
                if self.radius + particle2.radius > abs(dist) * mag and self in alive and self.rarea <= particle2.rarea:
                    self.collided = True
                    particle2.mass += self.mass
                    particle2.rarea += self.rarea
                    particle2.x_force += self.x_force
                    particle2.y_force += self.y_force
                    # print(particle2.id + str(math.pi * particle2.radius ** 2))
                    if self in alive:
                        alive.remove(self)
                        return None

                # Get area and radius
                self.rradius = math.sqrt(self.rarea / math.pi)
                self.radius = int(self.rradius * mag)

                # Calculate force
                force = g_constant * ((self.mass * particle2.mass) / dist ** 2)

                # Calculate acceleration
                acc = force / self.mass
                if acc > speed_cap:
                    acc = speed_cap

                # Get time
                t1 = time.perf_counter()
                t = 1 / (1 / (t1 - t0))

                # Calculate distance travelled
                d_travelled = acc * t

                # Get force vector
                mul = d_travelled / dist
                gravity_force = []
                if self.rcoords[0] < particle2.rcoords[0]:
                    gravity_force.append(abs(x_diff * mul))
                else:
                    gravity_force.append(-abs(x_diff * mul))

                if self.rcoords[1] < particle2.rcoords[1]:
                    gravity_force.append(abs(y_diff * mul))
                else:
                    gravity_force.append(-abs(y_diff * mul))

                if not self.collided:
                    forces.append(gravity_force)

        # Get average of each force on every axis
        for force in forces:
            self.x_forces.append(force[0])
            self.y_forces.append(force[1])

        self.x_force = sum(self.x_forces) + self.force[0]
        self.y_force = sum(self.y_forces) + self.force[1]

        print(self.x_force)

        # Get sum of all forces
        distances_to_move = [self.x_force, self.y_force]

        # Update position of particle
        self.rcoords[0] += distances_to_move[0]
        self.rcoords[1] += distances_to_move[1]

        # Convert meters back to angstroms
        self.coords = []
        for coordinate in self.rcoords:
            self.coords.append(int(coordinate * mag))

        # Draw particle
        if self in alive:
            pygame.draw.circle(screen, (255, 255, 255), (self.coords[0], self.coords[1]), abs(self.radius))


particles = []
for i in range(2):
    particles.append(Particle((rand.randrange(0, s_width, 1), rand.randrange(0, s_height, 1), 0), (0, 0), weights["hydrogen"], i))

particles.append(Particle((50, 50), (5.1e-10, 0), weights["hydrogen"], 10))

# Main Loop

run = 1
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = 0

    # Get time for distance calculations
    t0 = time.perf_counter()

    screen.fill((0, 0, 0))

    # Calculate all forces
    for particle in particles:
        particle.update()

    pygame.display.update()

pygame.quit()
