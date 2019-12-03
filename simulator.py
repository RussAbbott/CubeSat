
from os import path
from math import atan2, copysign, pi, pow
import pygame
from pygame.math import Vector2
from random import random, uniform


class Params:
    # Simulator params and static methods
    # frames-per-second
    FPS = 50

    # Window dimensions. These are also plane dimensions.
    # As both window and plane dimensions. (0,0) is at the upper left
    window_width = 800
    window_height = 800

    @staticmethod
    def roundV2(v2: Vector2, prec=2):
        return Params.V2(round(v2.x, prec), round(v2.y, prec))

    @staticmethod
    def V2(x, y):
        # noinspection PyArgumentList
        return Vector2(x, y)

    # Satellite params and static methods
    # These are change limits per frame.
    max_velocity = 1
    max_angle_change = 2

    @staticmethod
    def limit_velocity(v2):
        if abs(v2.x) > Params.max_velocity:
            v2.x = copysign(Params.max_velocity, v2.x)
        if abs(v2.y) > Params.max_velocity:
            v2.y = copysign(Params.max_velocity, v2.y)
        return v2

    @staticmethod
    def normalize_angle(angle):
        return (angle + 180) % 360 - 180

    @staticmethod
    def target_repulsive_force(self_position, target_position):
        """
        Compute a virtual repulsive force from the target so that CubeSat
        maintains a reasonable distance. The force decreases with the cube
        of the distance between them. The actual numbers are arbitrary.
        """
        dist_to_target_squared = (self_position.x-target_position.x)**2 + (self_position.y-target_position.y)**2
        # The exponent is 1.5 since we are using the distance squared.
        repulsive_force = 2E6/pow(dist_to_target_squared, 1.5)
        return repulsive_force


class Satellite:

    # Must define pos, vel, and angle as functions, which are executed at each instantiation.
    # Otherwise, they are executed only on the first instantiation, with those values used
    # for all future instantiations.
    def __init__(self, pos=lambda: Params.V2(uniform(100, Params.window_width-100),
                                             uniform(100, Params.window_height-100)),
                       vel=lambda: Params.V2(uniform(-1, 1), uniform(-1, 1)),
                       angle=lambda: uniform(-180, 180),
                       image=None):

        # pos, vel, and angle are with respect to the fixed window/plane
        self.position = pos()
        self.velocity = vel()
        self.angle = angle()

        current_directory = path.dirname(path.abspath(__file__))
        image_path = path.join(current_directory, image)
        self.image = pygame.image.load(image_path)


    def compute_velocity_correction(self, target_position):
        """ Compute CubeSat velocity correction. """
        desired_direction = target_position - self.position
        correction = desired_direction - self.velocity

        # Act as if there is a repulsive force from target to maintain a distance.
        repulsive_force = Params.target_repulsive_force(self.position, target_position)
        move_toward_target = 1 - repulsive_force
        # Allow correction to have an absolute value > 1.
        # update_cubesat_velocity imposes a speed limit.
        correction = move_toward_target * correction
        return correction

    def compute_angle_correction(self, target_position):
        """ Compute CubeSat angle correction. """
        (rel_x, rel_y) = (target_position.x - self.position.x, target_position.y - self.position.y)
        rel_angle = (180 / pi) * (-atan2(rel_y, rel_x))
        correction = rel_angle - self.angle
        return Params.normalize_angle(correction)

    def update_angle(self, correction):
        if abs(correction) > Params.max_angle_change:
            correction = copysign(Params.max_angle_change, correction)
        new_angle = Params.normalize_angle(self.angle + correction)
        self.angle = new_angle

    def update_cubesat_velocity(self, correction):
        self.velocity += correction
        self.velocity = Params.limit_velocity(self.velocity)

    def update_target_velocity(self):
        # Change direction every once in a while
        if random() < 0.01:
            self.velocity = Params.V2(uniform(-2, 2), uniform(-2, 2))

        # Don't go go off the screen. (Respond to virtual repulsion force from screen edges.)
        # The force is treated as always existing. It's strength depends on how close the
        # target is to an edge. The exponent in the denominator must be odd to retain
        # the sign of the value it is raising to a power. Other than that, the numbers
        # are arbitrary.
        delta_x = min(1, Params.window_width**3/self.position.x**5) + \
                  max(-1, Params.window_width**3/(self.position.x-Params.window_width)**5)
        delta_y = min(1, Params.window_height**3/self.position.y**5) + \
                  max(-1, Params.window_height**3/(self.position.y-Params.window_height)**5)
        self.velocity += Params.V2(delta_x, delta_y)

        # Ensure that the target is moving at a reasonable speed.
        # Allow it to move faster than CubeSat: 1.5 vs 1.
        # The actual numbers are arbitrary.
        if abs(self.velocity.x) < 0.4:
            self.velocity.x *= 2
        if abs(self.velocity.x) > 1.5:
            self.velocity.x *= 0.7
        if abs(self.velocity.y) < 0.4:
            self.velocity.y *= 2
        if abs(self.velocity.y) > 1.5:
            self.velocity.y *= 0.7

    def update_velocity(self, correction):
        """ Update CubeSat's velocity differently from the target's velocity. """
        self.update_target_velocity() if correction is None else self.update_cubesat_velocity(correction)

    def update_position(self):
        self.position += self.velocity


class Sim:

    def __init__(self):
        pygame.init()
        pygame.display.set_caption("CubeSat Simulator")
        window_dimensions = (Params.window_width, Params.window_height)
        self.screen = pygame.display.set_mode(window_dimensions)
        self.clock = pygame.time.Clock()

        # Use a square with an arrow for CubeSat
        self.cubesat = Satellite(image="CubeSat.png")
        # Use an ArUco marker for the target
        self.target = Satellite(image="ArUcoTarget.png")

        self.exit = False

    def add_obj_to_screen(self, obj):
        obj_display = pygame.transform.rotate(obj.image, obj.angle)
        rect = obj_display.get_rect( )
        self.screen.blit(obj_display, obj.position - (rect.width/2, rect.height/2))

    def refresh_screen(self):
        self.screen.fill((0, 0, 0))
        self.add_obj_to_screen(self.target)
        self.add_obj_to_screen(self.cubesat)
        pygame.display.flip()

    def run(self):
        while not self.exit:
            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            cubesat_velocity_correction = self.cubesat.compute_velocity_correction(self.target.position)
            self.cubesat.update_velocity(cubesat_velocity_correction)
            self.cubesat.update_position()

            # CubeSat does not have a rotational velocity. It is always at a fixed angle, which
            # changes frame-by-frame.
            cubesat_angle_correction = self.cubesat.compute_angle_correction(self.target.position)
            self.cubesat.update_angle(cubesat_angle_correction)

            self.target.update_velocity(None)
            self.target.update_position()

            self.refresh_screen()
            self.clock.tick(Params.FPS)
        pygame.quit()


if __name__ == '__main__':
    Sim().run()
