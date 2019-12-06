
from os import path
from math import atan2, copysign, pi, sqrt
import pygame
from pygame.math import Vector2
from random import random, uniform

"""
This code consists of three classes.
Params: A collection of constants and static methods
Satellite: A class of which both CubeSat and the target are instances.
Sim: The simulation infrastructure
"""


class Params:
    
    # ==================================================================================
    # The simulation object itself is available here. It stores the positions of
    # both CubeSat and the target, in Euclidian pixel coordinates. (Upper left is (0,0).)
    sim = None
    
    # Simulator params and static methods
    # frames-per-second
    FPS = 50

    # Window dimensions, in pixels. These are also plane dimensions and are taken as plane units.
    # As both window and plane dimensions. (0,0) is at the upper left.
    window_width = 800   # pixels
    window_height = 800  # pixels

    @staticmethod
    def roundV2(v2: Vector2, prec=2):
        return Params.V2(round(v2.x, prec), round(v2.y, prec))

    @staticmethod
    def V2(x, y):
        # noinspection PyArgumentList
        return Vector2(x, y)

    # ==================================================================================
    # Satellite params and static methods
    # Velocities are pizels per frame.
    
    # CubeSat
    cubesat_max_velocity = 1  # pixels / frame
    # pygame uses degrees for angeles rather than radians
    cubesat_max_angle_change = 2  # degrees/frame
    
    # Target
    target_max_velocity = 1.9   # pixels / frame
    target_min_velocity = 0.75  # pixels / frame

    # Probability of changing velocity each frame.
    # About once every second and a half.
    prob_change_vel = 0.009

    @staticmethod
    def distance(a, b):
        return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    @staticmethod
    def limit_cubesat_velocity(v2):
        if abs(v2.x) > Params.cubesat_max_velocity:
            v2.x = copysign(Params.cubesat_max_velocity, v2.x)
        if abs(v2.y) > Params.cubesat_max_velocity:
            v2.y = copysign(Params.cubesat_max_velocity, v2.y)
        return v2

    @staticmethod
    def normalize_angle(angle):
        return (angle + 180) % 360 - 180

    # @staticmethod
    # def stay_inbounds_force(position):
    #     """
    #     Don't go go off the screen. (Respond to virtual repulsion force from screen edges.)
    #     The force is treated as always existing. It's strength depends on how close the
    #     position is to an edge. The exponent in the denominator must be odd to retain
    #     the sign of the value it is raising to a power. Other than that, the numbers
    #     are arbitrary.
    #     """
    #     delta_x = min(1, Params.window_width**5/position.x**9) + \
    #               max(-1, Params.window_width**5/(position.x-Params.window_width)**9)
    #     delta_y = min(1, Params.window_height**5/position.y**9) + \
    #               max(-1, Params.window_height**5/(position.y-Params.window_height)**9)
    #     return Params.V2(delta_x, delta_y)

    @staticmethod
    def stay_in_screen(position):
        """
        Rigidly keep the position within the screen.
        """
        position.x = max(50, min(Params.window_width-50, position.x))
        position.y = max(50, min(Params.window_height-50, position.y))

    @staticmethod
    def target_repulsive_force(self_position, target_position):
        """
        Compute a virtual repulsive force from the target so that CubeSat
        maintains a reasonable distance. The force decreases with the fourth
        power of the distance between them. The actual numbers are arbitrary.
        Doing it this way makes the relationship between the two smoother and
        more fluid. An alternative might have been to keep CubeSat a fixed
        distance away from the target.
        """
        dist_to_target = Params.distance(self_position, target_position)
        repulsive_force = 1E9/dist_to_target**4
        return repulsive_force


class Satellite:

    def __init__(self, pos=None, vel=None, angle=None, image=None):

        # pos, vel, and angle are with respect to the fixed window/plane
        self.position = pos if pos is not None else \
                        Params.V2(uniform(100, Params.window_width-100), uniform(100, Params.window_height-100))
        self.velocity = vel if vel is not None else Params.V2(uniform(-2, 2), uniform(-2, 2))
        self.angle = angle if angle is not None else uniform(-180, 180)

        current_directory = path.dirname(path.abspath(__file__))
        image_path = path.join(current_directory, image)
        self.image = pygame.image.load(image_path)

    def cubesat_angle_correction(self):
        """ Compute CubeSat angle correction for the current frame update. """
        target_position = Params.sim.target.position
        (rel_x, rel_y) = (target_position.x - self.position.x, target_position.y - self.position.y)
        rel_angle = (180 / pi) * (-atan2(rel_y, rel_x))
        correction = rel_angle - self.angle
        return Params.normalize_angle(correction)

    def cubesat_velocity_correction(self):
        """ Compute CubeSat velocity correction for the current frame update. """
        target_position = Params.sim.target.position
        desired_direction = target_position - self.position
        correction = desired_direction - self.velocity

        # To maintain a distance, act as if there is a repulsive force from target.
        repulsive_force = Params.target_repulsive_force(self.position, target_position)
        move_toward_target = 1 - repulsive_force
        correction = move_toward_target * correction

        return correction

    def update_cubesat_angle(self, correction):
        """
        Update the CubeSat heading (it's angle). Limit the maximum
        change that can occur in any frame. If the entire correction
        does not occur during the current frame, it will continue
        (possibly adjusted) in the next frame.
        """
        if abs(correction) > Params.cubesat_max_angle_change:
            correction = copysign(Params.cubesat_max_angle_change, correction)
        new_angle = Params.normalize_angle(self.angle + correction)
        self.angle = new_angle

    def update_cubesat_velocity(self, correction):
        """
        Update the CubeSat velocity. Limit the maximum change that
        can occur in any frame. If the entire correction does not
        occur during the current frame, it will continue (possibly
        adjusted) in the next frame.
        """
        self.velocity += correction
        self.velocity = Params.limit_cubesat_velocity(self.velocity)
        # If we are too close to the target, backpedal faster. (Very ad hoc.)
        dist_to_target = Params.distance(self.position, Params.sim.target.position)
        # noinspection PyTypeChecker
        velocity_multiplier = max(1, (125/max(dist_to_target, 50))**2)
        self.velocity *= velocity_multiplier
        # if dist_to_target < 100:
        #     target_velocity = Params.sim.target.velocity
        #     velocity_ratio = (self.velocity.x / self.velocity.y) / (target_velocity.x / target_velocity.y)
        #     if 0.9 < velocity_ratio < 1.1:
        #         print(round(velocity_ratio, 2))
        #         correction.x += 10
        #         correction.y -= 10

    def update_target_velocity(self):
        """
        Update the target velocity--for this frame. These change
        are arbitrary and get the target to move around the screen.
        """
        # Change direction every once in a while
        if random() < Params.prob_change_vel:
            self.velocity = Params.V2(uniform(-2, 2), uniform(-2, 2))
        # If too close to the window walls or CubeSat, reverse direction
        if (self.position.x == 50 or self.position.x == Params.window_width - 50) and \
                (self.position.y == 50 or self.position.y == Params.window_height - 50) or \
                Params.distance(self.position, Params.sim.cubesat.position) < 70:
            self.velocity *= -1

        # self.velocity += Params.stay_inbounds_force(self.position)

        # Ensure that the target is moving at a reasonable speed.
        # Allow it to move faster than CubeSat: 1.5 vs 1.
        # The actual numbers are arbitrary.
        if abs(self.velocity.x) < Params.target_min_velocity:
            self.velocity.x *= 2
        if abs(self.velocity.x) > Params.target_max_velocity:
            self.velocity.x *= 0.7
        if abs(self.velocity.y) < Params.target_min_velocity:
            self.velocity.y *= 2
        if abs(self.velocity.y) > Params.target_max_velocity:
            self.velocity.y *= 0.7

    def update_velocity(self, correction):
        """ Update CubeSat's velocity differently from the target's velocity. """
        self.update_target_velocity() if correction is None else self.update_cubesat_velocity(correction)

    def update_position(self):
        self.position += self.velocity
        Params.stay_in_screen(self.position)
        # if self is Params.sim.target and \
        #         (self.position.x == 50 or self.position.x == Params.window_width-50) and \
        #         (self.position.y == 50 or self.position.y == Params.window_height-50):
        #     self.velocity = Params.V2(uniform(-2, 2), uniform(-2, 2))


class Sim:

    def __init__(self):
        # Make this object itself available in the Params class.
        Params.sim = self
        
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
        """ Update the screen "surface" before displaying it. """
        obj_display = pygame.transform.rotate(obj.image, obj.angle)
        rect = obj_display.get_rect()
        self.screen.blit(obj_display, obj.position - (rect.width/2, rect.height/2))

    def refresh_screen(self):
        """
        Refresh the screen. Create a black background and
        put the two objects in the surface. Then make it visible.
        """
        self.screen.fill((0, 0, 0))
        self.add_obj_to_screen(self.target)
        self.add_obj_to_screen(self.cubesat)
        pygame.display.flip()

    def run(self):
        """ The main loop. """
        while not self.exit:
            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            cubesat_velocity_correction = self.cubesat.cubesat_velocity_correction()
            self.cubesat.update_velocity(cubesat_velocity_correction)
            self.cubesat.update_position()

            # CubeSat does not have a rotational velocity. It is always at a fixed angle,
            # which changes frame-by-frame.
            cubesat_angle_correction = self.cubesat.cubesat_angle_correction()
            self.cubesat.update_cubesat_angle(cubesat_angle_correction)

            self.target.update_velocity(None)
            self.target.update_position()

            self.refresh_screen()
            self.clock.tick(Params.FPS)
        pygame.quit()


if __name__ == '__main__':
    Sim().run()
