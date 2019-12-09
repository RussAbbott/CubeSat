
from math import atan2, copysign, pi, sqrt
from os import path
from random import choice, random, uniform

import pygame
from pygame.math import Vector2

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

    # Must set this outside the class(?)
    screen_center = None

    # recenter_mode:    False: the objects are unable to penetrate the window borders
    #                   True: the object are recentered in the window when they approach a window border
    recenter_mode = True
    recentering = False
    centered_enough = 50

    v2_zero = None

    @staticmethod
    def roundV2(v2: Vector2, prec=2):
        return Params.V2(round(v2.x, prec), round(v2.y, prec))

    @staticmethod
    def V2(x, y):
        # noinspection PyArgumentList
        return Vector2(float(x), float(y))

    # ==================================================================================
    # Satellite params and static methods
    # Velocities are pizels per frame.
    
    # CubeSat
    cubesat_max_velocity = 1  # pixels / frame
    # pygame uses degrees rather than radians
    cubesat_max_angle_change = 0.5  # degrees/frame

    # For when not in rd_mode
    directional_ticks_limit = 50

    # Target
    target_max_velocity = 1.9   # pixels / frame
    target_min_velocity = 0.75  # pixels / frame
    target_vel_change = 1

    # Probability of changing velocity each frame.
    # About once every second and a half.
    prob_change_vel = 0.009
    
    @staticmethod
    def at_edge_of_screen(position):
        return not (50 < position.x < Params.window_width-50) or not (50 < position.y < Params.window_height-50)


    @staticmethod
    def distance(a, b):
        return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    @staticmethod
    def limit_velocity(v2, max_velocity):
        max_xy = max(abs(v2.x), abs(v2.y))
        if max_xy < max_velocity:
            return v2
        v2 *= max_velocity/max_xy
        return v2

    @staticmethod
    def normalize_angle(angle):
        return (angle + 180) % 360 - 180

    @staticmethod
    def recenter_all():
        (pos1, pos2) = (Params.sim.cubesat.position, Params.sim.target.position)
        center_point = (pos1 + pos2) / 2
        correction = Params.screen_center - center_point
        if correction.magnitude() < 5:
            Params.recentering = False
            return
        max_speed = 2.5*Params.cubesat_max_velocity
        while abs(correction.x) > max_speed or abs(correction.y) > max_speed:
            correction *= 0.9
        pos1 += correction
        pos2 += correction

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

    def __init__(self, pos=None, vel=None, angle=None, image=None, degraded=False):

        # pos, vel, and angle are with respect to the fixed window/plane
        self.position = pos if pos is not None else \
                        Params.V2(uniform(100, Params.window_width-100), uniform(100, Params.window_height-100))
        self.velocity = vel if vel is not None else Params.V2(uniform(-2, 2), uniform(-2, 2))
        self.angle = angle if angle is not None else uniform(-180, 180)

        current_directory = path.dirname(path.abspath(__file__))
        image_path = path.join(current_directory, image)
        self.image = pygame.image.load(image_path)
        self.degraded = degraded
        self.ticks = 0

    def allow_angle_correction(self):
        return self.ticks == 0

    def allow_posiion_change(self):
        return self.ticks > 0

    def allow_velocity_correction(self):
        return 0 < self.ticks < 6

    def cubesat_angle_correction(self):
        """ Compute CubeSat angle correction for the current frame update. """
        if self.degraded:
            self.velocity = Params.v2_zero
        target_position = Params.sim.target.position
        (rel_x, rel_y) = (target_position.x - self.position.x, target_position.y - self.position.y)
        rel_angle = (180 / pi) * (-atan2(rel_y, rel_x))
        correction = Params.normalize_angle(rel_angle - self.angle)
        # If we are pointing to the target, switch to directional mode (ticks = 1).
        if self.degraded and abs(correction) < 1 and self.ticks == 0:
            self.ticks = 1
        return correction

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
        Params.limit_velocity(self.velocity, Params.cubesat_max_velocity)
        # If we are too close to the target, backpedal faster. (Very ad hoc.)
        dist_to_target = Params.distance(self.position, Params.sim.target.position)
        # noinspection PyTypeChecker
        velocity_multiplier = max(1, (125/max(dist_to_target, 50))**2)
        self.velocity *= velocity_multiplier

    def update_target_velocity(self):
        """
        Update the target velocity--for this frame. These change
        are arbitrary and get the target to move around the screen.
        """
        # Change direction every once in a while
        if random() < Params.prob_change_vel:
            velocity_change = Params.V2(choice((-1, 1))*Params.target_vel_change,
                                        choice((-1, 1))*Params.target_vel_change)
            self.velocity += velocity_change
            Params.limit_velocity(self.velocity, Params.target_max_velocity)

        # If too far away from CubeSat, reverse direction
        if Params.distance(self.position, Params.sim.cubesat.position) > Params.window_width * 0.7:
            self.velocity *= -1

        # Ensure that the target is moving at a reasonable speed.
        # Allow it to move faster than CubeSat: 1.9 to 1.
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
        if Params.recentering or Params.recenter_mode and Params.at_edge_of_screen(self.position):
            Params.recentering = True
            Params.recenter_all()
        else:
            self.position += self.velocity


class Sim:

    def __init__(self):
        # Must do these after defining Params
        Params.screen_center = Params.V2(Params.window_width, Params.window_height) / 2
        Params.v2_zero = Params.V2(0, 0)

        # Make this Sim object itself available in the Params class. The two
        # satellites use it to retrieve information about each other.
        Params.sim = self
        
        pygame.init()
        pygame.display.set_caption("CubeSat Simulator")
        window_dimensions = (Params.window_width, Params.window_height)
        self.screen = pygame.display.set_mode(window_dimensions)
        self.clock = pygame.time.Clock()

        # Use a square with an arrow for CubeSat
        self.cubesat = Satellite(image="CubeSat.png", degraded=True)
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
        if Params.recentering:
            font = pygame.font.Font(None, 36)
            text = font.render("Recentering", 1, (150, 250, 250))
            self.screen.blit(text, Params.V2(50, 50))
        self.add_obj_to_screen(self.target)
        self.add_obj_to_screen(self.cubesat)
        pygame.display.flip()

    def run(self):
        """ The main loop. """
        while not self.exit:
            # Event queue.  Not used here, but standard in pygame applications.
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            # CubeSat does not have a rotational velocity. It is always at a fixed angle,
            # which changes frame-by-frame.
            if not Params.recentering and (not self.cubesat.degraded or self.cubesat.allow_angle_correction()):
                cubesat_angle_correction = self.cubesat.cubesat_angle_correction()
                self.cubesat.update_cubesat_angle(cubesat_angle_correction)

            if not self.cubesat.degraded or self.cubesat.allow_velocity_correction():
                cubesat_velocity_correction = self.cubesat.cubesat_velocity_correction()
                self.cubesat.update_velocity(cubesat_velocity_correction)
            if self.cubesat.ticks > 0:
                self.cubesat.ticks = (self.cubesat.ticks + 1) % Params.directional_ticks_limit
            if not self.cubesat.degraded or self.cubesat.allow_posiion_change():
                self.cubesat.update_position()

            self.target.update_velocity(None)
            self.target.update_position()

            self.refresh_screen()
            self.clock.tick(Params.FPS)
        pygame.quit()


if __name__ == '__main__':
    Sim().run()
