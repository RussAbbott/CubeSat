
from math import atan2, copysign, pi, sqrt
from os import path
from random import choice, random, uniform

import pygame
from pygame.math import Vector2

"""
This code includes two primary classes.
Satellite: A class of which both CubeSat and Target are subclasses. ImpairedCubeSat is a subclass of CubeSat.
Sim: The simulation infrastructure
"""


class Satellite:

    # The simulation object itself is available here. It includes references to both
    # CubeSat and the target
    sim = None

    def __init__(self, pos=None, vel=None, angle=None, image=None):
        
        # pos, vel, and angle are with respect to the fixed window/plane
        self.position = pos if pos is not None else \
                        Sim.V2(uniform(100, Sim.window_width-100), uniform(100, Sim.window_height-100))
        self.velocity = vel if vel is not None else Sim.V2(uniform(-1, 1), uniform(-1, 1))
        self.angle = angle if angle is not None else uniform(-180, 180)

        current_directory = path.dirname(path.abspath(__file__))
        image_path = current_directory + '/' + 'images/' + image
        self.image = pygame.image.load(image_path)

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

    def update(self):
        """ Update CubeSat differently from the target. """
        pass

    def update_angle(self, correction):
        """
        Update the satellite's heading (it's angle). Limit the maximum
        change that can occur in any frame. If the entire correction
        does not occur during the current frame, it will continue
        (possibly adjusted) in the next frame.
        """
        if abs(correction) > CubeSat.cubesat_max_angle_change:
            correction = copysign(CubeSat.cubesat_max_angle_change, correction)
        new_angle = Satellite.normalize_angle(self.angle + correction)
        self.angle = new_angle

    def update_velocity(self, correction):
        """ Update CubeSat's velocity differently from the target's velocity. """
        pass

    def update_position(self):
        if Sim.recentering or Sim.recenter_mode and Sim.at_edge_of_screen(self.position):
            Sim.recentering = True
            Sim.recenter_all()
        else:
            self.position += self.velocity


class CubeSat(Satellite):

    # pygame uses degrees rather than radians
    cubesat_max_angle_change = 0.5  # degrees/frame
    cubesat_max_velocity = 1.0  # pixels / frame

    def __init__(self, pos=None, vel=None, angle=None, image='CubeSat.png'):
        super().__init__(pos, vel, angle, image)

    def angle_correction(self):
        """ Compute CubeSat angle correction for the current frame update. """
        # if self.impaired:
        #     self.velocity = Sim.v2_zero()
        target_position = Satellite.sim.target.position
        (rel_x, rel_y) = (target_position.x - self.position.x, target_position.y - self.position.y)
        rel_angle = (180 / pi) * (-atan2(rel_y, rel_x))
        correction = Satellite.normalize_angle(rel_angle - self.angle)
        return correction

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
        dist_to_target = Sim.distance(self_position, target_position)
        repulsive_force = 1E9/dist_to_target**4
        return repulsive_force

    def update(self):
        """
        Update CubeSat's angle, velocity, and position.
        CubeSat does not have a rotational velocity. It is always at a fixed angle,
        which changes frame-by-frame.
        """
        if not Sim.recentering:
            angle_correction = self.angle_correction( )
            self.update_angle(angle_correction)

        velocity_correction = self.velocity_correction( )
        self.update_velocity(velocity_correction)
        self.update_position( )

    def update_velocity(self, correction):
        """
        Update CubeSat's velocity. Limit the maximum change that
        can occur in any frame. If the entire correction does not
        occur during the current frame, it will continue (possibly
        adjusted) in the next frame. Keep a safe distance from
        the target.
        """
        self.velocity += correction
        Satellite.limit_velocity(self.velocity, CubeSat.cubesat_max_velocity)
        # If we are too close to the target, backpedal faster. (Very ad hoc.)
        dist_to_target = Sim.distance(self.position, Satellite.sim.target.position)
        # noinspection PyTypeChecker
        velocity_multiplier = max(1, (125/max(dist_to_target, 50))**2)
        self.velocity *= velocity_multiplier

    def velocity_correction(self):
        """ Compute CubeSat velocity correction for the current frame update. """
        target_position = Satellite.sim.target.position
        desired_direction = target_position - self.position
        correction = desired_direction - self.velocity

        # To maintain a distance, act as if there is a repulsive force from target.
        repulsive_force = CubeSat.target_repulsive_force(self.position, target_position)
        move_toward_target = 1 - repulsive_force
        correction = move_toward_target * correction
        return correction


class ImpairedCubeSat(CubeSat):

    # For when in impaired mode
    directional_ticks_limit = 50

    def __init__(self, pos=None, vel=None, angle=None, image='CubeSat.png'):
        self.ticks = 0
        super().__init__(pos, vel, angle, image)

    def allow_angle_correction(self):
        return self.ticks == 0

    def allow_position_change(self):
        return self.ticks > 0

    def allow_velocity_correction(self):
        return 0 < self.ticks < 6

    def angle_correction(self):
        """ Compute CubeSat angle correction for the current frame update. """
        self.velocity = Sim.v2_zero()
        correction = super().angle_correction()
        # If we are pointing to the target, switch to directional mode (ticks = 1).
        if abs(correction) < 1 and self.ticks == 0:
            self.ticks = 1
        return correction

    def update(self):
        """
        Update CubeSat's angle, velocity, and position.
        CubeSat does not have a rotational velocity. It is always at a fixed angle,
        which changes frame-by-frame.
        """
        if not Sim.recentering and self.allow_angle_correction():
            angle_correction = self.angle_correction()
            self.update_angle(angle_correction)

        if self.allow_velocity_correction():
            velocity_correction = self.velocity_correction()
            self.update_velocity(velocity_correction)
        if self.ticks > 0:
            self.ticks = (self.ticks + 1) % ImpairedCubeSat.directional_ticks_limit
        if self.allow_position_change():
            self.update_position()


class Target(Satellite):

    # Probability of changing velocity on any frame.
    prob_velocity_change = 0.05  
    
    target_velocity_change = 1  # pixels / frame

    target_max_velocity = 1.9   # pixels / frame
    target_min_velocity = 0.75  # pixels / frame

    def __init__(self, pos=None, vel=None, image='ArUco_64_1.png'):
        super().__init__(pos, vel, image=image)

    def update(self):
        self.update_velocity()
        self.update_position()

    def update_velocity(self, _correction=None):
        """
        Update the target velocity--for this frame. These changes
        are arbitrary and get the target to move around the screen.
        """
        # Change direction every once in a while
        if random() < Target.prob_velocity_change:
            velocity_change = Sim.V2(choice((-0.3, 0.3))*Target.target_velocity_change,
                                        choice((-0.3, 0.3))*Target.target_velocity_change)
            self.velocity += velocity_change
            Satellite.limit_velocity(self.velocity, Target.target_max_velocity)

        # If too far away from CubeSat, reverse direction
        if Sim.distance(self.position, Satellite.sim.cubesat.position) > Sim.window_width * 0.7:
            self.velocity *= -1

        # Ensure that the target is moving at a reasonable speed.
        # Allow it to move faster than CubeSat.
        # The actual numbers are arbitrary.
        if abs(self.velocity.x) < Target.target_min_velocity:
            self.velocity.x *= 2
        if abs(self.velocity.x) > Target.target_max_velocity:
            self.velocity.x *= 0.7
        if abs(self.velocity.y) < Target.target_min_velocity:
            self.velocity.y *= 2
        if abs(self.velocity.y) > Target.target_max_velocity:
            self.velocity.y *= 0.7


class Sim:

    FPS = 50

    # Window dimensions, in pixels. These are also plane dimensions and are taken as plane units.
    # As both window and plane dimensions. (0,0) is at the upper left.
    window_width = 800   # pixels
    window_height = 800  # pixels

    # recenter_mode:    False: the objects are unable to penetrate the window borders
    #                   True: the object are recentered in the window when they approach a window border
    recenter_mode = True

    # Recentering is occuring.
    recentering = False

    # How close (in pixels) recentering must get to satisfy recentering.
    centered_enough = 50

    def __init__(self):
        # Make this Sim object itself available in the Params class.
        # The satellites use it to retrieve information about each other.
        # (That's a cheat.)
        Satellite.sim = self
        
        pygame.init()
        pygame.display.set_caption("CubeSat Simulator")
        window_dimensions = (Sim.window_width, Sim.window_height)
        self.screen = pygame.display.set_mode(window_dimensions)
        self.clock = pygame.time.Clock()
        self.exit = False

    def add_obj_to_screen(self, obj):
        """ Update the screen "surface" before displaying it. """
        obj_display = pygame.transform.rotate(obj.image, obj.angle)
        rect = obj_display.get_rect()
        self.screen.blit(obj_display, obj.position - (rect.width/2, rect.height/2))

    @staticmethod
    def at_edge_of_screen(position):
        return not (50 < position.x < Sim.window_width-50) or not (50 < position.y < Sim.window_height-50)

    @staticmethod
    def distance(a, b):
        return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    @staticmethod
    def recenter_all():
        """
        Take one step (frame) in the recentering process.
        """
        (pos1, pos2) = (Satellite.sim.cubesat.position, Satellite.sim.target.position)
        center_point = (pos1 + pos2) / 2
        correction = Sim.screen_center() - center_point
        if correction.magnitude() < 5:
            Sim.recentering = False
            return
        max_speed = 2.5*min(CubeSat.cubesat_max_velocity, Target.target_max_velocity)
        while abs(correction.x) > max_speed or abs(correction.y) > max_speed:
            correction *= 0.9
        pos1 += correction
        pos2 += correction

    def refresh_screen(self):
        """
        Refresh the screen. Create a black background and
        put the two objects in the surface. Then make it visible.
        """
        self.screen.fill((0, 0, 0))
        if Sim.recentering:
            font = pygame.font.Font(None, 36)
            text = font.render("Recentering", 1, (150, 250, 250))
            self.screen.blit(text, Sim.V2(50, 50))
        # Put the target in front of cubesat.
        self.add_obj_to_screen(self.cubesat)
        self.add_obj_to_screen(self.target)
        pygame.display.flip()

    @staticmethod
    def roundV2(v2: Vector2, prec=2):
        return Sim.V2(round(v2.x, prec), round(v2.y, prec))

    # noinspection PyAttributeOutsideInit
    def run(self):
        """ Create CubeSat and the target and run the main loop. """
        self.cubesat = CubeSat()
        # self.cubesat = ImpairedCubeSat()
        self.target = Target()

        while not self.exit:
            # Event queue.  Not used here, but standard in pygame applications.
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            self.cubesat.update()
            self.target.update()

            self.refresh_screen()
            self.clock.tick(Sim.FPS)

        pygame.quit()

    @staticmethod
    def screen_center():
        """
        Can't make this a class constant because it uses a class function.
        (There's probably a better way.)
        """
        return Sim.V2(Sim.window_width, Sim.window_height) / 2

    @staticmethod
    def V2(x, y):
        # noinspection PyArgumentList
        return Vector2(float(x), float(y))

    @staticmethod
    def v2_zero():
        """
        Can't make this a class constant because it uses a class function.
        (There's probably a better way.)
        """
        return Sim.V2(0, 0)


if __name__ == '__main__':
    Sim().run()
