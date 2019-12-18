
from math import atan2, copysign, pi, sqrt
from os import path
from random import random, uniform

import pygame
from pygame.math import Vector2

"""
This code includes two primary classes.
Satellite: A class of which both CubeSat and Target are subclasses. ImpairedCubeSat is a subclass of CubeSat.
Sim: The simulation infrastructure
"""


class Satellite:

    max_angle_change = 1  # degrees/frame
    sat_number = 0

    def __init__(self, pos=None, vel=None, angle=None, image=None):
        
        # pos, vel, and angle are with respect to the fixed window/plane
        self.position = pos if pos is not None else \
                        Sim.V2(uniform(100, Sim.window_width-100), uniform(100, Sim.window_height-100))
        self.velocity = vel if vel is not None else Sim.V2(uniform(-1, 1), uniform(-1, 1))
        self.angle = angle if angle is not None else uniform(-180, 180)

        current_directory = path.dirname(path.abspath(__file__))
        image_path = current_directory + '/' + 'images/' + image
        self.image = pygame.image.load(image_path)
        Satellite.sat_number += 1
        self.id = 'Tgt' if isinstance(self, Target) else (
                  'I-' if isinstance(self, ImpairedCubeSat) else
                  'C-') + str(self.sat_number)

    @staticmethod
    def limit_velocity(v2, max_velocity):
        magnitude = v2.magnitude()
        if magnitude > max_velocity:
            v2 *= max_velocity/magnitude
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
        if abs(correction) > Satellite.max_angle_change:
            correction = copysign(Satellite.max_angle_change, correction)
        new_angle = Satellite.normalize_angle(self.angle + correction)
        self.angle = new_angle

    def update_velocity(self, correction):
        """ Update CubeSat's velocity differently from the target's velocity. """
        pass

    def update_position(self):
        self.position += self.velocity
        if Sim.sim.print_ids:
            if isinstance(self, Target):
                print()
            print(f'{self.id}: {Sim.roundV2(self.position)}; ', end='')


class CubeSat(Satellite):

    # pygame uses degrees rather than radians
    # cubesat_max_angle_change = 0.5  # degrees/frame
    max_velocity = 1.0  # pixels / frame

    def __init__(self, pos=None, vel=None, angle=None, image='CubeSat.png'):
        super().__init__(pos, vel, angle, image)

    def angle_correction(self):
        """ Compute CubeSat angle correction for the current frame update. """
        target_position = Sim.sim.target.position
        (rel_x, rel_y) = (target_position.x - self.position.x, target_position.y - self.position.y)
        rel_angle = (180 / pi) * (-atan2(rel_y, rel_x))
        correction = Satellite.normalize_angle(rel_angle - self.angle)
        return correction

    def repulsive_force(self, other):
        """
        Compute a virtual repulsive force from the other satellite so that
        CubeSat maintains a reasonable distance. The force decreases with
        the square of the distance between them. The actual numbers are
        arbitrary.  Doing it this way makes the relationship between the two
        smoother and more fluid. An alternative might have been to keep CubeSat
        a fixed distance away from the target.
        """
        dist_to_other = Sim.distance(self.position, other.position)
        # Don't divide by 0 if self_position == other_position (or if very close)
        limited_dist_to_target = max(100.0, dist_to_other)
        # Divide by 100 (or some other arbitrary number) to scale repulsive
        # force to distance units. Could let the Target have a stronger repulsive
        # force than the other CubeSats.
        divisor = 100  # if isinstance(other, Target) else 80
        repulsive_force = 1/(limited_dist_to_target/divisor)**2
        return repulsive_force

    def stay_away_from_other_sats(self):
        repulsive_aggregate = Sim.v2_zero()
        if isinstance(self, ImpairedCubeSat) and self.ticks > ImpairedCubeSat.directional_ticks_limit:
            return repulsive_aggregate
        for sat in Sim.sim.sats:
            direction = self.position - sat.position
            repulsive_aggregate += direction * self.repulsive_force(sat)
        return repulsive_aggregate

    def update(self):
        """
        Update CubeSat's angle, velocity, and position.
        CubeSat does not have a rotational velocity. It is always at a fixed angle,
        which changes frame-by-frame.
        """
        angle_correction = self.angle_correction( )
        self.update_angle(angle_correction)

        velocity_correction = self.velocity_correction( )
        self.update_velocity(velocity_correction)
        self.update_position()

    def update_velocity(self, correction):
        """
        Update CubeSat's velocity. Limit the maximum change that
        can occur in any frame. If the entire correction does not
        occur during the current frame, it will continue (possibly
        adjusted) in the next frame. Keep a safe distance from
        the target.
        """
        self.velocity += correction
        Satellite.limit_velocity(self.velocity, CubeSat.max_velocity)

    def velocity_correction(self):
        """ Compute CubeSat velocity correction for the current frame update. """
        target_position = Sim.sim.target.position
        desired_direction = target_position - self.position
        correction = desired_direction - self.velocity
        correction += self.stay_away_from_other_sats()
        return correction


class ImpairedCubeSat(CubeSat):

    directional_ticks_limit = 20

    def __init__(self, pos=None, vel=None, angle=None, image='CubeSatImpaired.png'):
        self.ticks = 0
        super().__init__(pos, vel, angle, image)

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
        # Update the angle when ticks == 0. Keep it at 0 until angle is correct.
        if self.ticks == 0:
            angle_correction = super().angle_correction( )
            self.update_angle(angle_correction)
            # If we are pointing to the target, switch to directional mode (ticks = 1).
            if abs(angle_correction) < 1:
                self.ticks = 1
            return

        # Otherwise, update the velocity and position for ImpairedCubeSat.directional_ticks_limit
        velocity_correction = self.velocity_correction()
        self.update_velocity(velocity_correction)
        self.update_position()
        self.ticks = (self.ticks + 1) % ImpairedCubeSat.directional_ticks_limit


class Target(Satellite):

    # Probability of changing velocity on any frame.
    prob_velocity_change = 0.05  
    
    velocity_change = 1.0  # pixels / frame

    max_velocity = 1.9   # pixels / frame
    min_velocity = 0.75  # pixels / frame

    def __init__(self, pos=None, vel=None, image='ArUco_64_1.png', fixed=False):
        self.fixed = fixed
        if fixed:
            vel = Sim.v2_zero()
        super().__init__(pos, vel, image=image)

    def update(self):
        if not self.fixed:
            self.update_velocity( )
        self.update_position()

    def update_velocity(self, _correction=None):
        """
        Update the target velocity--for this frame. These changes
        are arbitrary and get the target to move around the screen.
        """
        # Change direction every once in a while
        if random() < Target.prob_velocity_change:
            velocity_change = Sim.V2(uniform((-1)*Target.velocity_change, Target.velocity_change),
                                     uniform((-1)*Target.velocity_change, Target.velocity_change))
            self.velocity += velocity_change
            Satellite.limit_velocity(self.velocity, Target.max_velocity)

        # If too far away from any CubeSat, reverse direction
        if max([Sim.distance(self.position, cubesat.position) for cubesat in Sim.sim.cubesats]) > \
            Sim.window_width * 0.6:
            self.velocity *= -0.01  # uniform(-0.2, -0.2)

        # Ensure that the target is moving at a reasonable speed.
        # Allow it to move faster than CubeSat.
        # The actual numbers are arbitrary.
        if abs(self.velocity.x) < Target.min_velocity:
            self.velocity.x *= 2
        if abs(self.velocity.x) > Target.max_velocity:
            self.velocity.x *= 0.7
        if abs(self.velocity.y) < Target.min_velocity:
            self.velocity.y *= 2
        if abs(self.velocity.y) > Target.max_velocity:
            self.velocity.y *= 0.7


class Sim:

    FPS = 50

    # Window dimensions, in pixels. These are also plane dimensions and are taken as plane units.
    # As both window and plane dimensions. (0,0) is at the upper left.
    window_width = 800   # pixels
    window_height = 800  # pixels

    # Recentering is occuring.
    recentering = False

    # How close (in pixels) recentering must get to satisfy recentering.
    centered_enough = 50

    # The simulation object itself is available here. It includes references to both
    # CubeSat and the target
    sim = None

    def __init__(self, print_ids=False):
        # Make this Sim object itself available in the Sim class.
        # The satellites use it to retrieve information about each other.
        # (That's a cheat.)
        Sim.sim = self
        self.print_ids = print_ids
        
        pygame.init()
        pygame.display.set_caption("CubeSat Simulator")
        window_dimensions = (Sim.window_width, Sim.window_height)
        self.screen = pygame.display.set_mode(window_dimensions)
        self.clock = pygame.time.Clock()
        self.exit = False

        self.cubesats = None
        self.stats = None
        self.target = None

    def add_obj_to_screen(self, obj):
        """ Update the screen "surface" before displaying it. """
        obj_display = pygame.transform.rotate(obj.image, obj.angle)
        rect = obj_display.get_rect()
        self.screen.blit(obj_display, obj.position - (rect.width/2, rect.height/2))

    @staticmethod
    def at_edge_of_screen(sat):
        """
        This is defined in Sim rather than in Satellite because it concerns
        the window within which the simulation is run, not the satellite, which
        may or may not be within the window boundaries.
        """
        return sat.position.x < 50 or sat.position.x > Sim.window_width-50 or \
               sat.position.y < 50 or sat.position.y > Sim.window_height - 50

    @staticmethod
    def distance(a, b):
        return sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    @staticmethod
    def recenter_all():
        """
        Take one step (frame) in the recentering process.
        """
        sum_positions = Sim.v2_zero()
        for sat in Sim.sim.sats:
            sum_positions += sat.position
        center_point = sum_positions/(1+len(Sim.sim.cubesats))
        correction = Sim.screen_center() - center_point
        if max([abs(correction.x), abs(correction.y)]) < 5:
            Sim.recentering = False
            return
        max_speed = 2.5*min(CubeSat.max_velocity, Target.max_velocity)
        max_dir_speed = max([abs(correction.x), abs(correction.y)])
        correction *= max_speed/max_dir_speed
        for sat in Sim.sim.sats:
            sat.position += correction

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
        for sat in self.sats:
            self.add_obj_to_screen(sat)
        pygame.display.flip()

    @staticmethod
    def roundV2(v2: Vector2, prec=2):
        return Sim.V2(round(v2.x, prec), round(v2.y, prec))

    # noinspection PyAttributeOutsideInit
    def run(self, cubesats, target=None):
        """ Create CubeSat and the target and run the main loop. """
        self.cubesats = cubesats
        self.target = target if target else Target()
        # Displays the satellites in front of the target
        self.sats = [self.target] + self.cubesats

        while not self.exit:
            # Event queue.  Not used here, but standard in pygame applications.
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            if not Sim.recentering and any(Sim.at_edge_of_screen(sat) for sat in self.sats):
                Sim.recentering = True

            if Sim.recentering:
                Sim.recenter_all()
            else:
                for sat in self.sats:
                    sat.update()

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
    # Displays the unimpaired CubeSat in front of the impaired CubeSate
    Sim(print_ids=False).run([CubeSat(), ImpairedCubeSat(), CubeSat()], Target(fixed=True))
